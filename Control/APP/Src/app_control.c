#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "app_control.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_key.h"
#include "bsp_voice.h"
#include "bsp_bluetooth.h"
#include "bsp_receiverk210.h"
#include "bsp_systimer.h"
#include "bsp_watchdog.h"
#include "bsp_indicatorled.h"
#include "bsp_gyro.h"
#include "mid_pid.h"
#include "mid_protocol.h"
#include "Debug.h"

/* ===== 弯道状态机分两半（读这个文件之前先看这段）=========================
 * 30ms 任务（有视觉帧才会跑）：决定「要不要认这个弯」
 *     —— 查里程门禁、查冷却、置 Corner_Armed
 *  5ms  任务（每 5 毫秒，不依赖视觉）：决定「什么时候真的开始转、转够了没有」
 *     Corner_Armed --走够 TURN_ARM_CM--> Corner_Active --陀螺仪量够角度--> Corner_Done
 *
 * 为什么必须分两半：
 *   弯里 47.5% 的帧好行=0（11-20 日志统计），视觉一黑，30ms 任务第一句就
 *   `if (got == 0) return;` 直接退出；而转弯停不下来会一直转（240 度/秒）。
 *   所以「停」必须由 5ms 的编码器/陀螺仪说了算，不能挂在视觉上。
 *
 * 为什么"认不认"在 30ms：只有拿到视觉帧才知道 state 是几，没别的办法。
 *
 * 目标速度 target_left/right 目前有 6 处赋值（5ms 一处、30ms 五处），
 * 见下方各自的行尾注释。这是下一版要合并的地方。
 * ====================================================================== */

static void Task_StartStopCmd(void);
static void Task_CarStop(void);
static void Task_BtCommand(void);
static void Task_SendTelemetry(void);
static uint8_t Task_FetchVisionData(VisionData_t *vis_data);
static void Task_30ms_Control(VisionData_t *vis_data);
static void Task_Indicate(void);
static void CarRun_Set(uint8_t run);
static uint8_t Task_EncoderCalib(void);
static void Task_5ms_CornerStep(uint32_t curr_ms);
static void Corner_SetTarget(uint8_t kind);
static uint8_t Task_CornerDecide(uint8_t state);
static uint8_t Corner_TurnAngleReached(void);
static uint8_t Task_5ms_FinishStep(void);
static uint8_t Task_5ms_StallGuard(uint32_t curr_ms);

static uint32_t task_last_5ms = 0;//5ms时间片
static uint32_t task_last_10ms = 0;//10ms时间片
static uint32_t task_last_30ms = 0;//30ms时间片
static uint32_t task_last_100ms = 0;//100ms时间片

static pid_velocity_t pid_v_left;//左速度环pid结构体
static pid_velocity_t pid_v_right;//右速度环pid结构体
static pid_line_t pid_l;//方向环结构体

static uint8_t CarRun_State = 0;//启停状态变量

static float Speed_Straight   = 20.0f; //直道基速(电机上限37，留7个count给差速)  ← 用户手改(原30)，方便测试不用每次发蓝牙
static float Speed_SmallAngle = 20.0f; //小角度(直角,钝角)基速  ← 用户手改(原25)
static float Speed_LargeAngle = 15.0f; //大角度(锐角)基速
static float target_left = 0.0f;//目标左轮速度
static float target_right = 0.0f;//目标右轮速度
static float actual_left = 0.0f;//实际左轮速度
static float actual_right = 0.0f;//实际右轮速度
static float last_vision_error = 0.0f; // 用于记录上一次的视觉误差，给 100ms 发波形

/* 弯道转向：用编码器脉冲累计差分量角度（2026-11-20 实车改造）（详见 Doc/代码说明.md §1）
 * 2026-11-23 深夜重新标定：1730 → **1487**。
 * 依据是两次实测（陀螺仪这两天不可靠，只能靠编码器）：
 *   2400 计数 → 车实际转约 145°；1817 计数 → 约 110°。
 *   两次相除：2400/145 = 16.55、1817/110 = 16.52 → **16.5 计数/度**，两条线对得很齐。
 *   所以 90° = 90 × 16.53 ≈ 1487 计数。
 * 原来填 1730 是按「1230 × 90/64」推的旧标定（说的是内轮被拖着走、编码器多算），
 *   和现在这组实车数据差了 16%，导致每次转弯都多转 16°。 */
#define TURN_DIFF_TARGET   1487.0f  // 转 90 度需要累计的(左-右)counts（16.5 计数/度）

/* 转弯结束条件：用陀螺仪量出来的角度，还是数编码器计数（详见 Doc/代码说明.md §2） */
#define TURN_USE_GYRO_ANGLE  1      /* 1 = 转到目标角就停；0 = 老办法数编码器 */
#define TURN_TARGET_ANGLE_DEG 90.0f /* 默认目标角（度）。真正用哪个由下面的表决定 */

/* 里程查表：按「这是第几个要甩的弯」决定转多少度   2026-11-21 新增（详见 Doc/代码说明.md §3） */
#define TURN_USE_MILEAGE_TABLE  1//里程校验开关
#define TURN_USE_38_BENDS       1//决定钝角弯要不要也要当弯道处理（2026-11-23 改 1：钝角弯靠 PD 太勉强）
/* 决定是否用「里程」来钉住开转点。
 * 2026-11-23 夜由 0 改成 1，之后又从「兜底」升级成「最迟开转点」（方案 B，详见下面 TURN_FORCE_CM）。
 * 打开以后的行为：段内里程一到「阈值 + TURN_FORCE_CM」就无条件转，不看视觉、也不等那 16 厘米。 */
#define TURN_ALLOW_FORCE        1

/* 死区：里程离本段阈值太远，就当视觉在报假信号，不转。（详见 Doc/代码说明.md §4） */
#define TURN_DIST_EARLY_CM      30.0f   /* 允许比阈值早这么多厘米（门就开在这里） */
/* 2026-11-23：试过改成 45，结果 1 号弯在 135 厘米就开始接受报弯 → 提前开转 → 掉头。
 * 早侧这道闸是全局的，不能为了救一个弯放宽。锐角弯改成靠「阈值 325→299」解决：
 * 阈值降下来以后门自然就开得早（269 厘米），视觉 267 厘米的报弯下一帧就能进。 */
#define TURN_DIST_LATE_CM       10.0f   /* 允许比阈值晚这么多厘米（只给刻度误差留一点） */

/* 【最迟开转点】段内里程超过「阈值 + 这个数」就无条件转（要 TURN_ALLOW_FORCE = 1 才生效）
 * 2026-11-23 夜：30 → 0 → **−10.7**（方案 B：用它把开转点钉死）
 * 为什么是 −10.7：理想开转点 = 弯心 − R = 阈值 − 10.7。
 *   R = 10.7 是实测转弯半径（转弯 0.4 秒 × 42.1 厘米/秒 = 弧长 16.8 厘米，除 π/2）。
 *   弯心就是这里的「阈值」（段内里程从上一个弯转完算起，阈值 = 到下一个弯心的距离）。
 * 填 30（原来是兜底）→ 比理想点晚 41 厘米，等于没救；填 0 → 晚 11 厘米。
 * 填 −10.7 → 正好落在理想点，开转点 = 阈值 − 10.7，一个弯一个数，和视觉报弯早晚无关。
 * 注意 103° 那个弯（第 3 个）转得久、R 约 14，理想点略早于 阈值 − 10.7；
 *   要单独修就改那个弯的阈值（调小 3~4 厘米），不用动这里。 */
#define TURN_FORCE_CM           (-10.7f)

/* 第二道闸（2026-11-21 新增）：编码器兜底。
 * 2026-11-24 凌晨：比例 1.05 → **3.0**。
 * 为什么必须放宽 —— 抓包实测（lap_new_07）：
 *   1 号弯 7.6 秒开转，8.0 秒就停了，陀螺仪只走到 **−42.8°**（命令 78°）；
 *   钝角弯 10.2 秒开转，10.3 秒就停，陀螺仪只走到 **+1.6°** —— 等于根本没转。
 *   两次都是这行编码器兜底先到（1353 / 581 计数），陀螺仪还没到角度就被掐了。
 * 原因：转弯是“原地打转”，外侧轮拖着内侧轮，轮胎在打滑 —— 编码器数出来的
 *   转角大约是真值的 **2 倍**（实测 82° 计数 vs 43° 真转角）。
 *   所以这个兜底只要按“理论计数”设，就一定会抢在陀螺仪前面，把弯转成半个。
 * 现在陀螺仪已经可靠（软件 I2C 降速后没再 −999），让它做主；
 *   编码器兜底放到 3 倍，只在“陀螺仪彻底冻住”时才会到，同时保留 600 毫秒超时兜底。 */
#define TURN_COUNT_BACKSTOP_RATIO  3.0f

/* ===== 弯道速度表（2026-11-21）=====
 * 直道 30 档，一看到弯就降到 15（用视觉报弯到弯口那 20 厘米当刹车距离），
 * 转弯本身也是 15 档，出弯先保持 20 档跑一段再回 30。 */
#define Speed_CornerExit    20.0f   /* 出弯后交接期间的速度，交接完才回 Speed_Straight */

#if TURN_USE_38_BENDS
  #define TURN_TABLE_SIZE 5
  /* 2026-11-23 启用「钝角弯也当弯道」后，这张表真正生效。第一项从 200 改成 180：
   * 三条实车数据都指向第一个弯心在 180（报弯 151~154、开转 170.6~171.7 都吻合）。
   * 后四项还是几何值：四段 110 / 240 / 240 / 110（按弯心到弯心），等实测替换。 */
  static const float Turn_ThresholdCm[TURN_TABLE_SIZE] = { 189.0f, 110.0f, 240.0f, 240.0f, 110.0f };
  /* 第 1 项 180 → 200 → 197 → 189（2026-11-24 凌晨，实车一次一次标定）。
   * 实测记录：
   *   阈值 180 → 开转 169.3 厘米 → 用户「早约一拃（20 厘米）」
   *   阈值 200 → 开转 189.3 厘米 → 用户「略晚一点点」
   *   阈值 197 → 开转 186.3 厘米 → 用户「还是晚了」→ 用户直接指定再减 8
   * 说明：我按「每 1 厘米阈值 = 1.15 厘米车身」插值算出的 197 被实测否掉了，
   *   实际灵敏度比这更低（要减 8 才够）。所以**别再用插值，直接按用户的话加减**。
   * 后面四项 {110,240,240,110} 全是图纸值，同样要一个一个这样试出来。 */
  /* 2026-11-23 深夜：全部乘 0.87（90→78、38.5→33.5、103→90）。
   * 原因：差速撤掉那一刻车身还有约 240 度/秒的角速度，靠惯性会再多转一段。
   * 实测（陀螺仪数据）：命令 90° 停到 −85、峰值 −103 → 多转 18°；
   *   38.5° 那个弯停到 −32 附近、峰值 −45 → 多转 7°。大约是指令角的 13~20%。
   * 所以指令先少给 13%，加上惯性正好落在目标角上。
   * 注意：这个系数是"整车+地面+电量"的综合结果，换场地要重新验。 */
  static const float Turn_AngleDeg[TURN_TABLE_SIZE]     = {  78.0f,  33.5f,  90.0f,  33.5f,  78.0f };
#else
  #define TURN_TABLE_SIZE 3
  /* 阈值单位：厘米，从「起点 / 上一个弯转完」算起。（详见 Doc/代码说明.md §6） */
  /* 第三项暂时不动；第一项 180 和实测吻合（车在 170.7 开转）；第二项 325 → 299，
   * 依据：视觉在 267 厘米报弯、提前量和 1 号弯一样约 27 厘米 → 真实弯心约 294~299。 */
  static const float Turn_ThresholdCm[TURN_TABLE_SIZE] = { 180.0f, 299.0f, 330.0f };
  static const float Turn_AngleDeg[TURN_TABLE_SIZE]     = {  90.0f, 103.0f,  90.0f };
#endif

#define TURN_SPEED         15.0f    // 转弯时基速(= Speed_LargeAngle)
#define TURN_VALUE         11.55f   // 转弯时差速(= Kp 0.231 * 视觉固定值 50)
#define TURN_TIMEOUT_MS    600      // 兜底:0.6秒还没转够就强制结束(陀螺仪冻住时才会到)
                                    /* 2026-11-24 凌晨 800→600：陀螺仪做主以后，这个只是
                                     * 「角度一直不涨」时的最后一道闸。90° 弯约 0.33 秒、
                                     * 103° 弯约 0.43 秒，600 毫秒留够了余量；
                                     * 而陀螺仪真冻住时，600 毫秒 ≈ 145°，总比 800 毫秒的 190° 好。 */
#define TURN_ARM_CM        16.0f    // 视觉报弯后,还要再直走这么多厘米才真的开始转
/* 2026-11-23 由 20 改成 16，依据是遥测实测：
 *   第一个弯：报弯 153 → 开转 173；转弯持续 0.4 秒，速度 15 档 42.1 厘米/秒
 *   → 弧长 16.8 厘米 → 半径 R = 16.8 ÷ (π/2) = 10.7 厘米
 *   要和两边直道相切，就得提前 R 开转：180 − 10.7 = 169.3 → ARM = 169.3 − 153 ≈ 16
 * 原来 20 意味着 173 才开转，比切点晚 4 厘米，拐出弯时车身已经偏到车道内侧 4~5 厘米，
 * 视觉偏差直接跳到 +26，方向环再往同一侧猛打 —— 实测车在弯口又拧出去几十度、往回跑。 */
                                    // 2026-11-21: CM_PER_COUNT 改回理论值 0.01402 后,
                                    //   这里填20 就是真走20厘米(之前填的是0.01610,实际只走17.4厘米)
                                    // 参考:标定前填30时, 实际约走 34 厘米(用户反馈"转弯略晚")
                                    // 太早就加大(24/28), 冲出去就减小
#define CM_PER_COUNT       0.01402f // 1 个编码器计数 = 多少厘米
                                    /* 2026-11-21 改回【理论值】, 不用台架手推那个数:（详见 Doc/代码说明.md §7） */
#define TURN_ARM_SPEED     15.0f    // 等里程期间的直行速度:提前降到转弯基速,别带着最高速进弯
/* 等里程这 16 厘米里，方向环允许的最大差速（单位同目标速度，计数/5ms）。
 * 2026-11-23 深夜加：把方向环放回这段以后，用户反馈「弯前画龙」——
 * 原因是弯口附近九行里常常只剩 1~3 行可用，偏差本身在跳，方向环全权跟着跳就摆起来了。
 * 压到 ±3（20 档的 15%）以后，它只做「慢慢往中间靠」，不会被噪声带着甩。
 *
 * 2026-11-24 凌晨：±3 → ±1.2。用户反馈「弯前歪头」，实测算一下：
 *   差速 3、基速 15 → 左 18 右 12，偏航 = (18−12)×2.804÷13 = 74 度/秒；
 *   这 16 厘米走 0.38 秒 → 最多拧 28 度。一个弯总共才 90 度，弯前先拧 28 度太多。
 *   收到 1.2：偏航 = 2×1.2×2.804÷13 = 30 度/秒 → 0.38 秒只拧 11 度，够「慢慢靠中间」。 */
#define TURN_ARM_TURN_MAX  1.2f

/* 转完90度之后,最多再直行这么久,就得把方向盘交回视觉。（详见 Doc/代码说明.md §8） */
#define CORNER_HANDOVER_MS 500      // 转完之后最多直行 500 毫秒

/* 弯道冷却（厘米）。（详见 Doc/代码说明.md §9） */
/* 弯道冷却（厘米）。（详见 Doc/代码说明.md §9）
 * 2026-11-23 启用钝角弯（5 个弯）后从 70 降到 50：
 * 两个钝角弯之间最短一段只有 110 厘米，门开在 110−30 = 80，冷却 70 只剩 10 厘米余量；
 * 降到 50 之后余量 30 厘米，摩擦/打滑抖一下也不会把下一段的判定起点盖住。 */
#define CORNER_COOLDOWN_CM 50.0f//过弯之后再走50cm才允许再次过弯,否则不算

/* 终点兜底（2026-11-23 新增）：最后一个弯转完之后再走这么多厘米就停车。
 * 为什么需要：视觉只发 0/1/2，「case 3 终点急停」永远触发不了（详见 Doc/代码说明.md §26）。
 * 数字怎么来的：最后一段中心线 98.8 厘米，转完弯那一刻已经过去约 5 厘米 → 到起点线约 93.8；
 *   再留 5 厘米保证确实过了线 → 99。实测完按「过线位置」微调这一个数就行。 */
#define FINISH_USE_MILEAGE 1        // 里程兜底停车开关：1=开。/ 0=关（关掉就只剩视觉那条死路）
#define FINISH_STOP_CM     99.0f    // 最后一个弯转完后的停车里程（厘米）

/* 偏差限幅(像素)。（详见 Doc/代码说明.md §10） */
#define LIMIT_LINE_ERR_MAX 30.0f
/* 2026-11-23 夜：把前一天加的两条限幅（LIMIT_P_ERR_MAX = 10、TURN_RATIO_MAX = 0.35）全撤了。
 * 为什么撤：它们当初是给「拐出弯时的假偏差」打的补丁。那 ±26 并不是车真的偏了 12 厘米，
 *   而是 K210 拿「一条线 + 车道半宽」倒推出来的（run_log.txt 实测，见 fig120 / fig121）。
 * 补丁的副作用：真偏 5 厘米以上时方向环也打不动（偏差 10 像素就饱和），车一路贴着内侧线。
 * 现在改成从源头治：K210 只让「两条线都在」的行参与偏差（VisualTest/live_scan_test.py）。
 * 假数没了，就不必再压方向环 —— 回到 11-20 在直道上调定的那一版：
 *   Kp 0.231 / Kd 0.53，D 项按帧限幅 8 像素（见 MID/Src/mid_pid.c 的 LIMIT_LINE_DIFF_MAX）。 */

/* ===== 堵转保护参数（2026-11-23 夜新增，判据见 Task_5ms_StallGuard） ======
 * 背景：TB6612 绝对最大 3.2A(峰值)/1.2A(连续)，而 12.6V 加在堵转的绕组上是 6~12A。
 *   实测抓包 lap_new_01 里有三段「目标 20、实际 0」（1.4 / 5.5 / 5.2 秒），程序毫无反应。 */
#define STALL_TARGET_MIN    15.0f   /* 有轮子被要求跑到这么多以上才管（单位：计数/5ms） */
#define STALL_SPEED_MAX     1.5f    /* 两轮平均低于这么多算「不走」（1.5×0.01402÷0.005 = 4.2 厘米/秒） */
#define STALL_DUTY_MIN      999.0f  /* 速度环输出（‰）正向顶到这么多算「还在死推」 */
#define STALL_CONFIRM_MS    400     /* 连续这么多毫秒都符合才算真堵转 */
#define STALL_GRACE_MS      500     /* 每次启动后这么久之内不检测（挡起步误杀） */

static uint32_t Stall_Tick     = 0;    /* 这一轮「疑似堵转」是从什么时候开始的 */
static uint8_t  Stall_Running  = 0;    /* 1=上一拍已经在「疑似堵转」里，正在计时 */
static uint32_t Stall_RunStart = 0;    /* 这一趟是什么时候开始跑的，宽限期用 */
static uint8_t  Stall_RunPrev  = 0;    /* 上一拍的启停状态，用来抓 0→1 那个边沿 */
static uint8_t  Stall_CutTimes = 0;    /* 断过几次（调试器里能看，台架上用得到） */

static uint8_t  Corner_Armed  = 0;    // 1=视觉已经报弯,正在等里程
static uint8_t  Corner_Active = 0;    // 1=正在按固定差速转弯
static uint8_t  Corner_Done   = 0;    // 1=这个弯已经转够90度,别再转
static uint8_t  Corner_Clear  = 0;    // 连续多少帧视觉说"不是弯"
static float    Corner_ArmDist = 0.0f; // 报弯后已经走的距离(换算厘米)
static float    Corner_Accum  = 0.0f;  // 累计(左轮-右轮)counts, 就是转过的角度
static uint32_t Corner_Tick   = 0;     // 本次转弯开始的时刻, 供兜底超时用
static uint32_t Corner_DoneTick = 0;   // 转完的时刻, 供"限时交回视觉"用
/* 2026-11-21 合并：原来的 Corner_Cool（冷却距离）删掉了。
 * 它和 Corner_SegMileage 数的是同一件事——都是「转完到现在走了多少厘米」，
 * 唯一的差别是 Cool 涨到 70 就停。所以直接拿 SegMileage 比 70 就行，
 * 少一个变量、少一处会不一致的地方。 */
static uint8_t  Corner_Index      = 0;      // 正要去第几个要甩的弯(查表下标)
static float    Corner_SegMileage = 0.0f;   // 从上一个弯转完到现在走了多少厘米
static uint8_t  Finish_Armed      = 0;      // 1=最后一个弯已经转完,正在数停车里程（终点兜底）

/* 调试：锁存「报弯里程」和「开转里程」      2026-11-22 新增（详见 Doc/代码说明.md §11） */
static float    Corner_ArmMileage       = 0.0f;  // 视觉报弯那一刻的段内里程（厘米）
static float    Corner_TurnStartMileage = 0.0f;  // 真正开始转那一刻的段内里程（厘米）

/* 台架/调试统计量                                    （详见 Doc/代码说明.md §12） */
static float    Odo_LeftTotal      = 0.0f;   // 左轮累计计数（永不清零）
static float    Odo_RightTotal     = 0.0f;   // 右轮累计计数（永不清零）
static uint16_t VisionFrameCount   = 0;      // 已收到的视觉帧数（约 65535 溢出，36 fps 下约 30 分钟）
static uint8_t  VisionLostTicks    = 0;      // 连续没拿到视觉数据的次数（0~11）
static uint8_t  last_vision_state  = 0;      // 最近一帧的视觉状态（0/1/2/3），台架显示用
static float    Turn_TargetAngle_Deg = TURN_TARGET_ANGLE_DEG; // 本次转弯的目标角(度)

static uint8_t  Indicator_Tick    = 0;   //闪烁计数
static uint8_t  Indicator_Fault   = 0;   //视觉断线锁存, 收到新数据才清除
static uint32_t Vision_LastTick   = 0;   //最后一次收到视觉数据的时刻
static uint8_t  CarRun_PrevState  = 0;   //上一次的运行状态, 用于捕捉启动瞬间
static uint8_t  Vision_EverSeen   = 0;   //上电后有没有收到过视觉数据(没就绪前不许启动)

/* 台架：编码器里程标定（2026-11-20 深夜加）（详见 Doc/代码说明.md §13） */
static uint8_t  Calib_Mode   = 0;      // 1=标定模式
static uint8_t  Calib_Frozen = 0;      // 1=停手1秒后锁定了数值
static float    Calib_Left   = 0.0f;   // 左轮累计 counts
static float    Calib_Right  = 0.0f;   // 右轮累计 counts
static float    Calib_Cm     = 0.0f;   // 累计里程(换算厘米)
static uint32_t Calib_IdleMs = 0;      // 已经多少毫秒没有新计数


/* 陀螺仪已拆到 HardwareLayer/Gyro.c + Gyro.h（2026-11（详见 Doc/代码说明.md §14） */
#define TELEMETRY_MODE    0
//蓝牙调参范围限制(参数上限/下限)
#define LIMIT_V_KP_MAX      400.0f //内环速度环Kp上限(误差 2.5 counts 就打满，再大无意义)
#define LIMIT_V_KI_MAX      80.0f  //内环速度环Ki上限(积分时间 6.3 ms，再大必振)
#define LIMIT_LINE_KP_MAX   1.0f  //外环方向环Kp上限
#define LIMIT_LINE_KD_MAX   1.0f //外环方向环Kd上限
#define LIMIT_SPEED_MAX     37.0f //基础速度上限
//7.板载 LED 状态指示(100ms)
#define INDICATE_SLOW_TICKS     5       //慢闪: 5 x 100ms = 500ms 翻转一次
#define INDICATE_FAST_TICKS     1       //快闪: 1 x 100ms = 100ms 翻转一次
#define INDICATE_VISION_LOST_MS 330     //超过该时间没收到视觉数据即判定断线(与丢包判定一致, 11次*30ms)


void App_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Encoder_Init();
	Motor_Init();
	Key_Init();
	// 台架标定模式:上电瞬间按住开关不放就进标定
	// (按键接在 PA0, 上拉输入, 按下 = 低电平; 具体引脚见 Key.c)
	if (Key_IsDown()) Calib_Mode = 1;//按住按钮上电:进台架标定模式
	Voice_Init();
	BlueTooth_Init();
	Debug_Init();
	Indicator_Init();
	Gyro_Init();
	k210rcv_Init();
	SysTimer_Init();
	PID_VelocityLoop_Init(&pid_v_left,100.0f,10.0f,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//台架调定 Kp=100 Ki=10
	PID_VelocityLoop_Init(&pid_v_right,100.0f,10.0f,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//台架调定，左右同参数
	PID_LineLoop_Init(&pid_l,0.15f,0.53f,Speed_Straight,-(float)Speed_Straight);
	/* 方向环 Kp / Kd（2026-11-24 凌晨，第二次调）
	 *
	 * 上一版改成 0.18 / 1.10，用户实车反馈「更大了，十分严重的画龙」。
	 * 错在 Kd：偏差每帧会单帧跳 10~23 格（见 mid_pid.c 里 LIMIT_LINE_DIFF_MAX 那段实测），
	 *   Kd 从 0.53 提到 1.10，等于假跳变抽的鞭子也翻倍 —— 那不是阻尼，是往火上浇油。
	 *
	 * 这一版：
	 *   Kp 0.231 → 0.15   （收 P，画龙的"油门"。0.1 那趟太钝，所以停在这中间）
	 *   Kd 1.10  → 0.53   （退回 2026-11-20 实车调定的值）
	 *   D 的每帧限幅 8 → 4（去 mid_pid.c 改；真摆动每帧只变 3.4 格，限幅 4 不影响真阻尼，
	 *                      但能把「跳 20 格」的假信号砍掉一大半）
	 * 判据：这趟只看摆的幅度有没有变小，不看能不能跑完全程。
	 */

	//Kp 上限 ≈ 1 / (k × 死区时间)
    WatchDog_Init();
}

void AppTask_Idle(void)
{
	WatchDog_FeedDog();
}


uint32_t App_Clock(void)
{
	return SysTimer_GetTick();
}


void AppTask_5ms(uint32_t curr_ms)//内环速度环控制
{
	if(curr_ms - task_last_5ms < 5) return;
    task_last_5ms = curr_ms; //重置闹钟

	Encoder_Update();
	actual_left  = (float)Encoder_GetDeltaLeft();//读取实际左轮速度
	actual_right = (float)Encoder_GetDeltaRight();//读取实际右轮速度

	/* 陀螺仪每 5 毫秒读一次。                               （详见 Doc/代码说明.md §15） */
	Gyro_Update(curr_ms);//更新陀螺仪数据,保证后续每一次读取到的都是陀螺仪更新后的数据

	/* 台架标定模式：只数轮子、不驱动电机。返回 1 说明这一拍别跑车。 */
	if(Task_EncoderCalib()) return;

	/* 堵转保护：车被顶住/卡住时断电停车，别把驱动烧掉。（详见 Task_5ms_StallGuard）
	 * 为什么放在这个位置，是两个取舍：
	 *   1) 必须在 if(CarRun_State == 0) return 之前 —— 它靠这一行抓「0→1 刚启动」那个边沿；
	 *   2) 它读的是【上一拍】的速度环输出（5 毫秒前的值，够快）。换来的是断电动手在
	 *      这一拍算 PWM 之前，不会被下面的 Motor_SetDuty 覆盖掉。 */
	if(Task_5ms_StallGuard(curr_ms)) return;

	if(CarRun_State == 0) return;//停车模式不启动

	/* 段内里程：从「上一个弯转完」到现在走了多少厘米。（详见 Doc/代码说明.md §16） */
	Corner_SegMileage += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;

	/* 里程台架用：两轮累计计数，永不清零（段内里程每过一个弯会清零，量全程要用这个） */
	Odo_LeftTotal  += actual_left;//总左里程
	Odo_RightTotal += actual_right;//总右里程

	/* 终点兜底：最后一个弯转完之后按里程停车。
	 * 返回 1 = 这一拍已经停车了，必须马上 return —— 不然后面算出来的 PWM
	 * 会把刚写进去的 0 覆盖掉。（详见 Doc/代码说明.md §26） */
	if (Task_5ms_FinishStep()) return;

	/* 弯道转向：这一段单独成函数，见文件后半的 Task_5ms_CornerStep()。 */
	Task_5ms_CornerStep(curr_ms);

	float pwm_left = PID_V_Calc(&pid_v_left,actual_left,target_left);
	float pwm_right = PID_V_Calc(&pid_v_right,actual_right,target_right);
	Motor_SetDuty((int16_t)pwm_left,(int16_t)pwm_right);
}


void AppTask_10ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_10ms < 10) return;

    task_last_10ms = curr_ms;
    Key_CheckRequest(curr_ms);
}


void AppTask_30ms(uint32_t curr_ms)//外环方向环控制
{
	if(curr_ms - task_last_30ms < 30) return;
    task_last_30ms = curr_ms;

    if(Calib_Mode) return;//台架标定模式不需要视觉

    // 停车时也要读视觉数据。
    // 为什么:启动按键要等「收到过至少一帧视觉」才允许启动,而那个标志是在
    // Task_FetchVisionData 里置的。如果这里先 return,停车状态下永远读不到数据,
    // 标志永远置不上 —— 结果就是「怎么按都不走」(2026-11-20 夜里踩过的坑)。
    VisionData_t vis_data;
    uint8_t got = Task_FetchVisionData(&vis_data);

    if(CarRun_State == 0) return;
    if(got == 0) return;//没拿到新数据直接结束避免用旧数据算
    Task_30ms_Control(&vis_data);
}


void AppTask_100ms(uint32_t curr_ms)//开关控制与数据发送
{
	
    if(curr_ms - task_last_100ms < 100) return;
    task_last_100ms = curr_ms;

    /* 陀螺仪「静止重估零偏」只在停车时允许（2026-11-24 凌晨加）。
     * 为什么必须跟着 CarRun_State 走：车贴内侧线进弯、原地打转被卡住转不动时，
     * 陀螺仪读数就是「几乎不动」，会被误判成静止、把累计角度清零，
     * 于是永远到不了目标角度 → 转弯只能等 600 毫秒超时 → 实测转了 135 度。
     * 放在 Calib_Mode 判断之前：台架标定模式下车本来就不动，正好让它重估零偏。 */
    Gyro_SetStillTrim((CarRun_State == 0) ? 1 : 0);

    if(Calib_Mode)//台架标定模式:只管发波形,不处理启停、不做停车清理
    {
        Task_SendTelemetry();
        Task_Indicate();
        return;
    }

    Task_StartStopCmd();//1.按键启停请求
    Task_BtCommand();//2.蓝牙指令: 启停 + 调参
    if(CarRun_State == 0) Task_CarStop();//3.停车安全清理(必须放在读取指令之后)
    Task_SendTelemetry();//4.往VOFA+发送波形数据
    Task_Indicate();//5.刷新板载灯状态
}


/* ===== 弯道目标速度：全文件只有这一处给 target_left/right 写「弯道值」=====
 * 原来这段逻辑散在 6 个地方（5ms 一处、30ms 五处），读代码要来回翻两个函数。
 * 现在集中到这里，调用方只说「现在是什么情况」，不自己算速度。 */
#define CORNER_T_STRAIGHT   0   /* 当直道跑（冷却期 / 里程对不上） */
#define CORNER_T_ARM        1   /* 视觉已报弯，等里程走够 TURN_ARM_CM */
#define CORNER_T_TURN       2   /* 固定差速转 */
#define CORNER_T_EXIT       3   /* 刚转完，交接期直行 */

static void Corner_SetTarget(uint8_t kind)
{
	switch (kind)
	{
	case CORNER_T_TURN:
		target_left  = TURN_SPEED + TURN_VALUE;
		target_right = TURN_SPEED - TURN_VALUE;
		break;
	case CORNER_T_ARM:
		target_left  = TURN_ARM_SPEED;
		target_right = TURN_ARM_SPEED;
		break;
	case CORNER_T_EXIT:
		target_left  = Speed_CornerExit;
		target_right = Speed_CornerExit;
		break;
	case CORNER_T_STRAIGHT:
	default:
		target_left  = Speed_Straight;
		target_right = Speed_Straight;
		break;
	}
}


/* ===== 弯道状态机：30ms 那一半 —— 「认不认这个弯」 =====
 * 形参：state —— 视觉这一帧报的状态（2 = 有弯）
 * 返回：1 = 接管了（target_left/right 已写好，调用者直接 return）
 *       0 = 不接管，交给下面的 PD
 *
 * 它只决定「信不信这次报弯」；「什么时候真的开转、转够了没有」在 5ms 的
 * Task_5ms_CornerStep() 里。为什么必须分两半，见文件开头那段说明。 */
static uint8_t Task_CornerDecide(uint8_t state)
{
	if (state != 2)
	{
		/* 要连续 3 次方向环（3 × 30ms = 90ms）都说「不是弯」才算过去 */
		if (Corner_Clear < 3) Corner_Clear++;
		if ((Corner_Clear >= 3) && (Corner_Active == 0))
		{
			/* 正在转弯时不许被「视觉说不是弯」取消 */
			Corner_Armed   = 0;
			Corner_Done    = 0;
			Corner_ArmDist = 0.0f;
			Corner_Accum   = 0.0f;
		}
		/* 正在转弯（包括里程强转）时绝不允许落到 PD —— 否则每 30 毫秒把转弯差速覆盖掉，
		 * 车就直着冲过弯口。视觉在弯里本来就会短暂丢帧（47.5% 的帧好行=0），
		 * 这一条同时也防住了「转着转着视觉哑火」那种情况。 */
		if (Corner_Active)
		{
			Corner_SetTarget(CORNER_T_TURN);
			return 1;
		}
		return 0;                       /* 不接管，交给 PD */
	}

	Corner_Clear = 0;                   /* 视觉说这里就是弯 */

	if (Corner_Done)                    /* 这个弯已经转够了 */
	{
		Corner_Armed  = 0;
		Corner_Active = 0;
		if ((App_Clock() - Corner_DoneTick) > CORNER_HANDOVER_MS)
		{
			/* 交接期结束：这个弯彻底完事，Corner_Done 必须在这里清掉。
			 * 为什么非清不可（2026-11-23 夜修）：原来它只在下面「state != 2」那条分支里清，
			 *   结果是 —— 只要视觉一直报「有弯」（哪怕车早就转完、视觉卡住），
			 *   Corner_Clear 被反复清零、这行永远走不到，Corner_Done 就永远是 1。
			 *   而「里程强转」的第 1 条判据正是 Corner_Done == 0，
			 *   于是这一趟只转得动第一个弯，后面几个全直着冲过去。
			 * 清掉以后会不会被同一个弯骗第二次？不会：段内里程在转完那一刻已经清零，
			 *   要走够「冷却 50 厘米 + 下一段阈值 −30 厘米」才可能再被接受。 */
			Corner_Done = 0;
		}
		else
		{
			Corner_SetTarget(CORNER_T_EXIT);
			return 1;
		}
	}

	if (Corner_Active)                  /* 正在转：固定差速，不看视觉 */
	{
		Corner_SetTarget(CORNER_T_TURN);
		return 1;
	}

	if (Corner_SegMileage < CORNER_COOLDOWN_CM)     /* 上一个弯的冷却期 */
	{
		/* 2026-11-23 深夜改：这里原来直接 Corner_SetTarget(CORNER_T_STRAIGHT) 并 return 1，
		 * 等于冷却的 50 厘米里方向环完全不管。
		 * 抓包实测（用户在钝角弯后失控的那一趟）：转完 50 厘米内目标一直是 20.00 / 20.00，
		 * 而视觉偏差从 0 一路涨到 +42（= 20 厘米）—— 车歪着跑完这 50 厘米才重新开始纠。
		 * 现在改成 return 0：冷却期照样「不认下一个弯」，但方向交给方向环。
		 * 这正是用户说的「开始过钝角弯就歪了」。 */
		return 0;
	}

#if TURN_USE_MILEAGE_TABLE
	if ((Corner_SegMileage < (Turn_ThresholdCm[Corner_Index] - TURN_DIST_EARLY_CM)) ||
	    (Corner_SegMileage > (Turn_ThresholdCm[Corner_Index] + TURN_DIST_LATE_CM)))
	{
		/* 里程对不上 = 这次报弯是假的，不认它。
		 * 2026-11-23 深夜同样改成 return 0：不信这次报弯，不等于「连方向都不打」。 */
		return 0;
	}
#endif

	Corner_Armed      = 1;
	Corner_ArmMileage = Corner_SegMileage;         /* 调试：记下报弯那一刻的里程 */
	/* 2026-11-23 夜修（用户截图发现：报弯里程 165.8、实际 166.7，只差 0.9 厘米）：
	 * Corner_ArmDist 是「报弯之后已经走了多少厘米」，原来只在「视觉连续说不是弯」那条
	 * 分支里清零，报弯这一刻【没有】清零。于是只要上一个弯之后视觉没松口（一直报 2），
	 * 残留的十几厘米还在里面，下一次报弯当场就够 16 厘米 → 立刻开转，
	 * 表现为 I6 − I5 ≈ 0，等于 TURN_ARM_CM 那个「提前量」白设了。
	 * 修法：报弯这一刻必须从零开始数。 */
	Corner_ArmDist    = 0.0f;
	/* 2026-11-23 深夜再改（用户报「弯道前歪头然后转弯」）：
	 * 原来这里是 Corner_SetTarget(CORNER_T_ARM); return 1; —— 目标直接写 15/15 并接管，
	 * 等于接下来 16 厘米里方向环完全不管，车头有一点歪就会一直歪到弯口。
	 * 现在 return 0：把这一拍让给方向环去算（速度在 Task_30ms_Control 里被压到 15 档）。 */
	return 0;
}




/* 这个弯转够了没有？每 5 毫秒问一次。                         （详见 Doc/代码说明.md §17） */
static uint8_t Corner_TurnAngleReached(void)
{
	/* 第二道闸：编码器计数。陀螺仪半路坏了、角度不涨时，靠这个兜底。
	 * 2026-11-23 深夜改（抓包抓出来的）：原来是写死的 2400 计数，而 TURN_DIFF_TARGET = 1730
	 *   才对应 90° —— 2400 等于 **125°**。也就是说陀螺仪一旦不涨，车会转过头 35°，
	 *   实测就是这个：两次转弯都转了约 0.6 秒（2400 ÷ 21 计数每拍 ≈ 0.57 秒），
	 *   陀螺仪读数全程只有 1~2°，车却转出 126°、140°。
	 * 现在按本次目标角度实时算：90° 弯 = 1730×1.05 ≈ 1817，103° 弯 = 1980×1.05 ≈ 2080。
	 *   多留 5% 是因为编码器估算会受打滑影响，宁可多一点点也不要提前停。 */
	{
		float count_backstop = TURN_DIFF_TARGET
		                       * (Turn_TargetAngle_Deg / 90.0f)
		                       * TURN_COUNT_BACKSTOP_RATIO;
		if (Corner_Accum >= count_backstop)
		{
			return 1;
		}
	}

#if TURN_USE_GYRO_ANGLE
	{
		float turned_angle_degrees;//转过的角度

		if (Gyro_IsOk())//如陀螺仪无问题
		{
			turned_angle_degrees = Gyro_GetTotalAngle();//获取当前转角
			if (turned_angle_degrees < 0.0f)//正负无所谓
			{
				turned_angle_degrees = -turned_angle_degrees;
			}

			if (turned_angle_degrees >= Turn_TargetAngle_Deg)//转角达到目标角度
			{
				return 1;//转弯结束
			}
			return 0;//尚未转完
		}
	}
#endif
	/*用老办法保险，或者陀螺仪不在时的退路 */
	if (Corner_Accum >= TURN_DIFF_TARGET)
	{
		return 1;//转弯结束
	}
	return 0;//尚未转完
}


/* 弯道转向状态机：每 5 毫秒问一次「这个弯该不该开始转、转够了没有」。（详见 Doc/代码说明.md §18） */
static void Task_5ms_CornerStep(uint32_t curr_ms)
{
	//弯道转向:每5毫秒检查"这个弯转够了没有",够了就停
	if(Corner_Active)//正在差速转弯
	{
		Corner_Accum += (actual_left - actual_right);   // 老办法用；走陀螺仪时只作记录
		if(Corner_TurnAngleReached() || (curr_ms - Corner_Tick) > TURN_TIMEOUT_MS)//转角足够或超时
		{
			Corner_Active = 0;//转够了(或超时兜底),不再按弯道差速走
			Corner_Done   = 1;//锁存:这个弯已经转过,视觉再喊弯也不转第二次
			Corner_DoneTick = curr_ms;//记下转完的时刻,供"限时交回视觉"用

			/* 这个弯过完了：段内里程清零、编号往后挪一个。   2026-11-21 新增
			 * 这两行是「误差不跨段累积」的关键：下一段的阈值永远从这一刻重新算。 */
			Corner_SegMileage = 0.0f;

			/* 「报弯后已走距离」也一起清零（2026-11-23 夜加）：它和段内里程是一对，
			 * 段内里程清零了它不清，就会把这次的十几厘米带进下一个弯，
			 * 让下一次报弯当场就够 16 厘米、来不及等（见 Task_CornerDecide 里报弯那段）。 */
			Corner_ArmDist = 0.0f;

			/* 终点兜底：先判断「刚转完的这个弯是不是最后一个」，再决定编号加不加。
			 * 2026-11-23 修 off-by-one：顺序一反就出事 —— 自增之后再判，
			 * 第 2 个弯转完时编号已经变成 2（最后一格），会误判成「最后一个也转完了」，
			 * 车就在第 3 个弯前面 99 厘米停住、少转一个弯。
			 * 正确轨迹（3 个弯，TURN_TABLE_SIZE=3）：弯1 编号 0 → 否 → 变 1；
			 * 弯2 编号 1 → 否 → 变 2；弯3 编号 2 → 是 → 置位，编号不动。 */
			if (Corner_Index + 1 >= TURN_TABLE_SIZE)
			{
				Finish_Armed = 1;
			}

			if (Corner_Index + 1 < TURN_TABLE_SIZE)
			{
				Corner_Index++;
			}

			/* 拐完这个弯，把方向环的「上一次偏差」对齐到当前值。
			 * 为什么：拐出弯那一瞬间偏差会从 ~0 突跳到 +26，而方向环的 D 项算的是
			 * 「这一帧偏差 − 上一帧偏差」，这个台阶会被当成"偏差变化极快"，
			 * 额外顶一记方向（0.53 × 8 = 4.2，和 P 项 0.231 × 26 = 6.0 差不多大）。
			 * isFirstRun = 1 就是代码里原有的「把历史对齐到当前值」机制，
			 * 下一帧 D 项自动变成 0，不会凭空踢一下。 */
			pid_l.isFirstRun = 1;

			/* 立刻改成出弯速度（走统一入口 Corner_SetTarget，不再各写各的） */
			Corner_SetTarget(CORNER_T_EXIT);
		}
	}
	else if(Corner_Armed)//准备再往前一段距离开始转弯
	{
		//视觉报弯时,相机还看得远,车其实离弯道还有一段距离,先直着走过去
		Corner_ArmDist += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;//取均值减小单侧误差,最后换算为cm
		if(Corner_ArmDist >= TURN_ARM_CM)//报弯之后再走一段长度之后,开始正式转弯
		{
			Corner_Armed  = 0;//解除准备状态
			Corner_Active = 1;//激活弯道状态:正式转弯
			Corner_Accum  = 0.0f;//累计转角归零开始计数
			Corner_Tick   = curr_ms;//对齐当前时间戳

			/* 调试：把开转这一刻的段内里程锁存下来（见文件上方的说明） */
			Corner_TurnStartMileage = Corner_SegMileage;//标定使用,锁存转弯时间

			/* 从表里取出「这个弯该转多少度」。                    2026-11-21 新增
			 * 直角 90 / 锐角 103 / 两个 38.5 的浅弯默认不甩（不在这张表里）。 */
			Turn_TargetAngle_Deg = Turn_AngleDeg[Corner_Index];//查表要转的角度

			/* 转弯就从这一刻算起，陀螺仪角度清零。
			 * 为什么在这里清：等里程那 20 厘米里车会小幅画龙，
			 * 角度从那时开始累的话会带进弯里；从"真正开始转"起算才干净。 */
			Gyro_ResetTotalAngle();//清理角度开始转弯

			/* 开转这一刻就把目标切成转弯差速，不等下一个 30 毫秒。
			 * 旧代码要等 30ms 任务看到 Corner_Active 才切，那最多 30 毫秒里车还按
			 * TURN_ARM_SPEED 直行（15 档 × 30ms ≈ 1.3 厘米）。这是本次唯一的
			 * 有意行为变化，实车对比时留意。 */
			Corner_SetTarget(CORNER_T_TURN);
		}
	}
}


/* ===== 堵转保护：车被顶住 / 卡住时，别让速度环把占空比一直推到 100% ====（2026-11-23 夜新增）
 *
 * 为什么要它：TB6612 的绝对最大电流只有 3.2A（峰值）/ 1.2A（连续），
 *   而 12.6V 直接加在堵转的电机绕组上，电流是 6~12A —— 差一个数量级以上。
 *   实测抓包（_field_shots/lap_new_01.bin）里有三段这个状态：
 *   跨度 1.4 / 5.5 / 6.0 秒；按判据算的实际累计 1.4 / 5.5 / 5.2 秒
 *   （第三段跨度 6.0 秒，中间有三次约 0.3 秒的短暂恢复，所以累计比跨度少 0.8 秒），
 *   目标一直是 20，两个轮子的实际速度一直是 0，程序没有任何反应。
 *   （这件事和"停车抱死烧管子"是两码事：抱死一次只有 0.1 焦耳量级，可以忽略。）
 *
 * 判据（逐轮判，任意一个轮子满足就算）：下面三条同时成立
 *   1) 这个轮子被要求跑到 15 以上                —— 慢速挪车、转弯内侧轮都不判
 *   2) 它的实际速度 < 1.5 个计数/5ms（= 4.2 厘米/秒）—— 轮子几乎不转
 *   3) 它的速度环输出【正向】顶到 999‰ 以上       —— 「还在往前死推」才叫堵转
 *      为什么必须判正向：转弯时内侧轮的目标只有 3.45，实际被车身拖着跑 4~6，
 *      误差是负的 → 内侧环会饱和到 −1000。若用绝对值判，这条就形同虚设。
 * 再加：连续满足 400 毫秒（中途好一拍就重新计时）。
 * 另外：每次启动后 500 毫秒内不检测。起步时轮子本来就还没转起来，那是正常现象，
 *   这时候速度环同样是满占空比 —— 所以「饱和」这一条挡不住起步，挡起步的是宽限期。
 *
 * 2026-11-23 夜改：原来要求「两轮平均速度 < 1.5 且两个环都饱和」。
 *   那个写法在转弯时有个洞：外侧轮目标 26.55、内侧轮 3.45，平均下来 15 左右，
 *   永远大于 1.5 → 「转弯中有一个轮子被墙顶死」抓不到（外侧还在原速跑）。
 *   改成逐轮判以后这个洞没有了；而转弯内侧轮因为「目标 3.45 < 15」压根不参与判断，
 *   所以也不会误判。
 *
 * 仍然抓不到的一种（写在这里备案）：轮子被要求「几乎不转」（目标 < 15）但仍然被外力
 *   长时间顶着 —— 那种情况下电流本来就小，不构成烧驱动的风险，故意不管。
 *
 * 返回值：1 = 这一拍已经断电停车了，调用者必须马上 return（不然占空比会被后面覆盖）。
 */
static uint8_t Task_5ms_StallGuard(uint32_t curr_ms)
{
	/* 1. 抓「开始跑」的那一刻，用来做启动宽限。CarRun_State 只有 0 / 1 两种值 */
	if (CarRun_State != Stall_RunPrev)
	{
		Stall_RunPrev = CarRun_State;
		if (CarRun_State != 0)              /* 0 → 1：刚按了启动 */
		{
			Stall_RunStart = curr_ms;       /* 宽限期从这里往后数 */
			Stall_Running  = 0;             /* 计时清零 */
		}
	}
	if (CarRun_State == 0) return 0;        /* 已经停着，不用管 */

	/* 2. 启动宽限期：起步这几百毫秒轮子还没转起来是正常的 */
	if ((curr_ms - Stall_RunStart) < STALL_GRACE_MS)
	{
		Stall_Running = 0;
		return 0;
	}

	/* 3. 逐轮判：哪个轮子「被要求快跑 + 却在原地不动 + 速度环已经正向顶死」就算它堵转。
	 * 为什么不用「两轮平均速度」：转弯时外侧轮目标 26.55、内侧轮只有 3.45，
	 *   平均下来还有 15 左右，永远大于阈值 —— 于是「转弯中有一个轮子被墙顶死」抓不到
	 *   （外侧还在原速跑，平均速度一点不低）。逐轮判就没有这个洞。
	 * 为什么不会误判转弯内侧轮：内侧轮目标 3.45 < STALL_TARGET_MIN(15)，
	 *   压根不参与判断。这是有意的：它本来就被要求"几乎不转"。 */
	{
		uint8_t stall_left  = ((target_left  >= STALL_TARGET_MIN) &&   /* 左轮被要求快跑 */
		                       (pid_v_left.output  >= STALL_DUTY_MIN) && /* 左环正向顶死 */
		                       (actual_left  <  STALL_SPEED_MAX) &&      /* 左轮几乎不转 */
		                       (actual_left  > -STALL_SPEED_MAX));
		uint8_t stall_right = ((target_right >= STALL_TARGET_MIN) &&
		                       (pid_v_right.output >= STALL_DUTY_MIN) &&
		                       (actual_right <  STALL_SPEED_MAX) &&
		                       (actual_right > -STALL_SPEED_MAX));
		uint8_t suspect = (stall_left || stall_right) ? 1 : 0;

		/* 4. 连续够久才动手（条件4） */
		if (suspect)
		{
			if (Stall_Running == 0)             /* 刚进「疑似」 */
			{
				Stall_Running = 1;
				Stall_Tick    = curr_ms;
			}
			else if ((curr_ms - Stall_Tick) >= STALL_CONFIRM_MS)
			{
				Stall_Running = 0;
				Stall_CutTimes++;               /* 调试用：一共断过几次 */
				CarRun_Set(0);                  /* 走统一入口进停车态(100ms 任务清积分) */
				Motor_SetDuty(0, 0);            /* 立刻停推。注意这【不是】真断开：
				                                 * 00/00 在 TB6612 上是短路制动。但此刻车不动、
				                                 * 反电动势≈0，所以制动电流也≈0，等于没电流。 */
				return 1;
			}
		}
		else
		{
			Stall_Running = 0;                  /* 中间只要好一拍，就重新计时 */
		}
	}
	return 0;
}


/* 终点兜底：最后一个弯转完之后，按里程停车。每 5 毫秒问一次。   2026-11-23 新增
 * 为什么不能用视觉：K210 那份程序只发 0/1/2，控制端 case 3 永远走不到（详见 Doc/代码说明.md §26）。
 * 为什么用段内里程：只数 94 厘米，CM_PER_COUNT 差 3% 也才 3 厘米；全程数 998 厘米，3% 就是 30 厘米。
 * 返回 1 = 这一拍已经停车了，调用者别再算 PID、别再动电机。 */
static uint8_t Task_5ms_FinishStep(void)
{
#if FINISH_USE_MILEAGE
	if (Finish_Armed == 0) return 0;                    // 还没转完最后一个弯
	if (Corner_SegMileage < FINISH_STOP_CM) return 0;   // 还没走够停车里程
	Finish_Armed = 0;                                   // 只停一次，别反复进
	CarRun_State = 0;                                   // 进停车状态(100ms 任务会做安全清理)
	Motor_SetDuty(0, 0);                                // 立刻断输出,不等那 100 毫秒
	return 1;
#else
	return 0;
#endif
}


/* 台架：编码器里程标定（原来直接写在 5 毫秒任务里，2026-11-22 抽成函数）（详见 Doc/代码说明.md §20） */
static uint8_t Task_EncoderCalib(void)
{
	if(Calib_Mode == 0)//不开启调试模式
	{
		return 0;		// 没进标定模式，交给下面的正常跑车逻辑
	}
    //此后为打开标定模式的命令
	if((actual_left > 0.5f) || (actual_left < -0.5f) ||
	   (actual_right > 0.5f) || (actual_right < -0.5f))//有动作才开始标定,防止上电卡死
	{
		// 有动作：解锁并继续累加
		// (踩过的坑:以前"静止1秒就锁定",而一上电摆车时本来就是静止的,
		//  于是程序上电1秒后就把自己锁死了,之后怎么推都是 0)
		Calib_IdleMs = 0;//已过多少ms计数不改变
		Calib_Frozen = 0;//尚未锁定
		Calib_Left  += actual_left;//计左编码器计数
		Calib_Right += actual_right;//计右编码器计数
		Calib_Cm    += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;//为累计里程,取均值减小误差并转换单位
	}
	else//正式开始标定
	{
		Calib_IdleMs += 5;//最小时间单位5ms
		// 只有"已经走过一段"之后,静止 1 秒才锁定
		if((Calib_IdleMs > 1000) && ((Calib_Left != 0.0f) || (Calib_Right != 0.0f)))
		{
			Calib_Frozen = 1;//冻结标定,锁数值,防止后续因误操作数值变化导致标定不准
		}
	}
	Motor_SetDuty(0,0);	// 标定期间不驱动电机, 可以随便推
	return 1;			// 告诉 5 毫秒任务:这一拍到此为止
}


// 1. 处理启停请求: 物理按键(100ms)
static void Task_StartStopCmd(void)
{
    // 目前：处理物理按键
    if(Key_GetRequest() == 1)
    {
        // 视觉还没起来(K210 上电要 10 秒左右)时,按启动直接忽略 —— 不启动。
        // 为什么不"先记着想启动、等视觉一到就走":那样按下启动后走开,
        // 10 秒后车会突然窜出去,容易撞到手。
        // 正确用法:上电 → 摆车/检查(这 10 秒正好干这些) → 再按启动。
        if(Vision_EverSeen == 0)
        {
            return;
        }
        CarRun_Set(!CarRun_State);
    }

    /* 语音启停（2026-11-21 正式接入）（详见 Doc/代码说明.md §21） */
    uint8_t voice_cmd = Voice_GetCommand();//获取语音指令
    if (voice_cmd == VOICE_CMD_START)//启动指令
    {
        if (Vision_EverSeen != 0)
        //k210没有准备好即使发送指令也作废,若准备好(曝光完成)启动则可被执行
        {
            CarRun_Set(1);
        }
    }
    else if (voice_cmd == VOICE_CMD_STOP)//停止指令
    {
        CarRun_Set(0);
    }
}


//2.停车时的安全清理(100ms)
static void Task_CarStop(void)
{
    Motor_SetDuty(0,0);//使电机停转
    target_left = 0;//清空左速度
    target_right = 0;//清空右速度
    pid_v_left.err_integral = 0;//清空左积分
    pid_v_left.err = 0;//清空左误差
    pid_v_right.err_integral = 0;//清空右积分
    pid_v_right.err = 0;//清空右误差
    pid_l.err = 0;//清空方向环误差
    pid_l.err_diff_last = 0;//清空方向环上次原始偏差
    pid_l.err_diff = 0;//清空本次原始偏差
    Corner_Armed   = 0;//清空弯道准备状态
    Corner_Active  = 0;//清空弯道进行时状态
    Corner_Done    = 0;//清空转弯完毕标志
    Corner_Clear   = 3;//强制清空视觉判弯次数
    Corner_ArmDist = 0.0f;//清空视觉报弯后走过的路程
    Corner_Accum   = 0.0f;//清空转弯累计角度
    Corner_DoneTick = 0;//清空转弯完的时刻
    /* 里程编号与段内里程也要归零：下次启动必须从「第一个要甩的弯」重新数。  2026-11-21 */
    Corner_Index         = 0;//清空弯道序号,下次从直角开始
    Corner_SegMileage    = 0.0f;//清空上个弯道结束到现在走过的路程
    Finish_Armed         = 0;//清空终点兜底标志,下次跑重新等最后一个弯
    Turn_TargetAngle_Deg = TURN_TARGET_ANGLE_DEG;//恢复保守角度,下次被查表值覆盖

    /* 停车时顺手再试一次陀螺仪（2026-11-23 深夜加）。
     * 背景：开机 WHO_AM_I 一次没读通就整趟禁用，实测「偶发 −999」。
     * 这个函数每 100 毫秒被调一次（停车状态），每 10 次（约 1 秒）试一把，
     * 免得一直在那儿读 I2C 占时间。救回来以后这一趟就能用陀螺仪量角度。 */
    {
        static uint8_t gyro_recover_tick = 0;
        if (++gyro_recover_tick >= 10)
        {
            gyro_recover_tick = 0;
            Gyro_TryRecover();
        }
    }
}


//3.处理上位机发给蓝牙的指令:启停+pid调参(100ms)
static void Task_BtCommand(void)
{
    uint8_t rx_byte;
    BtCommand_t bt_cmd;
    
    while (BlueTooth_ReadByte(&rx_byte) == 1)
    {
        if (Protocol_BtParse(rx_byte, &bt_cmd) == 1)
        {
            //第一道安检:拦截无穷大(Inf)和非数字(NaN)的乱码
            if (!(bt_cmd.param_val > -1e6f && bt_cmd.param_val < 1e6f))
            {
                continue; //发现异常直接扔掉这包,不执行下面的switch
            }

            switch(bt_cmd.cmd_type)
            {
                //运行控制: 0.0=停车, 1.0=启动 (与按键共用 CarRun_State)
                case BT_CMD_SET_RUN_STATE: 
                    CarRun_Set((uint8_t)(bt_cmd.param_val > 0.5f)); 
                    break;
                
                //第二道安检:对每类参数设定合理上下限
                //内环速度环
                case BT_CMD_SET_V_L_KP: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_V_KP_MAX) pid_v_left.Kp = bt_cmd.param_val; 
                    break;
                case BT_CMD_SET_V_L_KI: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_V_KI_MAX) pid_v_left.Ki = bt_cmd.param_val; 
                    break;
                case BT_CMD_SET_V_R_KP: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_V_KP_MAX) pid_v_right.Kp = bt_cmd.param_val; 
                    break;
                case BT_CMD_SET_V_R_KI: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_V_KI_MAX) pid_v_right.Ki = bt_cmd.param_val; 
                    break;
                
                //外环方向环
                case BT_CMD_SET_LINE_KP: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_LINE_KP_MAX) pid_l.Kp = bt_cmd.param_val; 
                    break;
                case BT_CMD_SET_LINE_KD: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_LINE_KD_MAX) pid_l.Kd = bt_cmd.param_val; 
                    break;
                
                //基础速度
                case BT_CMD_SET_SPD_STRA:  
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_SPEED_MAX) Speed_Straight = bt_cmd.param_val; 
                    break;
                case BT_CMD_SET_SPD_SMALL: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_SPEED_MAX) Speed_SmallAngle = bt_cmd.param_val; 
                    break;
                case BT_CMD_SET_SPD_LARGE: 
                    if (bt_cmd.param_val >= 0.0f && bt_cmd.param_val <= LIMIT_SPEED_MAX) Speed_LargeAngle = bt_cmd.param_val; 
                    break;
                
                default: break;//非指令集元素不执行
            }
        }
    }
}


//4.发送 VOFA+调试波形(100ms)　2026-11-22 改成按 TELEMETRY_MODE 选内容
static void Task_SendTelemetry(void)
{
    debugdata_t data;

#if (TELEMETRY_MODE == 3)//调试模式3:陀螺仪调试
    /* ===== 3 陀螺仪台架 =====
     * I0 = WHO_AM_I（ICM-20602 应该是 18）
     * I1 = X 原始读数   I2 = Y 原始读数   I3 = Z 原始读数
     * I4 = 累计角度（度）—— 静置时应几乎不动
     * 注意 GYRO_READ_ALL_AXES = 0 时 I1/I2 恒为 0，这是正常的。 */
    data.target_left  = (float)Gyro_GetWhoAmIValue();//实际发送当前陀螺仪的编号
    data.actual_left  = (float)Gyro_GetAxisXRaw();//实际发送X轴原始数据(没用,为0)
    data.target_right = (float)Gyro_GetAxisYRaw();//实际发送Y轴原始数据(没用,为0)
    data.actual_right = (float)Gyro_GetAxisZRaw();//实际发送Z轴原始数据,用于处理转角
    data.error        = Gyro_GetTotalAngle();//实际发送当前转角
    data.mileage_arm  = 0.0f;//实际发送视觉报弯那一刻的段内里程(cm)
    data.mileage_turn = 0.0f;//正式开始转那一刻的段内里程(cm)

#elif (TELEMETRY_MODE == 1)//调试模式1:视觉调试
    /* ===== 1 视觉台架 =====
     * 车可以不动（按住按键上电，电机不驱动），把车摆到赛道上不同位置看这几个数。
     * I0 = 视觉状态（0 直道 / 1 预警 / 2 有弯 / 3 终点）
     * I1 = 视觉偏差（像素）
     * I2 = 实时段内里程（厘米）
     * I3 = 报弯里程（锁存）      I4 = 开转里程（锁存）
     * I5 = 上电以来收到多少帧     I6 = 连续没收到几次（到 11 会触发停车保护） */
    data.target_left  = (float)last_vision_state;//实际发送上一次接收到的视觉状态(0123)
    data.actual_left  = last_vision_error;//实际发送视觉计算的偏差
    data.target_right = Corner_SegMileage;//实际发送此时段内走过的里程
    data.actual_right = Corner_ArmMileage;//实际发送视觉判弯时已走过的段内里程
    data.error        = Corner_TurnStartMileage;//实际发送真正开始转那一刻的段内里程
    data.mileage_arm  = (float)VisionFrameCount;//实际发送已收到的视觉帧数
    data.mileage_turn = (float)VisionLostTicks;//实际发送视觉已经丢线的帧数

#elif (TELEMETRY_MODE == 2)//调试模式2:里程调试
    /* ===== 2 里程台架 =====
     * 正常跑车（视觉循迹），跑一段已知长度，用卷尺量完对比 I3。
     * I0 = 左轮累计计数      I1 = 右轮累计计数
     * I2 = 实时段内里程（厘米）
     * I3 = 两轮平均累计换算成厘米 ← 和卷尺比这个
     * I4 = 报弯里程          I5 = 开转里程          I6 = 0 */
    data.target_left  = Odo_LeftTotal;//实际发送左轮累计计数
    data.actual_left  = Odo_RightTotal;//实际发送右轮累计计数
    data.target_right = Corner_SegMileage;//实际发送此时的段内里程
    data.actual_right = (Odo_LeftTotal + Odo_RightTotal) * 0.5f * CM_PER_COUNT;//实际发送计算的走过的全部里程
    data.error        = Corner_ArmMileage;//实际发送视觉报弯时的里程
    data.mileage_arm  = Corner_TurnStartMileage;//实际发送开始转弯时的里程
    data.mileage_turn = 0.0f;//没数据发:占位

#else
    /* ===== 0 跑车（默认，跑的时候发这一组）=====
     * I0/I2 = 左右轮目标速度   I1/I3 = 左右轮实际速度
     * I4    = 视觉偏差
     * I5    = 报弯里程（锁存）  I6 = 开转里程（锁存）
     *
     * 后面两个就是转弯点标定要看的：跑完一圈，三个弯各给一对数。
     * 报弯里程 = 视觉说"有弯"那一刻的段内里程
     * 开转里程 = 真正开始转那一刻的段内里程，两者之差应 ≈ TURN_ARM_CM */
    data.target_left  = target_left;
    data.actual_left  = actual_left;
    data.target_right = target_right;
    /* I3 从 2026-11-23 夜起改发【陀螺仪累计转角（度）】，不再发右轮实际速度。
     * 为什么：调转弯时最需要知道「陀螺仪读到多少度才停」——
     *   命令 90° 却转了 140°，到底是①陀螺仪读得偏小（读到 90 时车已转 140）、
     *   ②正常停在 90、多出来的角度是方向环拧的、还是③压根没按角度停（超时兜底），
     *   看这个通道一眼就能分开，不用录像也不用猜。
     * 读法：转弯开始时它被清零，从 0 往上涨；转完还要不要它停，全看这个数。
     * 陀螺仪没工作时故意发 −999 —— 屏幕上一看到 −999 就是陀螺仪坏了/没插好，
     *   此时转弯走的是编码器兜底（TURN_DIFF_TARGET = 1730 计数），不可信。
     * 想看右轮实际速度就把下面这一行换回 actual_right。 */
    data.actual_right = Gyro_IsOk() ? Gyro_GetTotalAngle() : -999.0f;
    data.error        = last_vision_error;
    data.mileage_arm  = Corner_ArmMileage;
    data.mileage_turn = Corner_TurnStartMileage;
#endif

    /* 下面这四行只是"用一下"这些变量，把编译告警消掉。
     * TELEMETRY_MODE = 0 时它们不参与显示，但仍然在后台照常更新 —— 想看随时切模式即可。
     * 不影响任何行为，也不占运行时间（编译后什么都不生成）。 */
    (void)VisionFrameCount;
    (void)VisionLostTicks;
    (void)last_vision_state;
    (void)Calib_Frozen;

    Debug_log(&data);
}


// 5. 获取视觉数据与丢包保护(30ms)
/* 视觉断多久算「真断了」，超了就停车。
 * 正常跑：连续 11 次没帧 = 330ms。
 * 转弯中：连续 34 次 = 1020ms —— 转弯是编码器 + 陀螺仪在管，本来就不看视觉，
 *   而弯里 47.5% 的帧好行=0（视觉常哑火）。为了哑火几帧就把车停在弯中间，
 *   既跑不完又危险。转完这个弯立刻收回 330ms。
 * 注意：计数在转弯期间照常累加，所以转完如果视觉还是断的，下一拍就按 330ms 停。 */
#define VISION_LOST_TICKS_RUN    10     /* >10 → 连续 11 次没帧 → 330ms  */
#define VISION_LOST_TICKS_TURN   33     /* >33 → 连续 34 次没帧 → 1020ms */

static uint8_t Task_FetchVisionData(VisionData_t *vis_data)
{
    static uint8_t lostdata_time = 0; 
    
    if(Protocol_GetVision(vis_data) == 0)//没有新数据
    {
        lostdata_time++;
        VisionLostTicks = lostdata_time;   /* 视觉台架显示用：连续没拿到几次 */
        // 2026-11-20 夜里:上电后还没收到过任何一帧时,不要因为超时就把车停掉。
        // 原因:日志里第一行是「已跑 9.9 秒」—— K210 从上电到开始发数据要 10 秒左右。
        // 这 10 秒里按启动键,车会「启动 → 0.33 秒后被超时停掉」,
        // 表现就是「要按好几次才能启动」。
        if(Vision_EverSeen == 0)
        {
            lostdata_time = 0;
            return 0;
        }
        /* 转弯期间用放宽的门槛（1020ms），其余时候 330ms。 */
        {
            uint8_t limit = (Corner_Active != 0) ? VISION_LOST_TICKS_TURN : VISION_LOST_TICKS_RUN;
            if(lostdata_time > limit)
            {
                CarRun_State = 0;//防止摄像头出问题了小车还在跑
                Motor_SetDuty(0,0);
                lostdata_time = 0;
            }
        }
        return 0; // 没拿到
    }
    
    Vision_EverSeen = 1;//收到过数据了,以后超时就真的要停
    
    lostdata_time = 0;//收到数据,清空超时计数
    Vision_LastTick  = App_Clock();//记录视觉存活时刻, 供板载灯判断断线
    Indicator_Fault  = 0;
    return 1; // 拿到了
}


// 6. 计算方向环与控制输出(30ms)
static void Task_30ms_Control(VisionData_t *vis_data)
{
    //弯道状态机
    uint8_t state = vis_data->car_state;//获取视觉报的状态

    float vision_error = (float)vis_data->error;//获取视觉报的像素偏差
    last_vision_error = vision_error;//波形发原始偏差，方便看视觉本身跳不跳

    /* 【视觉固定偏移补偿】2026-11-24 凌晨加。
     *
     * 实测（车不动、摆在起点线正中、连测 90 秒）：偏差稳定读 **−6**。
     *   同一次还把车挪到最左、最右对比：最左 +20、正中 −6、最右 −22
     *   → 符号是对的（左正右负），但零点偏了 6 格。
     *   6 格 × 0.48 厘米 = **2.9 厘米**。
     * 而且这不是偶然：2026-11-23 夜里那次静止验光（Doc §20.3）记的也是「偏差稳在 −6」，
     *   两次独立测量一模一样，说明是**固定的零点偏移**，不是摆偏了。
     *
     * 后果（用户看到的三个现象连成一条链）：
     *   零点偏 −6 → 方向环一直以为车偏在右边 → 一直往左拉
     *   → 车头长期朝左歪 4~6 度（就是「左歪头」）
     *   → 到了弯口，车已经朝左歪着，再右转 78 度就少转那么几度
     *   → 出来压在外侧（左）线上（「转弯延迟、压外侧线」）。
     *
     * 做法：喂给方向环之前把 6 格加回去，让它以为「摆正 = 0」。
     * 为什么放在这里而不是去改 K210：卡现在拆不下来（跑车要插着自己），
     *   而且这个偏移量是「这台车 + 这个摄像头安装角度」的量，放在 STM32 这边调起来快。
     * 以后要是换了摄像头安装角度，只改这一个数。 */
    #define LINE_ERR_OFFSET   3.0f
    vision_error += LINE_ERR_OFFSET;
    last_vision_error = vision_error;   /* 波形也发补偿后的，方便对账 */

    float raw_error = vision_error;//喂给PD的那一份：先限幅
    if (raw_error >  LIMIT_LINE_ERR_MAX) raw_error =  LIMIT_LINE_ERR_MAX;
    if (raw_error < -LIMIT_LINE_ERR_MAX) raw_error = -LIMIT_LINE_ERR_MAX;

#if (TURN_USE_MILEAGE_TABLE && TURN_ALLOW_FORCE)
    /* 【最迟开转点】2026-11-23 夜改成方案 B（用户定的）。
     *
     * 原来它是「视觉一直不报弯时的兜底」，而且要求 Corner_Armed == 0 —— 也就是
     *   一旦视觉报了弯（哪怕报得很晚），它就管不着了，开转点变成 I5 + 16。
     * 实测 I5 会从 150 漂到 170（20 厘米抖动），于是一个固定的 TURN_ARM_CM
     *   根本兜不住：同一个弯，开转点可能是 166，也可能是 186。
     *
     * 现在改成：段内里程一到「阈值 + TURN_FORCE_CM」就无条件转，
     *   **哪怕此时已经报弯、正在等那 16 厘米也照样转**（所以去掉了 Corner_Armed == 0）。
     *   配合 TURN_FORCE_CM = −10.7，开转点被钉死在「阈值 − 10.7」= 理想点。
     *
     * 于是最终规则变成：**开转点 = min(报弯点 + 16, 阈值 − 10.7)**
     *   —— 每段最多晚到理想点，绝不会更晚；而视觉报得特别早（I5 < 阈值 − 26.7）时
     *     仍可能提前两三厘米，那是门禁早侧允许的范围，无害。
     * 好处：一个弯只剩一个旋钮（阈值），调 5 厘米就是早/晚 5 厘米，不再看视觉脸色。
     * 代价：阈值必须准（±5 厘米内），因为它现在就等于开转点。 */
    if ((Corner_Done == 0) && (Corner_Active == 0)
        && (Corner_SegMileage >= CORNER_COOLDOWN_CM)
        && (Corner_SegMileage > (Turn_ThresholdCm[Corner_Index] + TURN_FORCE_CM)))
    {
        Corner_Armed   = 0;                            /* 已经在等 16 厘米的话，取消它 */
        Corner_ArmDist = 0.0f;                         /* 免得残留到下一个弯 */
        Corner_Active  = 1;
        Corner_Accum   = 0.0f;
        Corner_Tick    = App_Clock();
        Corner_TurnStartMileage = Corner_SegMileage;   /* 调试：强转也记开转里程 */
        Turn_TargetAngle_Deg = Turn_AngleDeg[Corner_Index];
        Gyro_ResetTotalAngle();
        Corner_SetTarget(CORNER_T_TURN);   /* 强转第一拍就按转弯差速走 */
    }
#endif

	/* 弯道拍板：状态机接管了目标速度就 return；没接管就往下走 PD。 */
	if (Task_CornerDecide(state)) return;

    //直道pd
    float error = raw_error;//P项用这份：±3 以内算 0，防中心附近细微抖动
    if(fabsf(error)<=3)
    {
        error = 0.0f;
    }

    float turn = PID_L_Calc(&pid_l,error,raw_error);//差速大小(P用error，D用raw_error)
    float base_speed = Speed_Straight;

    switch (state)
    {
    case 1://直角弯或钝角弯
        base_speed = Speed_SmallAngle;
        break;
    case 2:
        base_speed = Speed_LargeAngle;
        break;
    case 3:
        Motor_SetDuty(0, 0);
        CarRun_State = 0;
        return;
    }

    /* 报弯之后「等那 16 厘米」这段时间：速度压到 15 档，但方向仍然交给方向环。
     * 2026-11-23 深夜改（用户报「弯道前歪头然后转弯」）：
     *   原来 Task_CornerDecide 在报弯后直接把目标写成 15/15 并 return 1，
     *   等于这 16 厘米（15 档 42 厘米/秒 → 约 0.38 秒）方向环完全不管 ——
     *   车进入这段时只要车头有一点歪，就会一直歪着走到弯口，看起来就是「弯道前歪头」。
     *   现在改成：速度照样降到 15 档，但方向由方向环负责，弯道前也能贴着线走。 */
    if (Corner_Armed && (base_speed > TURN_ARM_SPEED))
    {
        base_speed = TURN_ARM_SPEED;
    }
    /* 但这段是整条赛道上视觉最不可靠的时候（弯口附近九行里常常只剩 1~3 行），
     * 所以只给「弱修正」：差速最多 ±3，别让方向环跟着跳动的偏差甩来甩去。 */
    if (Corner_Armed)
    {
        if (turn >  TURN_ARM_TURN_MAX) turn =  TURN_ARM_TURN_MAX;
        if (turn < -TURN_ARM_TURN_MAX) turn = -TURN_ARM_TURN_MAX;
    }

    /* 差速上限：恢复到「最多等于基础速度」（pid_l.output_max 已经是 Speed_Straight）。
     * 2026-11-23 白天曾压到 35%，配合 P 项饱和一起治假偏差；假偏差的源头已在 K210 那边堵住，
     * 这里就不再额外压 —— 真偏 12 厘米时需要 40% 左右的差速才拉得回来。 */
    target_left = base_speed + turn;
    target_right = base_speed - turn;
}


static void Task_Indicate(void)
{
    uint32_t now_ms = App_Clock();
    uint8_t  blink_ticks;

    // 台架标定模式:双闪(闪两下、停一下,周期 1.2 秒)。
    // 为什么不用常亮:2026-11-20 夜发现这块主控板的 LED 极性和代码里写的相反
    // (代码里 Indicator_TurnOn 是输出低电平,这块板低电平是灭),所以"常亮"会显示成常暗,
    // 而且常暗又跟"运行中视觉正常"撞在一起。闪烁不受极性影响,所以改用闪烁区分。
    if (Calib_Mode)//校验模式:打双闪
    {
        Indicator_Tick++;
        if (Indicator_Tick >= 12) Indicator_Tick = 0;
        if (Indicator_Tick < 2 || (Indicator_Tick >= 4 && Indicator_Tick < 6))
        {
            Indicator_TurnOn();
        }
        else
        {
            Indicator_TurnOff();
        }
        return;
    }

    //启动瞬间重新计时, 避免上电期间的空窗被判成断线
    if (CarRun_State == 1 && CarRun_PrevState == 0)
    {
        Vision_LastTick = now_ms;
    }
    CarRun_PrevState = CarRun_State;

    // 视觉还没就绪(K210 上电约 10 秒):快闪,告诉用户"现在按启动不算数"
    // 一旦收到第一帧,这个标志变 1,灯自动回到慢闪 = 可以按启动了
    if (Vision_EverSeen == 0)
    {
        Indicator_Fault = 1;
    }

    //只在运行中判超时; 停车后保留故障标志, 方便赛后看出是摄像头出问题
    if (CarRun_State == 1 && (now_ms - Vision_LastTick) > INDICATE_VISION_LOST_MS)
    {
        Indicator_Fault = 1;
    }

    //运行中且视觉正常 -> 常亮
    if (CarRun_State == 1 && Indicator_Fault == 0)
    {
        Indicator_Tick = 0;
        Indicator_TurnOn();
        return;
    }

    //视觉断线快闪, 否则慢闪
    blink_ticks = (Indicator_Fault == 1) ? INDICATE_FAST_TICKS : INDICATE_SLOW_TICKS;
    Indicator_Tick++;
    if (Indicator_Tick >= blink_ticks)
    {
        Indicator_Tick = 0;
        Indicator_Toggle();
    }
}


/* 启停状态的唯一写入口。
 * 按键是"按一次翻转一次", 蓝牙是"明确要求运行或停车",
 * 两种语义不同, 但最终都落到这里。
 * 以后把按键换成蓝牙, 只需要换掉调用者, 状态机本身不用动。 */
static void CarRun_Set(uint8_t run)
{
    CarRun_State = (run != 0) ? 1 : 0;
}
