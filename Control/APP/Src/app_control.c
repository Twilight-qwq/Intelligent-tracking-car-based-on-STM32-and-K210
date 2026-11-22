#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "app_control.h"

static void Task_StartStopCmd(void);
static void Task_CarStop(void);
static void Task_BtCommand(void);
static void Task_SendTelemetry(void);
static uint8_t Task_FetchVisionData(VisionData_t *vis_data);
static void Task_CalculateTargetSpeed(VisionData_t *vis_data);
static void Task_Indicate(void);
static void CarRun_Set(uint8_t run);
static uint8_t Task_EncoderCalib(void);

static uint32_t task_last_5ms = 0;
static uint32_t task_last_10ms = 0;
static uint32_t task_last_30ms = 0;
static uint32_t task_last_100ms = 0;

static pid_velocity_t pid_v_left;
static pid_velocity_t pid_v_right;
static pid_line_t pid_l;

static uint8_t CarRun_State = 0;

static float Speed_Straight   = 30.0f; //直道基速(电机上限37，留7个count给差速)
static float Speed_SmallAngle = 25.0f; //小角度(直角,钝角)基速
static float Speed_LargeAngle = 15.0f; //大角度(锐角)基速
static float target_left = 0.0f;
static float target_right = 0.0f;
static float actual_left = 0.0f;
static float actual_right = 0.0f;
static float last_vision_error = 0.0f; // 用于记录上一次的视觉误差，给 100ms 发波形

/* ===== 弯道转向：用编码器累计差分量角度（2026-11-20 实车改造） =====
 * 为什么不看视觉判断"弯结束"：弯里 66% 的帧好行<=2，视觉基本是瞎的，
 * 由它决定什么时候停止转，角度必然每次都不一样。
 * 原理：转弯时左右轮走的路不一样长，长度差 / 轮距 = 车转过的角度。
 * 换算：1 count/5ms = 0.01405 厘米（由 30档=30counts/5ms=84.3cm/s 推出）
 *       转 90° 需要累计 = 1.5708 * 轮距 / 0.01405 = 111.8 * 轮距
 *       轮距 11 厘米 → 111.8 * 11 = 1230 counts
 */
#define TURN_DIFF_TARGET   1730.0f  // 转 90 度需要累计的(左-右)counts
                                    // 标定记录:理论值 111.8*11=1230,实测只转出 64 度
                                    //          按比例 1230*90/64 = 1730(内轮被拖着走,编码器多算了)

/* ===== 转弯结束条件：用陀螺仪量出来的角度，还是数编码器计数 =====
 * 为什么要换：1730 这个数是"在固定场地上被手推出来的"，
 *   它把打滑、地面、电量全都折进去了。换个地面、电池电压变低、轮胎磨了，
 *   同样的 1730 就会转出不同角度。陀螺仪直接量车身转了多少度，和打滑无关。
 * 怎么退回去：把这个改成 0 就是老办法（数到 1730 就停），不用改别的任何一行。
 * 陀螺仪坏了怎么办：分两种情况，别混。
 *   ① 上电时就没认到（Gyro_IsOk() = 0）→ 转弯退回数编码器 1730，这条成立。
 *   ② 跑着跑着 I2C 接触不良 → Gyro_IsOk() 仍然是 1（它只在初始化时决定一次），
 *      角度只是不再增长，靠 TURN_COUNT_BACKSTOP(2400) 强制结束转弯。
 *      后果是转得多一点（约 125 度），不是转不停。 */
#define TURN_USE_GYRO_ANGLE  1      /* 1 = 转到目标角就停；0 = 老办法数编码器 */
#define TURN_TARGET_ANGLE_DEG 90.0f /* 默认目标角（度）。真正用哪个由下面的表决定 */

/* ===== 里程查表：按「这是第几个要甩的弯」决定转多少度   2026-11-21 新增 =====
 *
 * 为什么要有表：五个弯角度不同（直角 90 / 钝角 38.5 / 锐角 103 / 钝角 38.5 / 直角 90），
 *   全按 90 转必压线。而视觉认不出是哪种，只能认出「有没有弯」。
 *
 * 谁决定「是第几个弯」：里程。每转完一个弯，里程清零、编号 +1，所以误差不跨段累积。
 * 谁决定「转多少度」：这张表。
 * 谁保证「真转到位」：陀螺仪。
 *
 * 三个开关（都能单独回退，不用改别的行）：
 *   TURN_USE_MILEAGE_TABLE = 0 → 完全不查表，回到「视觉报弯就转 90」的老行为
 *   TURN_USE_38_BENDS      = 1 → 两个 38.5° 浅弯也甩（默认 0：当直线，靠方向环自己过）
 *   TURN_ALLOW_FORCE       = 1 → 里程到了视觉还没报，就强行转（默认 0：只认视觉）
 */
#define TURN_USE_MILEAGE_TABLE  1
#define TURN_USE_38_BENDS       0
#define TURN_ALLOW_FORCE        0

/* 死区：里程离本段阈值太远，就当视觉在报假信号，不转。
 * 2026-11-22 从「对称 40」改成「左 30 / 右 10」。为什么两边不能一样：
 *
 *   先记住一件事：容差不是「提前多少厘米转弯」，它只决定「信不信视觉喊弯」。
 *     门什么时候开 = 阈值 − 左容差
 *     车什么时候转 = 门开 + TURN_ARM_CM(20)
 *
 *   ① 左边（早）为什么收到 30：
 *      对称 40 时 180 − 40 = 140 门开，140 + 20 = 160 才转 ——
 *      160 正好是内拐点，也正好是合格线 [160, 180] 的最边上，一点余量都没有。
 *      收到 30：180 − 30 = 150 门开，150 + 20 = 170 才转，几乎就是理想点 171.5。
 *      （这 20 厘米是代码写死的，它会把「早报」抵掉一半，所以左边不能无限收。）
 *
 *   ② 右边（晚）为什么收到 10：
 *      车晚报弯本来就几乎不可能 —— 弯一进画面，视觉会每帧一直喊。
 *      而对称 40 允许 220 也通过，220 + 20 = 240 才开转，那时车已冲出弯口 60 厘米。
 *      只留 10，是给「里程刻度偏大」留的余量（CM_PER_COUNT 还是理论值 0.01402）。
 *
 * 硬约束：总宽（30 + 10 = 40）必须小于最短一段（110 厘米）的一半，否则会串到下一个弯。满足。 */
#define TURN_DIST_EARLY_CM      30.0f   /* 允许比阈值早这么多厘米（门就开在这里） */
#define TURN_DIST_LATE_CM       10.0f   /* 允许比阈值晚这么多厘米（只给刻度误差留一点） */

/* 里程兜底：超过阈值这么多厘米还没等到视觉报弯，就强行转（要 TURN_ALLOW_FORCE = 1 才生效） */
#define TURN_FORCE_CM           30.0f

/* 第二道闸（2026-11-21 新增）：编码器累计超过这个数也停。
 * 为什么需要：陀螺仪如果半路坏掉（不是上电就坏，Gyro_IsOk 还是 1），
 *   角度就永远不涨，车会一直转，只剩 800 毫秒超时兜底 ——
 *   800 毫秒 × 240 度/秒 ≈ 190 度，车直接转成背朝前。
 * 2400 的来历：90 度要 1730 计数，103 度约 1980；取 2400 = 1730 × 1.39，
 *   正常转弯（包括 103 度）都够不到，不会误触发。 */
#define TURN_COUNT_BACKSTOP     2400.0f

/* ===== 弯道速度表（2026-11-21）=====
 * 直道 30 档，一看到弯就降到 15（用视觉报弯到弯口那 20 厘米当刹车距离），
 * 转弯本身也是 15 档，出弯先保持 20 档跑一段再回 30。 */
#define Speed_CornerExit    20.0f   /* 出弯后交接期间的速度，交接完才回 Speed_Straight */

#if TURN_USE_38_BENDS
  #define TURN_TABLE_SIZE 5
  static const float Turn_ThresholdCm[TURN_TABLE_SIZE] = { 200.0f, 110.0f, 240.0f, 240.0f, 110.0f };
  static const float Turn_AngleDeg[TURN_TABLE_SIZE]     = {  90.0f,  38.5f, 103.0f,  38.5f,  90.0f };
#else
  #define TURN_TABLE_SIZE 3
  /* 阈值单位：厘米，从「起点 / 上一个弯转完」算起。
   *
   * 【2026-11-22 全部减去「外拐点偏移」】用户要求压小。
   * 原来这三个数是照 Doc\赛道数据.md 量的，而那份文档写明「按外轮廓量」——
   * 量到的是【外拐点的里程】。车跑的是车道中心线，两者沿行进方向差
   *   20 × tan(角度÷2)      20 = 外线到中心线的横向距离
   * 也就是：外拐点比中心线拐点靠前 20 厘米（90° 弯）/ 25.1 厘米（103° 弯）。
   *
   * 所以改法 = 每个阈值减去「那个弯自己的偏移」：
   *   200 → 180   （90°，  减 20 × tan45°   = 20.0）
   *   350 → 325   （103°， 减 20 × tan51.5° = 25.1 → 取 25）
   *   350 → 330   （90°，  减 20.0 → 330）
   * 现在这三个数的含义是「到中心线拐点（也就是拐角正方形的中心）的距离」。
   *
   * 还是没实测！等跑一圈拿到真实的「开始转那一刻的里程」，用那个数替换。
   * 注意它同时是「信不信视觉报弯」的门禁中心（TURN_DIST_EARLY_CM / TURN_DIST_LATE_CM）。 */
  static const float Turn_ThresholdCm[TURN_TABLE_SIZE] = { 180.0f, 325.0f, 330.0f };
  static const float Turn_AngleDeg[TURN_TABLE_SIZE]     = {  90.0f, 103.0f,  90.0f };
#endif

#define TURN_SPEED         15.0f    // 转弯时基速(= Speed_LargeAngle)
#define TURN_VALUE         11.55f   // 转弯时差速(= Kp 0.231 * 视觉固定值 50)
#define TURN_TIMEOUT_MS    800      // 兜底:0.8秒还没转够就强制结束(卡住了也不至于一直转)
#define TURN_ARM_CM        20.0f    // 视觉报弯后,还要再直走这么多厘米才真的开始转
                                    // 2026-11-21: CM_PER_COUNT 改回理论值 0.01402 后,
                                    //   这里填20 就是真走20厘米(之前填的是0.01610,实际只走17.4厘米)
                                    // 参考:标定前填30时, 实际约走 34 厘米(用户反馈"转弯略晚")
                                    // 太早就加大(24/28), 冲出去就减小
#define CM_PER_COUNT       0.01402f // 1 个编码器计数 = 多少厘米
                                    // 2026-11-21 改回【理论值】, 不用台架手推那个数:
                                    //   霍尔13线 x 4倍频 = 52 计数/电机圈, 减速比 1:28
                                    //   -> 1456 计数/轮子圈
                                    //   轮径 65mm -> 周长 pi*6.5 = 20.42 厘米
                                    //   20.42 / 1456 = 0.01402 厘米/计数
                                    // 为什么不信台架那个 0.01610(112cm/6955.5计数):
                                    //   它比理论高 14.8%, 用户确认那次手推不准;
                                    //   理论值不依赖任何一次手推或卷尺判点。
                                    // 注: 原来的 0.01405 其实就是这个理论值(差0.2%), 绕一圈回来了。
                                    // 换算成速度: 30档 = 84.1 厘米/秒  25档 = 70.1  15档 = 42.1
                                    // 视觉侧 SM_GEAR_SPEED 假设的 30档=84.3 厘米/秒, 现在两边对上了。
#define TURN_ARM_SPEED     15.0f    // 等里程期间的直行速度:提前降到转弯基速,别带着最高速进弯

/* 转完90度之后,最多再直行这么久,就得把方向盘交回视觉。
 * 为什么需要:编码器量的是轮子转了多少,不是车转了多少,中间隔着打滑。
 * 换个地面(更滑/更涩)同样的 1730 会转出不同角度,所以"转完"只保证大致到位。
 * 剩下的角度必须让视觉去修 —— 视觉不擅长从0转90度,但很擅长修这±20度。
 * 如果视觉一直说不"不是弯",以前会一直直行飞出去,这个超时就是防这个的。 */
#define CORNER_HANDOVER_MS 500      // 转完之后最多直行 500 毫秒

/* 弯道冷却（厘米）。
 * 为什么需要:实测推车时,同一个弯里视觉会"报弯 → 松开 → 又报弯"(14.5秒触发,
 * 18.2秒松开,23.3秒又触发)。STM32 转完一个弯之后就解锁了,于是会转第二次。
 * 加一段"转过之后先走够这么远才允许下一个弯"的距离锁。
 * 取 70 厘米的依据:用户实测赛道上最近的两个弯之间也隔着 1 米以上,
 * 所以 70 厘米不会漏掉任何真弯,同时留了 30 厘米余量。
 * 注意:转完那 500 毫秒直行的约 42 厘米也计在里面,所以交回视觉后还要再走约 28 厘米。 */
#define CORNER_COOLDOWN_CM 70.0f

/* 偏差限幅(像素)。
 * 2026-11-20 夜里实车数据:限幅 20 时,日志里出现连续 0.6 秒偏差卡在 +27~+31 不往回修
 * (20.7~21.3 秒那一段)。原因是 31 被削成 20,舵量从 0.231*31=7.2 掉到 4.6,
 * 少了四成纠正力,车就修不回来、越跑越偏,看着像画龙。
 * 放到 30:直道上正常偏差(p99=13)完全不受影响,只有真正的大偏差才削一点。
 * 这个限幅原本是防"进弯前那几帧畸变值把车拧歪"的,30 依然挡得住 50 以上那种。 */
#define LIMIT_LINE_ERR_MAX 30.0f

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

/* ===== 调试：锁存「报弯里程」和「开转里程」      2026-11-22 新增 =====
 * 为什么要锁存，而不是发遥测时现取：
 *   遥测 100 毫秒发一帧，车速 70 厘米/秒时一帧就走了 7 厘米；
 *   而我们关心的转弯窗口本身只有 20 厘米宽 —— 现取会差出三分之一。
 *   所以在事件那一瞬间把值存下来，之后保持不变，采样误差为 0。
 * 跑一圈后在 VOFA+ 上看这两个通道，怎么判断：
 *   ① 开转里程 − 报弯里程 应该 ≈ TURN_ARM_CM(20)。差得多，说明中间被别的条件拦过。
 *   ② 拿「开转里程」和 Turn_ThresholdCm 比：差多少，阈值就该往前挪多少。
 *   ③ 三个弯各看一次。三次的差值应该差不多；差得多说明里程刻度有问题。 */
static float    Corner_ArmMileage       = 0.0f;  // 视觉报弯那一刻的段内里程（厘米）
static float    Corner_TurnStartMileage = 0.0f;  // 真正开始转那一刻的段内里程（厘米）

/* ===== 台架/调试统计量                                    2026-11-22 新增 =====
 * Odo_LeftTotal / Odo_RightTotal：两轮从上车到现在一共转了多少个计数。
 *   和 Corner_SegMileage 同步累加，但**永远不清零** —— 段内里程每过一个弯就清零，
 *   要量"整段距离"必须有一个不清零的。里程台架就用它和卷尺对比，校 CM_PER_COUNT。
 * VisionFrameCount：上电以来收到过多少帧视觉数据。台架上这个数在涨，说明串口通。
 * VisionLostTicks ：连续多少次方向环没拿到新数据（0 ~ 11）。到 11 会触发停车保护。
 *   台架上它一直停在 11，说明视觉断了；一直在 0~2 之间跳，说明数据正常。 */
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

/* ===== 台架：编码器里程标定（2026-11-20 深夜加） =====
 * 目的：实测「1 个编码器计数 = 多少厘米」。
 *       现在代码里的 CM_PER_COUNT = 0.01405 是从"30档=84.3厘米/秒"反推的，
 *       而 84.3 本身是估算，从来没直接量过。这个数一错，等里程和转角累计两个都跟着错。
 * 用法：按住开关不放 → 上电 → 灯常亮 = 进入标定模式。
 *       电机不驱动（可以随便推），用手把车推过一段量好的距离（比如 100 厘米）。
 *       停手 1 秒后数值锁定。
 *       看 VOFA 五个通道（VOFA+ 里从上到下叫 I0~I4）：
 *         I0 = 左轮累计 counts
 *         I1 = 右轮累计 counts
 *         I2 = 里程（按当前 CM_PER_COUNT 换算的厘米）  ← 和卷尺量的比
 *         I3 = 1 表示数值已锁定（松手超过 1 秒）
 *         I4 = 0（保留，没用到）
 *       真实的「每计数多少厘米」= 卷尺厘米 ÷ ( (I0 + I1) ÷ 2 )
 *         例：推 294 厘米，(I0+I1)/2 = 20970 counts
 *             294 ÷ 20970 = 0.01402 厘米/计数
 *             正好等于现在的理论值 → 理论值对，什么都不用改
 * 退出：断电，不要按住开关再上电即可。 */
static uint8_t  Calib_Mode   = 0;      // 1=标定模式
static uint8_t  Calib_Frozen = 0;      // 1=停手1秒后锁定了数值
static float    Calib_Left   = 0.0f;   // 左轮累计 counts
static float    Calib_Right  = 0.0f;   // 右轮累计 counts
static float    Calib_Cm     = 0.0f;   // 累计里程(换算厘米)
static uint32_t Calib_IdleMs = 0;      // 已经多少毫秒没有新计数

/* 这个弯转够了没有？每 5 毫秒问一次。                                    2026-11-21 新增
 * 返回 1 = 够了，该停；0 = 还要继续转。
 *
 * 优先用陀螺仪：它量的是车身真的转了多少度，和轮子打滑无关。
 * 上电就没认到（Gyro_IsOk() = 0）才退回老办法：数编码器计数到 TURN_DIFF_TARGET。
 * 注意运行期 I2C 坏掉不会让 Gyro_IsOk() 变 0，那种情况靠下面的 2400 强制停。
 * 所以这个函数永远不会因为陀螺仪坏掉而转不停。
 *
 * Corner_Accum 是转弯过程中累计的 (左轮计数 − 右轮计数)。
 * Gyro_GetTotalAngle() 是陀螺仪累计的角度（度），右转为负，所以比较前取绝对值。 */
static uint8_t Corner_TurnAngleReached(void)
{
	/* 第二道闸：编码器计数。陀螺仪半路坏了、角度不涨时，靠这个兜底。
	 * 正常转弯（包括 103 度）都到不了 2400，所以平时不会误触发。 */
	if (Corner_Accum >= TURN_COUNT_BACKSTOP)
	{
		return 1;
	}

#if TURN_USE_GYRO_ANGLE
	{
		float turned_angle_degrees;

		if (Gyro_IsOk())
		{
			turned_angle_degrees = Gyro_GetTotalAngle();
			if (turned_angle_degrees < 0.0f)
			{
				turned_angle_degrees = -turned_angle_degrees;
			}

			if (turned_angle_degrees >= Turn_TargetAngle_Deg)
			{
				return 1;
			}

			return 0;
		}
	}
#endif
	/* 老办法，或者陀螺仪不在时的退路 */
	if (Corner_Accum >= TURN_DIFF_TARGET)
	{
		return 1;
	}

	return 0;
}

uint32_t Service_Clock(void)
{
	return SysTimer_GetTick();
}

/* =========================================================================
 * 陀螺仪已拆到 HardwareLayer/Gyro.c + Gyro.h（2026-11-21 用户要求单独成文件）
 *
 * 这里只留「怎么用它」：
 *   Service_Init()     里调一次 Gyro_Init()
 *   AppTask_5ms()  里每个周期调一次 Gyro_Update()
 *   要角度 -> Gyro_GetTotalAngle()；要清零 -> Gyro_ResetTotalAngle()
 *   模块不在时 Gyro_IsOk() 返回 0，直接退回编码器那套，不影响跑车
 * ========================================================================= */
/* ===== 遥测显示什么（VOFA+ 那 7 个通道）               2026-11-22 重做 =====
 *
 * 为什么改成「一个开关四选一」，而不是「按住键进台架就自动换」：
 *   视觉台架要求车不动、但视觉照常工作；而台架模式（Calib_Mode）会把视觉读取整个跳过。
 *   所以视觉台架不能算"台架模式"，它只是"换个显示内容"。
 *   于是统一成一个编译开关，和车动不动无关。
 *
 *   0 = 跑车用（默认）：车轮目标/实际 ×4 + 视觉偏差 + 报弯里程 + 开转里程
 *   1 = 视觉台架     ：状态 / 偏差 / 实时段内里程 / 报弯里程 / 开转里程 / 收到帧数 / 连丢次数
 *                      用法：按住按键上电（电机不驱动），把车摆到赛道上不同位置，看这几个数
 *   2 = 里程台架     ：左轮累计计数 / 右轮累计计数 / 实时段内里程 /
 *                      累计换算厘米 ← 和卷尺比这个 / 报弯里程 / 开转里程
 *                      用法：正常跑车跑一段已知长度，对比里程
 *   3 = 陀螺仪台架   ：WHO_AM_I / X / Y / Z 原始读数 / 累计角度
 *                      （相当于以前 GYRO_BENCH_TEST = 1；已验证通过，现在默认不用）
 *
 * 改完要重新编译烧录才生效。每种的通道含义见 Task_SendTelemetry() 里的注释。 */
#define TELEMETRY_MODE    0


void Service_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Encoder_Init();
	Motor_Init();
	Key_Init();
	// 台架标定模式:上电瞬间按住开关不放就进标定
	// (按键接在 PA0, 上拉输入, 按下 = 低电平; 具体引脚见 Key.c)
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
	{
		Calib_Mode = 1;
	}
	Voice_Init();
	BlueTooth_Init();
	Debug_Init();
	Indicator_Init();
	Gyro_Init();
	k210rcv_Init();
	SysTimer_Init();
	PID_VelocityLoop_Init(&pid_v_left,100.0f,10.0f,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//台架调定 Kp=100 Ki=10
	PID_VelocityLoop_Init(&pid_v_right,100.0f,10.0f,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//台架调定，左右同参数
	PID_LineLoop_Init(&pid_l,0.231f,0.53f,Speed_Straight,-(float)Speed_Straight);//方向环 Kp=0.231 Kd=0.53（2026-11-20 实车调定：0.5 画龙、0.1 偏钝，0.231/0.53 直道可用）
	//Kp 上限 ≈ 1 / (k × 死区时间)
    WatchDog_Init();
}


void AppTask_Idle(void)
{
	WatchDog_FeedDog();
}


/* 台架：编码器里程标定（原来直接写在 5 毫秒任务里，2026-11-22 抽成函数）
 * 形参：无（用文件里的 actual_left / actual_right）
 * 返回：1 = 正在标定，调用者这一拍到此为止；0 = 没进标定模式，照常跑车
 * 干什么：
 *   ① 电机不驱动，人可以随便推；
 *   ② 只要这一拍轮子动过，就把左右计数累加进 Calib_Left / Calib_Right，
 *      同时按 CM_PER_COUNT 换算成厘米累加进 Calib_Cm；
 *   ③ 停手超过 1 秒（而且之前确实推过一段）才锁定 Calib_Frozen = 1。
 * 踩过的坑：以前是「一静止就锁」，而上电摆车本来就是静止的，
 *   1 秒后就把自己锁死，之后怎么推都是 0 —— 所以多了「已经推过一段」这个前提。
 * 怎么用：按住按键上电 → 灯双闪 = 进了标定 → 推过一段量好的距离 → 看 VOFA+。 */
static uint8_t Task_EncoderCalib(void)
{
	if(Calib_Mode == 0)
	{
		return 0;		// 没进标定模式，交给下面的正常跑车逻辑
	}

	if((actual_left > 0.5f) || (actual_left < -0.5f) ||
	   (actual_right > 0.5f) || (actual_right < -0.5f))
	{
		// 有动作：解锁并继续累加
		// (踩过的坑:以前"静止1秒就锁定",而一上电摆车时本来就是静止的,
		//  于是程序上电1秒后就把自己锁死了,之后怎么推都是 0)
		Calib_IdleMs = 0;
		Calib_Frozen = 0;
		Calib_Left  += actual_left;
		Calib_Right += actual_right;
		Calib_Cm    += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;
	}
	else
	{
		Calib_IdleMs += 5;
		// 只有"已经走过一段"之后,静止 1 秒才锁定
		if((Calib_IdleMs > 1000) && ((Calib_Left != 0.0f) || (Calib_Right != 0.0f)))
		{
			Calib_Frozen = 1;
		}
	}
	Motor_SetDuty(0,0);	// 标定期间不驱动电机, 可以随便推
	return 1;			// 告诉 5 毫秒任务:这一拍到此为止
}

void AppTask_5ms(uint32_t curr_ms)//内环速度环控制
{
	if(curr_ms - task_last_5ms < 5) return;
    task_last_5ms = curr_ms; //重置闹钟

	Encoder_Update();
	actual_left  = (float)Encoder_GetDeltaLeft();
	actual_right = (float)Encoder_GetDeltaRight();

	/* 陀螺仪每 5 毫秒读一次。                                     2026-11-21 改
	 * 为什么从"只在台架模式读"挪到这里：
	 *   ① 转弯结束条件要靠它量角度，跑车时也必须更新；
	 *   ② 上电后头 1.25 秒在测零偏，放在这里的话一上电就开始测，
	 *      等用户按启动时零偏早就好了；放在台架分支里就只有台架模式才测。
	 * 传 curr_ms 进去：陀螺仪要用「这次和上次隔了多少毫秒」算角度，
	 *   不能用固定的 5 毫秒 —— 主循环不是精确节拍，被中断或长任务打断时
	 *   某一轮会变长（2026-11-21 波形改 DMA 后，Debug_log 已经不再阻塞主循环）。 */
	Gyro_Update(curr_ms);

	/* 台架标定模式：只数轮子、不驱动电机。返回 1 说明这一拍别跑车。 */
	if(Task_EncoderCalib()) return;

	if(CarRun_State == 0) return;

	/* 段内里程：从「上一个弯转完」到现在走了多少厘米。
	 * 一个数干两件事：
	 *   ① 和阈值表比，决定「该不该转、该转多少度」
	 *   ② 和 CORNER_COOLDOWN_CM(70) 比，当冷却用，防止同一个弯被报两次
	 * 为什么不数整圈：整圈 998.8 厘米，按 ±2% 的刻度误差会累积到 ±20 厘米；
	 * 每过一个弯清零，最长一段 350 厘米，误差只有 ±7 厘米。 */
	Corner_SegMileage += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;

	/* 里程台架用：两轮累计计数，永不清零（段内里程每过一个弯会清零，量全程要用这个） */
	Odo_LeftTotal  += actual_left;
	Odo_RightTotal += actual_right;

	//弯道转向:每5毫秒检查"这个弯转够了没有",够了就停
	if(Corner_Active)
	{
		Corner_Accum += (actual_left - actual_right);   // 老办法用；走陀螺仪时只作记录
		if(Corner_TurnAngleReached() || (curr_ms - Corner_Tick) > TURN_TIMEOUT_MS)
		{
			Corner_Active = 0;//转够了(或超时兜底),不再按弯道差速走
			Corner_Done   = 1;//锁存:这个弯已经转过,视觉再喊弯也不转第二次
			Corner_DoneTick = curr_ms;//记下转完的时刻,供"限时交回视觉"用

			/* 这个弯过完了：段内里程清零、编号往后挪一个。   2026-11-21 新增
			 * 这两行是「误差不跨段累积」的关键：下一段的阈值永远从这一刻重新算。 */
			Corner_SegMileage = 0.0f;
			if (Corner_Index + 1 < TURN_TABLE_SIZE)
			{
				Corner_Index++;
			}

			/* 立刻改成直行目标。
			 * 为什么:目标速度只有 30ms 任务会改,而那要等下一帧视觉(最多 27.7 毫秒)。
			 * 如果视觉正好在这一刻断掉,目标会一直停在转弯差速上,直到 330 毫秒超时停车 ——
			 * 这段时间车会再转将近 90 度(离线仿真里验证过)。所以这里直接把目标拨回直行。
			 * 但注意拨的是「出弯速度 20 档」不是 30 档:接下去还有 500 毫秒是盲开的
			 * (CORNER_HANDOVER_MS),那段时间加速会越跑越偏。            2026-11-21 改 */
			target_left  = Speed_CornerExit;
			target_right = Speed_CornerExit;
		}
	}
	else if(Corner_Armed)
	{
		//视觉报弯时,相机还看得远,车其实离弯道还有一段距离,先直着走过去
		Corner_ArmDist += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;
		if(Corner_ArmDist >= TURN_ARM_CM)
		{
			Corner_Armed  = 0;
			Corner_Active = 1;
			Corner_Accum  = 0.0f;
			Corner_Tick   = curr_ms;

			/* 调试：把开转这一刻的段内里程锁存下来（见文件上方的说明） */
			Corner_TurnStartMileage = Corner_SegMileage;

			/* 从表里取出「这个弯该转多少度」。                    2026-11-21 新增
			 * 直角 90 / 锐角 103 / 两个 38.5 的浅弯默认不甩（不在这张表里）。 */
			Turn_TargetAngle_Deg = Turn_AngleDeg[Corner_Index];

			/* 转弯就从这一刻算起，陀螺仪角度清零。
			 * 为什么在这里清：等里程那 20 厘米里车会小幅画龙，
			 * 角度从那时开始累的话会带进弯里；从"真正开始转"起算才干净。 */
			Gyro_ResetTotalAngle();
		}
	}

	float pwm_left = PID_V_Calc(&pid_v_left,actual_left,target_left);
	float pwm_right = PID_V_Calc(&pid_v_right,actual_right,target_right);
	Motor_SetDuty((int16_t)pwm_left,(int16_t)pwm_right);
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
    Task_CalculateTargetSpeed(&vis_data);
}


void AppTask_10ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_10ms < 10) return;

    task_last_10ms = curr_ms;
    Key_CheckRequest(curr_ms);
}


void AppTask_100ms(uint32_t curr_ms)//开关控制与数据发送
{
	
    if(curr_ms - task_last_100ms < 100) return;
    task_last_100ms = curr_ms;

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

    /* 语音启停（2026-11-21 正式接入）
     * ------------------------------------------------------------------
     * 这里原来是一段注释掉的计划，原文是：
     *     uint8_t voice_cmd = Voice_GetCommand();
     *     if (voice_cmd == 0x01) CarRun_State = 1;
     *     else if (voice_cmd == 0x02) CarRun_State = 0;
     *
     * 现在按原计划接上，判断逻辑一模一样，只多了两道保险：
     *   ① 启动走 CarRun_Set()（本文件底部写着"启停状态的唯一写入口"），
     *      不直接给 CarRun_State 赋值，这样以后加"启动要做的事"只改一个地方；
     *   ② 启动前查 Vision_EverSeen —— 和按键同一条门禁。
     *      视觉没起来就启动的话，车会先愣着，等 K210 那 10 秒过了再突然窜出去。
     *
     * 停车随时有效，不需要门禁（停车是"安全方向"，任何条件下都该生效）。
     *
     * 帧格式在 MID/Protocol.c 里解析：
     *     启动 AA 01 55  ->  Voice_GetCommand() 返回 0x01
     *     停车 AA 02 55  ->  返回 0x02
     *     没有新指令     ->  返回 0 */
    uint8_t voice_cmd = Voice_GetCommand();
    if (voice_cmd == VOICE_CMD_START)
    {
        if (Vision_EverSeen != 0)
        {
            CarRun_Set(1);
        }
    }
    else if (voice_cmd == VOICE_CMD_STOP)
    {
        CarRun_Set(0);
    }
}

//2.停车时的安全清理,防止积分饱和(100ms)
static void Task_CarStop(void)
{
    Motor_SetDuty(0,0);
    target_left = 0;
    target_right = 0;
    pid_v_left.err_integral = 0;
    pid_v_left.err = 0;
    pid_v_right.err_integral = 0;
    pid_v_right.err = 0;
    pid_l.err = 0;
    pid_l.err_diff_last = 0;
    pid_l.err_diff = 0;
    Corner_Armed   = 0;
    Corner_Active  = 0;
    Corner_Done    = 0;
    Corner_Clear   = 3;
    Corner_ArmDist = 0.0f;
    Corner_Accum   = 0.0f;
    Corner_DoneTick = 0;
    /* 里程编号与段内里程也要归零：下次启动必须从「第一个要甩的弯」重新数。  2026-11-21 */
    Corner_Index         = 0;
    Corner_SegMileage    = 0.0f;
    Turn_TargetAngle_Deg = TURN_TARGET_ANGLE_DEG;
}


//蓝牙调参范围限制(参数上限/下限)
#define LIMIT_V_KP_MAX      400.0f //内环速度环Kp上限(误差 2.5 counts 就打满，再大无意义)
#define LIMIT_V_KI_MAX      80.0f  //内环速度环Ki上限(积分时间 6.3 ms，再大必振)
#define LIMIT_LINE_KP_MAX   1.0f  //外环方向环Kp上限
#define LIMIT_LINE_KD_MAX   1.0f //外环方向环Kd上限
#define LIMIT_SPEED_MAX     37.0f //基础速度上限
//3.处理蓝牙指令: 启停 + 调参(100ms)
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

#if (TELEMETRY_MODE == 3)
    /* ===== 3 陀螺仪台架 =====
     * I0 = WHO_AM_I（ICM-20602 应该是 18）
     * I1 = X 原始读数   I2 = Y 原始读数   I3 = Z 原始读数
     * I4 = 累计角度（度）—— 静置时应几乎不动
     * 注意 GYRO_READ_ALL_AXES = 0 时 I1/I2 恒为 0，这是正常的。 */
    data.target_left  = (float)Gyro_GetWhoAmIValue();
    data.actual_left  = (float)Gyro_GetAxisXRaw();
    data.target_right = (float)Gyro_GetAxisYRaw();
    data.actual_right = (float)Gyro_GetAxisZRaw();
    data.error        = Gyro_GetTotalAngle();
    data.mileage_arm  = 0.0f;
    data.mileage_turn = 0.0f;

#elif (TELEMETRY_MODE == 1)
    /* ===== 1 视觉台架 =====
     * 车可以不动（按住按键上电，电机不驱动），把车摆到赛道上不同位置看这几个数。
     * I0 = 视觉状态（0 直道 / 1 预警 / 2 有弯 / 3 终点）
     * I1 = 视觉偏差（像素）
     * I2 = 实时段内里程（厘米）
     * I3 = 报弯里程（锁存）      I4 = 开转里程（锁存）
     * I5 = 上电以来收到多少帧     I6 = 连续没收到几次（到 11 会触发停车保护） */
    data.target_left  = (float)last_vision_state;
    data.actual_left  = last_vision_error;
    data.target_right = Corner_SegMileage;
    data.actual_right = Corner_ArmMileage;
    data.error        = Corner_TurnStartMileage;
    data.mileage_arm  = (float)VisionFrameCount;
    data.mileage_turn = (float)VisionLostTicks;

#elif (TELEMETRY_MODE == 2)
    /* ===== 2 里程台架 =====
     * 正常跑车（视觉循迹），跑一段已知长度，用卷尺量完对比 I3。
     * I0 = 左轮累计计数      I1 = 右轮累计计数
     * I2 = 实时段内里程（厘米）
     * I3 = 两轮平均累计换算成厘米 ← 和卷尺比这个
     * I4 = 报弯里程          I5 = 开转里程          I6 = 0 */
    data.target_left  = Odo_LeftTotal;
    data.actual_left  = Odo_RightTotal;
    data.target_right = Corner_SegMileage;
    data.actual_right = (Odo_LeftTotal + Odo_RightTotal) * 0.5f * CM_PER_COUNT;
    data.error        = Corner_ArmMileage;
    data.mileage_arm  = Corner_TurnStartMileage;
    data.mileage_turn = 0.0f;

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
    data.actual_right = actual_right;
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
        if(lostdata_time > 10)//11次*30ms=330ms。原来>3(120ms),视觉一帧就100ms,余量不够会误停(2026-11-20改)
        {
            CarRun_State = 0;//防止摄像头出问题了小车还在跑
            Motor_SetDuty(0,0);
            lostdata_time = 0;
        }
        return 0; // 没拿到
    }
    
    Vision_EverSeen = 1;//收到过数据了,以后超时就真的要停
    
    lostdata_time = 0;//收到数据,清空超时计数
    Vision_LastTick  = Service_Clock();//记录视觉存活时刻, 供板载灯判断断线
    Indicator_Fault  = 0;
    return 1; // 拿到了
}

// 6. 计算方向环与控制输出(30ms)
static void Task_CalculateTargetSpeed(VisionData_t *vis_data)
{
    uint8_t state = vis_data->car_state;

    float vision_error = (float)vis_data->error;
    last_vision_error = vision_error;//波形发原始偏差，方便看视觉本身跳不跳

    float raw_error = vision_error;//喂给PD的那一份：先限幅
    if (raw_error >  LIMIT_LINE_ERR_MAX) raw_error =  LIMIT_LINE_ERR_MAX;
    if (raw_error < -LIMIT_LINE_ERR_MAX) raw_error = -LIMIT_LINE_ERR_MAX;

#if (TURN_USE_MILEAGE_TABLE && TURN_ALLOW_FORCE)
    /* 里程兜底：已经过了阈值 TURN_FORCE_CM 这么远，视觉还没报弯 → 强行转。 2026-11-21
     * 默认关（TURN_ALLOW_FORCE = 0）：纯靠里程触发的最坏误差有 37 厘米（没实测过），
     * 要先跑一整圈数出「该转没转」几次，再决定开不开。 */
    if ((Corner_Done == 0) && (Corner_Active == 0) && (Corner_Armed == 0)
        && (Corner_SegMileage >= CORNER_COOLDOWN_CM)
        && (Corner_SegMileage > (Turn_ThresholdCm[Corner_Index] + TURN_FORCE_CM)))
    {
        Corner_Active = 1;
        Corner_Accum  = 0.0f;
        Corner_Tick   = Service_Clock();
        Corner_TurnStartMileage = Corner_SegMileage;   /* 调试：强转也记开转里程 */
        Turn_TargetAngle_Deg = Turn_AngleDeg[Corner_Index];
        Gyro_ResetTotalAngle();
    }
#endif

    /* ---------- 弯道转向状态机（三段） ----------
     * ① 视觉报弯：它看得比车走得远，这时候车离弯道还有一段，先直行过去
     * ② 走够 TURN_ARM_CM：开始按固定差速转，同时把陀螺仪角度清零
     * ③ 陀螺仪量到转够 TURN_TARGET_ANGLE_DEG 度：立刻停并锁存
     *    （陀螺仪不在时退回老办法：累计(左-右)达到 TURN_DIFF_TARGET 就停）
     * 视觉只负责"这里有弯"；什么时候开始转由里程定，转多少度由陀螺仪定。 */
    if (state == 2)
    {
        Corner_Clear = 0;//视觉说这里就是弯,清掉"已经离开"的计数
        if (Corner_Done)//这个弯已经转够了
        {
            Corner_Armed  = 0;
            Corner_Active = 0;
            if ((Service_Clock() - Corner_DoneTick) > CORNER_HANDOVER_MS)
            {
                // 直行够久了,强制交回视觉:不 return,落到下面的PD去。
                // 编码器只管"大致转到90度",剩下的航向让视觉修(偏差已限幅±20,
                // 最多算出差速 0.231*20=4.6,不会把车拧歪)。
            }
            else
            {
                // 刚转完这一小段先直行,让相机从"还在看弯道"缓过来
                target_left  = Speed_CornerExit;
                target_right = Speed_CornerExit;
                return;
            }
        }
        else if (Corner_Active)//里程到了:固定差速转,不看视觉偏差
        {
            target_left  = TURN_SPEED + TURN_VALUE;
            target_right = TURN_SPEED - TURN_VALUE;
            return;
        }
        else //刚报弯:先看是不是还在上一个弯的冷却期里
        {
            if (Corner_SegMileage < CORNER_COOLDOWN_CM)
            {
                // 冷却中:上一个弯刚转完,车还没走够距离。
                // 这时候视觉再喊弯,多半是同一个弯还没从画面里退出去 —— 当直道走,不转。
                target_left  = Speed_Straight;
                target_right = Speed_Straight;
                return;
            }

#if TURN_USE_MILEAGE_TABLE
            /* 死区检查：视觉说是弯，但里程离本段阈值太远 → 当假信号。   2026-11-21 新增
             * 为什么要这条：视觉认得「有弯」但认不出「是哪个弯」，见弯就喊；
             *   而里程知道现在该不该有弯。两边对不上就别信视觉。
             * 2026-11-22：左边 30、右边 10，不再对称（推导见上面 TURN_DIST_EARLY_CM 的注释）。
             * 窗口上限的来历：最短一段 110 厘米的一半，超过它「最近的阈值」就已经是下一个弯了。
             *   现在总宽 30 + 10 = 40，满足。 */
            if ((Corner_SegMileage < (Turn_ThresholdCm[Corner_Index] - TURN_DIST_EARLY_CM)) ||
                (Corner_SegMileage > (Turn_ThresholdCm[Corner_Index] + TURN_DIST_LATE_CM)))
            {
                target_left  = Speed_Straight;
                target_right = Speed_Straight;
                return;
            }
#endif

            Corner_Armed = 1;
            /* 调试：记下视觉报弯这一刻的段内里程（这一帧已经被门禁放行） */
            Corner_ArmMileage = Corner_SegMileage;
            target_left  = TURN_ARM_SPEED;
            target_right = TURN_ARM_SPEED;
            return;
        }
    }
    else
    {
        //要连续 3 次方向环(3 × 30 毫秒 = 90毫秒)都说"不是弯",才算这个弯真的过去了。
        //注意:这里数的是「方向环调了几次」,不是「视觉出了几帧」。
        //  视觉是 36 帧/秒(27.7 毫秒一帧),方向环 30 毫秒看一次,
        //  所以计数器每 30 毫秒才可能 +1。(旧注释写"3帧=300毫秒"是错的,那时视觉还是 10Hz)
        //只掉一帧就清空的话:里程被清零要重走、锁存被清掉会再转一次,就是"反复画圈"的来源。
        if (Corner_Clear < 3) Corner_Clear++;
        if (Corner_Clear >= 3)
        {
            /* 2026-11-21 修：正在转弯时不许被「视觉说不是弯」取消。
             * 为什么：弯里 47.5% 的帧一条好行都没有，视觉在转弯过程中本来就经常哑火。
             * 原来这里无条件清掉 Corner_Active，转弯会在半路停住 ——
             * 而一个 90 度弯要转 374 毫秒，3 次方向环只有 90 毫秒。
             * 真正的结束条件只有两个：「转到目标角」或「800 毫秒超时」。 */
            if (Corner_Active == 0)
            {
                Corner_Armed   = 0;
                Corner_Done    = 0;
                Corner_ArmDist = 0.0f;
                Corner_Accum   = 0.0f;
            }
        }
        //不 return,继续走下面的正常 PD 控制
    }

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

    if (turn > base_speed)  turn = base_speed;
    if (turn < -base_speed) turn = -base_speed;
    target_left = base_speed + turn;
    target_right = base_speed - turn;
}

//7.板载 LED 状态指示(100ms)
#define INDICATE_SLOW_TICKS     5       //慢闪: 5 x 100ms = 500ms 翻转一次
#define INDICATE_FAST_TICKS     1       //快闪: 1 x 100ms = 100ms 翻转一次
#define INDICATE_VISION_LOST_MS 330     //超过该时间没收到视觉数据即判定断线(与丢包判定一致, 11次*30ms)

static void Task_Indicate(void)
{
    uint32_t now_ms = Service_Clock();
    uint8_t  blink_ticks;

    // 台架标定模式:双闪(闪两下、停一下,周期 1.2 秒)。
    // 为什么不用常亮:2026-11-20 夜发现这块主控板的 LED 极性和代码里写的相反
    // (代码里 Indicator_TurnOn 是输出低电平,这块板低电平是灭),所以"常亮"会显示成常暗,
    // 而且常暗又跟"运行中视觉正常"撞在一起。闪烁不受极性影响,所以改用闪烁区分。
    if (Calib_Mode)
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
