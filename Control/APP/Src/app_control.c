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

static uint32_t task_last_5ms = 0;//5ms时间片
static uint32_t task_last_10ms = 0;//10ms时间片
static uint32_t task_last_30ms = 0;//30ms时间片
static uint32_t task_last_100ms = 0;//100ms时间片

static pid_velocity_t pid_v_left;//左速度环pid结构体
static pid_velocity_t pid_v_right;//右速度环pid结构体
static pid_line_t pid_l;//方向环结构体

static uint8_t CarRun_State = 0;//启停状态变量

static float Speed_Straight   = 30.0f; //直道基速(电机上限37，留7个count给差速)
static float Speed_SmallAngle = 25.0f; //小角度(直角,钝角)基速
static float Speed_LargeAngle = 15.0f; //大角度(锐角)基速
static float target_left = 0.0f;//目标左轮速度
static float target_right = 0.0f;//目标右轮速度
static float actual_left = 0.0f;//实际左轮速度
static float actual_right = 0.0f;//实际右轮速度
static float last_vision_error = 0.0f; // 用于记录上一次的视觉误差，给 100ms 发波形

/* 弯道转向：用编码器脉冲累计差分量角度（2026-11-20 实车改造）（详见 Doc/代码说明.md §1） */
#define TURN_DIFF_TARGET   1730.0f  // 转 90 度需要累计的(左-右)counts
                                    // 标定记录:理论值 111.8*11=1230,实测只转出 64 度
                                    //          按比例 1230*90/64 = 1730(内轮被拖着走,编码器多算了)

/* 转弯结束条件：用陀螺仪量出来的角度，还是数编码器计数（详见 Doc/代码说明.md §2） */
#define TURN_USE_GYRO_ANGLE  1      /* 1 = 转到目标角就停；0 = 老办法数编码器 */
#define TURN_TARGET_ANGLE_DEG 90.0f /* 默认目标角（度）。真正用哪个由下面的表决定 */

/* 里程查表：按「这是第几个要甩的弯」决定转多少度   2026-11-21 新增（详见 Doc/代码说明.md §3） */
#define TURN_USE_MILEAGE_TABLE  1//里程校验开关
#define TURN_USE_38_BENDS       0//决定钝角弯要不要也要当弯道处理
#define TURN_ALLOW_FORCE        0//决定是否在视觉迟迟不报弯的时候根据里程强行报弯

/* 死区：里程离本段阈值太远，就当视觉在报假信号，不转。（详见 Doc/代码说明.md §4） */
#define TURN_DIST_EARLY_CM      30.0f   /* 允许比阈值早这么多厘米（门就开在这里） */
#define TURN_DIST_LATE_CM       10.0f   /* 允许比阈值晚这么多厘米（只给刻度误差留一点） */

/* 里程兜底：超过阈值这么多厘米还没等到视觉报弯，就强行转（要 TURN_ALLOW_FORCE = 1 才生效） */
#define TURN_FORCE_CM           30.0f

/* 第二道闸（2026-11-21 新增）：编码器累计超过这个数也停。（详见 Doc/代码说明.md §5） */
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
  /* 阈值单位：厘米，从「起点 / 上一个弯转完」算起。（详见 Doc/代码说明.md §6） */
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
                                    /* 2026-11-21 改回【理论值】, 不用台架手推那个数:（详见 Doc/代码说明.md §7） */
#define TURN_ARM_SPEED     15.0f    // 等里程期间的直行速度:提前降到转弯基速,别带着最高速进弯

/* 转完90度之后,最多再直行这么久,就得把方向盘交回视觉。（详见 Doc/代码说明.md §8） */
#define CORNER_HANDOVER_MS 500      // 转完之后最多直行 500 毫秒

/* 弯道冷却（厘米）。（详见 Doc/代码说明.md §9） */
#define CORNER_COOLDOWN_CM 70.0f//过弯之后再走70cm才允许再次过弯,否则不算

/* 终点兜底（2026-11-23 新增）：最后一个弯转完之后再走这么多厘米就停车。
 * 为什么需要：视觉只发 0/1/2，「case 3 终点急停」永远触发不了（详见 Doc/代码说明.md §26）。
 * 数字怎么来的：最后一段中心线 98.8 厘米，转完弯那一刻已经过去约 5 厘米 → 到起点线约 93.8；
 *   再留 5 厘米保证确实过了线 → 99。实测完按「过线位置」微调这一个数就行。 */
#define FINISH_USE_MILEAGE 1        // 里程兜底停车开关：1=开。/ 0=关（关掉就只剩视觉那条死路）
#define FINISH_STOP_CM     99.0f    // 最后一个弯转完后的停车里程（厘米）

/* 偏差限幅(像素)。（详见 Doc/代码说明.md §10） */
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
	PID_LineLoop_Init(&pid_l,0.231f,0.53f,Speed_Straight,-(float)Speed_Straight);//方向环 Kp=0.231 Kd=0.53（2026-11-20 实车调定：0.5 画龙、0.1 偏钝，0.231/0.53 直道可用）
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
			return 0;                   /* 交接期结束，交回视觉 */
		}
		Corner_SetTarget(CORNER_T_EXIT);
		return 1;
	}

	if (Corner_Active)                  /* 正在转：固定差速，不看视觉 */
	{
		Corner_SetTarget(CORNER_T_TURN);
		return 1;
	}

	if (Corner_SegMileage < CORNER_COOLDOWN_CM)     /* 上一个弯的冷却期 */
	{
		Corner_SetTarget(CORNER_T_STRAIGHT);
		return 1;
	}

#if TURN_USE_MILEAGE_TABLE
	if ((Corner_SegMileage < (Turn_ThresholdCm[Corner_Index] - TURN_DIST_EARLY_CM)) ||
	    (Corner_SegMileage > (Turn_ThresholdCm[Corner_Index] + TURN_DIST_LATE_CM)))
	{
		Corner_SetTarget(CORNER_T_STRAIGHT);        /* 里程对不上 = 假信号 */
		return 1;
	}
#endif

	Corner_Armed      = 1;
	Corner_ArmMileage = Corner_SegMileage;         /* 调试：记下报弯那一刻的里程 */
	Corner_SetTarget(CORNER_T_ARM);
	return 1;
}




/* 这个弯转够了没有？每 5 毫秒问一次。                         （详见 Doc/代码说明.md §17） */
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
        Corner_Tick   = App_Clock();
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

    if (turn > base_speed)  turn = base_speed;
    if (turn < -base_speed) turn = -base_speed;
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
