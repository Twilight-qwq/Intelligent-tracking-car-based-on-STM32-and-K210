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

/* ==================== 可调参数（1）转弯开关 + 五个弯的阈值与角度 ====================
 * 注意：TURN_USE_38_BENDS 必须在本段内部、表的前面 —— 它决定用 5 弯表还是 3 弯表。
 */
#define TURN_DIFF_TARGET   1487.0f  // 转 90 度需要累计的(左-右)counts（16.5 计数/度）

/* 转弯结束条件：用陀螺仪量出来的角度，还是数编码器计数（详见 Doc/代码说明.md §2） */
#define TURN_USE_GYRO_ANGLE  1      /* 1 = 转到目标角就停；0 = 老办法数编码器 */
#define TURN_TARGET_ANGLE_DEG 90.0f /* 默认目标角（度）。真正用哪个由下面的表决定 */

#define TURN_USE_MILEAGE_TABLE  1//里程校验开关
#define TURN_USE_38_BENDS       1//决定钝角弯要不要也要当弯道处理（2026-11-23 改 1：钝角弯靠 PD 太勉强）
/* 决定是否用「里程」来钉住开转点。 */
#define TURN_ALLOW_FORCE        1

/* 死区：里程离本段阈值太远，就当视觉在报假信号，不转。（详见 Doc/代码说明.md §4） */
#define TURN_DIST_EARLY_CM      25.0f   /* 允许比阈值早这么多厘米（门就开在这里） */
#define TURN_DIST_LATE_CM       10.0f   /* 允许比阈值晚这么多厘米（只给刻度误差留一点） */

/* 【最迟开转点】段内里程超过「阈值 + 这个数」就无条件转（要 TURN_ALLOW_FORCE = 1 才生效） */
#define TURN_FORCE_CM           (-10.7f)


#define TURN_COUNT_BACKSTOP_RATIO  3.0f

#define Speed_CornerExit    20.0f   /* 出弯后交接期间的速度，交接完才回 Speed_Straight */

#if TURN_USE_38_BENDS
  #define TURN_TABLE_SIZE 5
  static const float Turn_ThresholdCm[TURN_TABLE_SIZE] = { 188.0f, 79.0f, 184.0f, 191.0f, 79.0f };
  static const float Turn_AngleDeg[TURN_TABLE_SIZE]     = {  90.0f,  38.5f, 103.0f,  38.5f,  90.0f };
  /* 几何真实角度。旧的眼睛标定值是 { 76.0f, 20.5f, 90.0f, 28.5f, 75.0f }（= 几何角 − 过冲），改成闭环后不再需要补偿，要退回就换回那一行 */
#else
  #define TURN_TABLE_SIZE 3
  /* 阈值单位：厘米，从「起点 / 上一个弯转完」算起。（详见 Doc/代码说明.md §6） */
  static const float Turn_ThresholdCm[TURN_TABLE_SIZE] = { 180.0f, 299.0f, 330.0f };
  static const float Turn_AngleDeg[TURN_TABLE_SIZE]     = {  90.0f, 103.0f,  90.0f };
#endif

/* ==================== 可调参数（2）速度 / 转弯动作 / 路段偏置 ==================== */
#define TURN_SPEED         15.0f    // 转弯时基速(= Speed_LargeAngle)
#define TURN_VALUE         11.55f   /* 转弯差速（= Kp 0.231 × 视觉固定值 50）
                                     */
#define TURN_TIMEOUT_MS   600      // 兜底:0.6秒还没转够就强制结束(陀螺仪冻住时才会到)

/* 转弯收尾制动（2026-11-26 加）：角度到目标后改反向差速，把收手那一下甩出去的度吃回来 */
#define TURN_BRAKE_VALUE      8.0f    /* 制动反向差速，单位同 TURN_VALUE：每 1 个计数 = 左右各 ±1 = 差 2 个计数 ≈ 24.7 度/秒，所以 8.0 → 约 198 度/秒 */
#define TURN_BRAKE_RATE_DPS   20.0f   /* 角速度降到这以下才算真的停住，单位 度/秒 */
#define TURN_BRAKE_TIMEOUT_MS 120    /* 制动最长这么久。电机机械时间常数约 39 毫秒，60 只有 1.5 倍，来不及把差速反过来 */
#define GYRO_DPS_PER_COUNT    (1.0f / 65.5f)  /* 65.5 计数 = 1 度/秒，见 bsp_gyro.c 第 44 行 */

/* ===== 转向角度 PD 闭环（2026-11-27 加；改开关可整体退回老办法）=====
 * 老办法：固定差速 11.55 一直转，角度到目标就松手 —— 松手后还会白转十几度。
 * 新办法：每 5 毫秒按「还差多少度」和「转多快」重算差速，接近目标自动减速。 */
#define TURN_USE_ANGLE_PD    1        /* 1=角度 PD 闭环；0=老的固定差速 + 收尾刹车 */
#define TURN_ANGLE_KP        0.85f    /* 还差 1 度给多少差速。按约 65 毫秒收敛时间反推 */
#define TURN_ANGLE_KD        0.015f   /* 每 1 度/秒的角速度压掉多少差速（防冲过头） */
#define TURN_ANGLE_DIFF_MAX  11.55f   /* 差速上限，11.55 约 285 度/秒，跟老办法同速 */
#define TURN_ANGLE_SLEW      3.0f     /* 每 5 毫秒差速最多变这么多（防单拍跳变打满反向） */
#define TURN_BRAKE_RATE_ALPHA 0.4f    /* 制动判据的低通系数，0.4 = 一拍吃掉 40% 差值 */
#define TURN_BRAKE_QUIET_TICKS 3      /* 连续几拍都安静才算真停住（3 拍 = 15 毫秒） */
#define TURN_STUCK_RATIO     1.15f    /* 哑火兜底：编码器转到这里 × 目标角/90 就强制结束 */

#define TURN_BIAS_WINDOW_CM  80.0f
static const float Turn_LineBias[TURN_TABLE_SIZE] = { 0.0f, -26.0f, 0.0f, -26.0f, 0.0f };   /* 见下面的表 */

static uint8_t  Corner_Active = 0;    // 1=正在按固定差速转弯
static uint8_t  Corner_Done   = 0;    // 1=这个弯已经转够90度,别再转
static uint8_t  Corner_Clear  = 0;    // 连续多少帧视觉说"不是弯"
static float    Corner_ArmDist = 0.0f; // 报弯后已经走的距离(换算厘米)
static float    Corner_Accum  = 0.0f;  // 累计(左轮-右轮)counts, 就是转过的角度
static uint32_t Corner_Tick   = 0;     // 本次转弯开始的时刻, 供兜底超时用
static uint8_t  Corner_Braking   = 0;  // 0=没制动 1=正在反向差速制动 2=已制动过(不许再进)
static uint32_t Corner_BrakeTick = 0;  // 制动开始的那一拍
	static float    Corner_DiffPrev       = 0.0f;   // 上一拍的差速，用来做斜率限幅
	static float    Corner_BrakeRateF     = 0.0f;   // 制动用的滤波后角速度
	static uint8_t  Corner_BrakeQuiet     = 0;      // 连续几拍低于阈值
	static uint16_t Corner_BrakeErrLast   = 0;      // 上一拍的读失败次数，用来识别陈旧读数
	static float Corner_TurnPeakAngle = 0.0f;   // 本弯转弯期间陀螺仪认到的最大角度
	static float Corner_LastPeakAngle = 0.0f;   // 锁存:上一个弯的峰值角度(直道漂移不计)
	static float Corner_LastEndAngle  = 0.0f;   // 锁存:上一个弯收手那一刻的角度
static uint32_t Corner_DoneTick = 0;   // 转完的时刻, 供"限时交回视觉"用
static uint8_t  Corner_Index      = 0;      // 正要去第几个要甩的弯(查表下标)
static uint8_t  Corner_TurnedForIndex = 0;
static float    Corner_SegMileage = 0.0f;   // 从上一个弯转完到现在走了多少厘米
static uint8_t  Finish_Armed      = 0;      // 1=最后一个弯已经转完,正在数停车里程（终点兜底）

static float Speed_Straight   = 20.0f; //直道基速(电机上限37，留7个count给差速)  ← 用户手改(原30)，方便测试不用每次发蓝牙
static float Speed_SmallAngle = 20.0f; //小角度(直角,钝角)基速  ← 用户手改(原25)
static float Speed_LargeAngle = 15.0f; //大角度(锐角)基速
static float target_left = 0.0f;//目标左轮速度
static float target_right = 0.0f;//目标右轮速度
static float actual_left = 0.0f;//实际左轮速度
static float actual_right = 0.0f;//实际右轮速度
static float last_vision_error = 0.0f; // 用于记录上一次的视觉误差，给 100ms 发波形


/* ===== 弯道状态机分两半（读这个文件之前先看这段）========================= */

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
/* 方向环的积分项（单位：计数/5ms，最多 ±1.5）。详见 Task_30ms_Control 里的说明。 */
static float Line_Integral = 0.0f;

static uint8_t CarRun_State = 0;//启停状态变量





static uint32_t Stall_Tick     = 0;    /* 这一轮「疑似堵转」是从什么时候开始的 */
static uint8_t  Stall_Running  = 0;    /* 1=上一拍已经在「疑似堵转」里，正在计时 */
static uint32_t Stall_RunStart = 0;    /* 这一趟是什么时候开始跑的，宽限期用 */
static uint8_t  Stall_RunPrev  = 0;    /* 上一拍的启停状态，用来抓 0→1 那个边沿 */
static uint8_t  Stall_CutTimes = 0;    /* 断过几次（调试器里能看，台架上用得到） */

static uint8_t  Corner_Armed  = 0;    // 1=视觉已经报弯,正在等里程


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



/* ==================== 可调参数（3）报弯提前量 / 交接 / 冷却 / 终点 / 限幅 / 堵转 ==================== */
#define TURN_ARM_CM        999.0f   /* 【2026-11-24 早上，再次关掉；原值 16】
                                      */
#define CM_PER_COUNT       0.01402f // 1 个编码器计数 = 多少厘米
#define TURN_ARM_SPEED     15.0f    // 等里程期间的直行速度:提前降到转弯基速,别带着最高速进弯
/* 等里程这 16 厘米里，方向环允许的最大差速（单位同目标速度，计数/5ms）。 */
#define TURN_ARM_TURN_MAX  1.2f

#define TURN_STRAIGHT_DIFF_MAX 1.5f

/* 转完90度之后,最多再直行这么久,就得把方向盘交回视觉。（详见 Doc/代码说明.md §8） */
#define CORNER_HANDOVER_MS 500      // 转完之后最多直行 500 毫秒

/* 弯道冷却（厘米）。（详见 Doc/代码说明.md §9） */
#define CORNER_COOLDOWN_CM 50.0f//过弯之后再走50cm才允许再次过弯,否则不算

#define FINISH_USE_MILEAGE 1        // 里程兜底停车开关：1=开。/ 0=关（关掉就只剩视觉那条死路）
#define FINISH_STOP_CM     99.0f    // 最后一个弯转完后的停车里程（厘米）

/* 偏差限幅(像素)。（详见 Doc/代码说明.md §10） */
#define LIMIT_LINE_ERR_MAX 30.0f

#define STALL_TARGET_MIN    15.0f   /* 有轮子被要求跑到这么多以上才管（单位：计数/5ms） */
#define STALL_SPEED_MAX     1.5f    /* 两轮平均低于这么多算「不走」（1.5×0.01402÷0.005 = 4.2 厘米/秒） */
#define STALL_DUTY_MIN      999.0f  /* 速度环输出（‰）正向顶到这么多算「还在死推」 */
#define STALL_CONFIRM_MS    400     /* 连续这么多毫秒都符合才算真堵转 */
#define STALL_GRACE_MS      500     /* 每次启动后这么久之内不检测（挡起步误杀） */
/* ---- 方向环（外环）---- */
    #define LINE_ERR_OFFSET   (-2.5f)

    //直道pd
    #define LINE_ERR_DEADBAND  1.0f

    #define LINE_KI      0.0f
    #define LINE_I_MAX   1.5f
    #define LINE_I_FREEZE_ERR  20.0f

void App_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Encoder_Init();
	Motor_Init();
	Key_Init();
	// 台架标定模式:上电瞬间按住开关不放就进标定
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

	/* 堵转保护：车被顶住/卡住时断电停车，别把驱动烧掉。（详见 Task_5ms_StallGuard） */
	if(Task_5ms_StallGuard(curr_ms)) return;

	if(CarRun_State == 0) return;//停车模式不启动

	/* 段内里程：从「上一个弯转完」到现在走了多少厘米。（详见 Doc/代码说明.md §16） */
	Corner_SegMileage += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;

	/* 里程台架用：两轮累计计数，永不清零（段内里程每过一个弯会清零，量全程要用这个） */
	Odo_LeftTotal  += actual_left;//总左里程
	Odo_RightTotal += actual_right;//总右里程

	/* 终点兜底：最后一个弯转完之后按里程停车。 */
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


/* ===== 弯道目标速度：全文件只有这一处给 target_left/right 写「弯道值」===== */
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
 * 返回：1 = 接管了（target_left/right 已写好，调用者直接 return）
 */
static uint8_t Task_CornerDecide(uint8_t state)
{
	/* 制动期间：目标速度由 5 毫秒任务直接写，这里一律不插手。
	   不写 target 比写错安全：两套代码抢同一组目标最容易埋雷 */
	if (Corner_Active == 1)
	{
		return 1;
	}

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
		/* 正在转弯（包括里程强转）时绝不允许落到 PD —— 否则每 30 毫秒把转弯差速覆盖掉， */
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
			/* 交接期结束：这个弯彻底完事，Corner_Done 必须在这里清掉。 */
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
		return 0;
	}

#if TURN_USE_MILEAGE_TABLE
	if ((Corner_SegMileage < (Turn_ThresholdCm[Corner_Index] - TURN_DIST_EARLY_CM)) ||
	    (Corner_SegMileage > (Turn_ThresholdCm[Corner_Index] + TURN_DIST_LATE_CM)))
	{
		/* 里程对不上 = 这次报弯是假的，不认它。 */
		return 0;
	}
#endif

	Corner_Armed      = 1;
	Corner_ArmMileage = Corner_SegMileage;         /* 调试：记下报弯那一刻的里程 */
	Corner_ArmDist    = 0.0f;
	return 0;
}




/* 这个弯转够了没有？每 5 毫秒问一次。                         （详见 Doc/代码说明.md §17） */
static uint8_t Corner_TurnAngleReached(void)
{
	/* 第二道闸：编码器计数。陀螺仪半路坏了、角度不涨时，靠这个兜底。 */
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

			if (turned_angle_degrees >= Turn_TargetAngle_Deg)
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

		/* 峰值取样：只在 Corner_Active 期间跑，直道上的微小漂移碰不到这里 */
		if (Turn_TargetAngle_Deg > 10.0f)
		{
			float peak_now = Gyro_GetTotalAngle();
			if (peak_now < 0.0f) peak_now = -peak_now;
			if (peak_now > Corner_TurnPeakAngle) Corner_TurnPeakAngle = peak_now;
		}

#if TURN_USE_ANGLE_PD
		/* ===== 角度 PD：每 5 毫秒重算一次差速 ===== */
		if (Gyro_IsOk())
		{
			float a_now = Gyro_GetTotalAngle();
			float err_deg;
			float r_now;
			float diff_cmd;

			if (a_now < 0.0f) a_now = -a_now;                     /* 右转角度记成正的 */
			err_deg = Turn_TargetAngle_Deg - a_now;                /* 还差多少度 */
			r_now   = -((float)Gyro_GetAxisZRaw() - Gyro_GetZeroBias()) * GYRO_DPS_PER_COUNT;
			                                                       /* 顺时针为正，跟右转同向 */

			diff_cmd = (TURN_ANGLE_KP * err_deg) - (TURN_ANGLE_KD * r_now);

			if (diff_cmd >  TURN_ANGLE_DIFF_MAX) diff_cmd =  TURN_ANGLE_DIFF_MAX;
			if (diff_cmd < -TURN_ANGLE_DIFF_MAX) diff_cmd = -TURN_ANGLE_DIFF_MAX;

			/* 斜率限幅：陀螺仪单拍跳变会让 err 突然变负，不加这个差速会一拍打到反向满值 */
			if (diff_cmd > (Corner_DiffPrev + TURN_ANGLE_SLEW))
			{
				diff_cmd = Corner_DiffPrev + TURN_ANGLE_SLEW;
			}
			if (diff_cmd < (Corner_DiffPrev - TURN_ANGLE_SLEW))
			{
				diff_cmd = Corner_DiffPrev - TURN_ANGLE_SLEW;
			}
			Corner_DiffPrev = diff_cmd;

			target_left  = TURN_SPEED + diff_cmd;
			target_right = TURN_SPEED - diff_cmd;
			if (target_left  < 0.0f) target_left  = 0.0f;          /* 目标速度不给负值 */
			if (target_right < 0.0f) target_right = 0.0f;
		}
#endif

		/* 哑火兜底：编码器转过「目标角的 115%」而陀螺仪还不到 10 度，才判它哑火。按目标角缩放，小角度弯不会被拖到 85 度 */
		if (Gyro_IsOk() && (Turn_TargetAngle_Deg > 10.0f) && (Corner_Accum > (TURN_DIFF_TARGET * (Turn_TargetAngle_Deg / 90.0f) * TURN_STUCK_RATIO)))
		{
			float gyro_angle_now = Gyro_GetTotalAngle();
			if (gyro_angle_now < 0.0f) gyro_angle_now = -gyro_angle_now;
			if (gyro_angle_now < 10.0f)
			{
				/* 陀螺仪哑火：立刻按「转完了」处理，别再等 600 毫秒超时 */
				Corner_Accum = TURN_DIFF_TARGET * 100.0f;   /* 让它走进下面的结束分支 */
			}
		}

		/* 制动阶段：反向差速（内侧轮比外侧轮快），专门吃掉收手之后那 20 度的白转 */
		if (Corner_Braking == 1)
		{
			float brake_rate;
			uint16_t err_now;

			/* 这一拍陀螺仪读失败过：读数可能是陈旧的，不能用它判断「停没停」 */
			err_now = Gyro_GetReadErrorCount();
			if (err_now != Corner_BrakeErrLast)
			{
				Corner_BrakeErrLast = err_now;
				return;
			}

			/* 一阶低通：单拍跳变只占 40%，不会因为一个坏点就提前收手 */
			brake_rate = -((float)Gyro_GetAxisZRaw() - Gyro_GetZeroBias()) * GYRO_DPS_PER_COUNT;
			Corner_BrakeRateF += TURN_BRAKE_RATE_ALPHA * (brake_rate - Corner_BrakeRateF);
			brake_rate = Corner_BrakeRateF;
			if (brake_rate < 0.0f) brake_rate = -brake_rate;

			target_left  = TURN_SPEED - TURN_BRAKE_VALUE;
			target_right = TURN_SPEED + TURN_BRAKE_VALUE;

			/* 要连续几拍都安静才算停住，避免被一两个坏点骗过去 */
			if (brake_rate < TURN_BRAKE_RATE_DPS)
			{
				if (Corner_BrakeQuiet < 255) Corner_BrakeQuiet++;
			}
			else
			{
				Corner_BrakeQuiet = 0;
			}

			if ((Corner_BrakeQuiet >= TURN_BRAKE_QUIET_TICKS) || ((curr_ms - Corner_BrakeTick) > TURN_BRAKE_TIMEOUT_MS))
			{
				Corner_Braking = 2;   /* 2=已制动过，不许再进；落到下面走正常收手 */
			}
			else
			{
				return;               /* 还在转，这一拍不动状态机 */
			}
		}

		if(Corner_TurnAngleReached() || (curr_ms - Corner_Tick) > TURN_TIMEOUT_MS)//转角足够或超时
		{
			/* 角度刚好够（不是超时），陀螺仪还在：先别收手，进制动阶段 */
			/* Corner_Accum 被哑火兜底写成 TURN_DIFF_TARGET*100，用 10 倍做分界：兜底触发的这次不进制动 */
			if ((Corner_Braking == 0) && (Turn_TargetAngle_Deg > 10.0f) && Gyro_IsOk() && (Corner_Accum < (TURN_DIFF_TARGET * 10.0f))
				&& ((curr_ms - Corner_Tick) <= TURN_TIMEOUT_MS))
			{
				Corner_Braking   = 1;
				Corner_BrakeTick = curr_ms;
				Corner_BrakeRateF   = 0.0f;
				Corner_BrakeQuiet   = 0;
				Corner_BrakeErrLast = Gyro_GetReadErrorCount();
				return;               /* 下一拍开始制动 */
			}

			Corner_Active = 0;//转够了(或超时兜底),不再按弯道差速走
			Corner_Done   = 1;//锁存:这个弯已经转过,视觉再喊弯也不转第二次
			Corner_DoneTick = curr_ms;//记下转完的时刻,供"限时交回视觉"用
			/* 锁存这一弯：峰值角度 + 收手角度。峰值不到 10 度说明没真转，保留上一弯的 */
			if (Corner_TurnPeakAngle > 10.0f)
			{
				float end_now = Gyro_GetTotalAngle();
				if (end_now < 0.0f) end_now = -end_now;
				Corner_LastPeakAngle = Corner_TurnPeakAngle;
				Corner_LastEndAngle  = end_now;
			}

			Corner_SegMileage = 0.0f;

			Corner_ArmDist = 0.0f;

			/* 终点兜底：先判断「刚转完的这个弯是不是最后一个」，再决定编号加不加。 */
			if (Corner_Index + 1 >= TURN_TABLE_SIZE)
			{
				Finish_Armed = 1;
			}

			if (Corner_Index + 1 < TURN_TABLE_SIZE)
			{
				Corner_Index++;
			}
			Corner_TurnedForIndex = 0;   /* 进入下一段：本段锁存清掉 */

			/* 拐完这个弯，把方向环的「上一次偏差」对齐到当前值。 */
			pid_l.isFirstRun = 1;

			Corner_SetTarget(CORNER_T_EXIT);
		}
	}
	else if(Corner_Armed)//准备再往前一段距离开始转弯
	{
		//视觉报弯时,相机还看得远,车其实离弯道还有一段距离,先直着走过去
		Corner_ArmDist += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;//取均值减小单侧误差,最后换算为cm
		if(Corner_ArmDist >= TURN_ARM_CM)//报弯之后再走一段长度之后,开始正式转弯
		{
			if (Finish_Armed)
			{
				Corner_Armed = 0;
				Corner_ArmDist = 0.0f;
				return;
			}
			Corner_Armed  = 0;//解除准备状态
			Corner_Active = 1;//激活弯道状态:正式转弯
			Corner_TurnedForIndex = 1;   // 锁存：本段已经转过了（两条路共用）
			Corner_Accum  = 0.0f;//累计转角归零开始计数
			Corner_Tick   = curr_ms;//对齐当前时间戳

			/* 调试：把开转这一刻的段内里程锁存下来（见文件上方的说明） */
			Corner_TurnStartMileage = Corner_SegMileage;//标定使用,锁存转弯时间

			Turn_TargetAngle_Deg = Turn_AngleDeg[Corner_Index];//查表要转的角度
			Corner_TurnPeakAngle = 0.0f;   /* 新的一弯，峰值从头记 */
			Corner_DiffPrev = 0.0f;       /* 新的一弯，差速从 0 起步 */

			/* 转弯就从这一刻算起，陀螺仪角度清零。 */
			Gyro_ResetTotalAngle();

			/* 开转这一刻就把目标切成转弯差速，不等下一个 30 毫秒。 */
			Corner_SetTarget(CORNER_T_TURN);
		}
	}
}


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

	/* 3. 逐轮判：哪个轮子「被要求快跑 + 却在原地不动 + 速度环已经正向顶死」就算它堵转。 */
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
        if(Vision_EverSeen == 0)
        {
            return;
        }
        CarRun_Set(!CarRun_State);
    }

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
    Line_Integral = 0.0f;//清空方向环积分项
    pid_l.err_diff_last = 0;//清空方向环上次原始偏差
    pid_l.err_diff = 0;//清空本次原始偏差
    Corner_Armed   = 0;//清空弯道准备状态
    Corner_Active  = 0;//清空弯道进行时状态
    Corner_Done    = 0;//清空转弯完毕标志
    Corner_Clear   = 3;//强制清空视觉判弯次数
	Corner_Braking = 0;//清空制动状态
    Corner_ArmDist = 0.0f;//清空视觉报弯后走过的路程
    Corner_Accum   = 0.0f;//清空转弯累计角度
    Corner_DoneTick = 0;//清空转弯完的时刻
    Corner_Index         = 0;//清空弯道序号,下次从直角开始
    Corner_TurnedForIndex = 0;//清空「本段已转过」锁存
    Corner_SegMileage    = 0.0f;//清空上个弯道结束到现在走过的路程
    Finish_Armed         = 0;//清空终点兜底标志,下次跑重新等最后一个弯
    Turn_TargetAngle_Deg = TURN_TARGET_ANGLE_DEG;//恢复保守角度,下次被查表值覆盖

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


static void Task_SendTelemetry(void)
{
    debugdata_t data;

#if (TELEMETRY_MODE == 3)//调试模式3:陀螺仪调试
    /* ===== 3 陀螺仪台架 ===== */
    data.target_left  = (float)Gyro_GetWhoAmIValue();//实际发送当前陀螺仪的编号
    data.actual_left  = (float)Gyro_GetAxisXRaw();//实际发送X轴原始数据(没用,为0)
    data.target_right = (float)Gyro_GetAxisYRaw();//实际发送Y轴原始数据(没用,为0)
    data.actual_right = (float)Gyro_GetAxisZRaw();//实际发送Z轴原始数据,用于处理转角
    data.error        = Gyro_GetTotalAngle();//实际发送当前转角
    data.mileage_arm  = (float)Gyro_GetReadErrorCount();//台架 ch6：陀螺仪读失败总次数
    data.mileage_turn = 0.0f;//正式开始转那一刻的段内里程(cm)

#elif (TELEMETRY_MODE == 1)//调试模式1:视觉调试
    /* ===== 1 视觉台架 ===== */
    data.target_left  = (float)last_vision_state;//实际发送上一次接收到的视觉状态(0123)
    data.actual_left  = last_vision_error;//实际发送视觉计算的偏差
    data.target_right = Corner_SegMileage;//实际发送此时段内走过的里程
    data.actual_right = Corner_ArmMileage;//实际发送视觉判弯时已走过的段内里程
    data.error        = Corner_TurnStartMileage;//实际发送真正开始转那一刻的段内里程
    data.mileage_arm  = (float)VisionFrameCount;//实际发送已收到的视觉帧数
    data.mileage_turn = (float)VisionLostTicks;//实际发送视觉已经丢线的帧数

#elif (TELEMETRY_MODE == 2)//调试模式2:里程调试
    /* ===== 2 里程台架 ===== */
    data.target_left  = Odo_LeftTotal;//实际发送左轮累计计数
    data.actual_left  = Odo_RightTotal;//实际发送右轮累计计数
    data.target_right = Corner_SegMileage;//实际发送此时的段内里程
    data.actual_right = (Odo_LeftTotal + Odo_RightTotal) * 0.5f * CM_PER_COUNT;//实际发送计算的走过的全部里程
    data.error        = Corner_ArmMileage;//实际发送视觉报弯时的里程
    data.mileage_arm  = Corner_TurnStartMileage;//实际发送开始转弯时的里程
    data.mileage_turn = 0.0f;//没数据发:占位

#else
    /* ===== 0 跑车（默认，跑的时候发这一组）===== */
    data.target_left  = target_left;
    data.actual_left  = actual_left;
    data.target_right = target_right;
    data.actual_right = Gyro_IsOk() ? Gyro_GetTotalAngle() : -999.0f;
    data.error        = last_vision_error;
    data.mileage_arm  = (float)Gyro_GetReadErrorCount();   /* 诊断：陀螺仪读失败总次数 */
    data.mileage_turn = Corner_LastPeakAngle;    /* 诊断：上一个弯峰值角度(度) */
#endif

    /* 下面这四行只是"用一下"这些变量，把编译告警消掉。 */
    (void)VisionFrameCount;
    (void)VisionLostTicks;
    (void)last_vision_state;
    (void)Calib_Frozen;
    (void)Corner_ArmMileage;        /* 诊断版暂时不发这两个数，先压掉告警 */
    (void)Corner_TurnStartMileage;
    (void)Corner_LastEndAngle;      /* 这一版 I6 让给峰值角度，收手角度先不发 */

    Debug_log(&data);
}


// 5. 获取视觉数据与丢包保护(30ms)
#define VISION_LOST_TICKS_RUN    10     /* >10 → 连续 11 次没帧 → 330ms  */
#define VISION_LOST_TICKS_TURN   33     /* >33 → 连续 34 次没帧 → 1020ms */

static uint8_t Task_FetchVisionData(VisionData_t *vis_data)
{
    static uint8_t lostdata_time = 0; 
    
    if(Protocol_GetVision(vis_data) == 0)//没有新数据
    {
        lostdata_time++;
        VisionLostTicks = lostdata_time;   /* 视觉台架显示用：连续没拿到几次 */
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

    /* 这个数现在**只干一件事：把视觉零点对齐到 0**，不再承担治漂移的任务。 */
    vision_error += LINE_ERR_OFFSET;
    last_vision_error = vision_error;   /* 波形也发补偿后的，方便对账 */

    float raw_error = vision_error;//喂给PD的那一份：先限幅
    if (raw_error >  LIMIT_LINE_ERR_MAX) raw_error =  LIMIT_LINE_ERR_MAX;
    if (raw_error < -LIMIT_LINE_ERR_MAX) raw_error = -LIMIT_LINE_ERR_MAX;

    /* ===== 按路段把偏差往外推（治钝角弯那一段必然的右偏）===== */
    {
        float dist_to_corner = Turn_ThresholdCm[Corner_Index] - Corner_SegMileage;
        float this_bias = Turn_LineBias[Corner_Index];
        if ((this_bias != 0.0f) && (dist_to_corner < TURN_BIAS_WINDOW_CM)
            && (dist_to_corner > -20.0f))
        {
            float k = (TURN_BIAS_WINDOW_CM - dist_to_corner) / TURN_BIAS_WINDOW_CM;
            if (k > 1.0f) k = 1.0f;
            raw_error += this_bias * k;
        }
    }

#if (TURN_USE_MILEAGE_TABLE && TURN_ALLOW_FORCE)
    if ((Corner_TurnedForIndex == 0) && (Corner_Active == 0)
        && (Finish_Armed == 0)                     /* 最后一个弯转完就不许再转了 */
        && (Corner_SegMileage >= CORNER_COOLDOWN_CM)
        && (Corner_SegMileage > (Turn_ThresholdCm[Corner_Index] + TURN_FORCE_CM)))
    {
        Corner_Armed   = 0;                            /* 已经在等 16 厘米的话，取消它 */
        Corner_ArmDist = 0.0f;                         /* 免得残留到下一个弯 */
        Corner_Active  = 1;
		Corner_Braking = 0;                            /* 新的一次转弯，先退出制动状态 */
        Corner_TurnedForIndex = 1;                     /* 锁存：本段已经转过了 */
        Line_Integral  = 0.0f;                         /* 进弯后车身姿态大变，旧积分没意义 */
        Corner_Accum   = 0.0f;
        Corner_Tick    = App_Clock();
        Corner_TurnStartMileage = Corner_SegMileage;   /* 调试：强转也记开转里程 */
        Turn_TargetAngle_Deg = Turn_AngleDeg[Corner_Index];
			Corner_TurnPeakAngle = 0.0f;   /* 新的一弯，峰值从头记 */
			Corner_DiffPrev = 0.0f;       /* 新的一弯，差速从 0 起步 */
        Gyro_ResetTotalAngle();
        Corner_SetTarget(CORNER_T_TURN);   /* 强转第一拍就按转弯差速走 */
    }
#endif

	/* 弯道拍板：状态机接管了目标速度就 return；没接管就往下走 PD。 */
	if (Task_CornerDecide(state)) return;

    float error = raw_error;
    if(fabsf(error) <= LINE_ERR_DEADBAND)
    {
        error = 0.0f;
    }


    if ((Corner_Active == 0) && (Corner_Armed == 0) && (state != 2)
        && (fabsf(raw_error) < LINE_I_FREEZE_ERR))
    {
        Line_Integral += LINE_KI * raw_error * 0.03f;   /* 0.03 = 30 毫秒一拍 */
        if (Line_Integral >  LINE_I_MAX) Line_Integral =  LINE_I_MAX;
        if (Line_Integral < -LINE_I_MAX) Line_Integral = -LINE_I_MAX;
    }

    float turn = PID_L_Calc(&pid_l,error,raw_error) + Line_Integral;
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

    /* 报弯之后「等那 16 厘米」这段时间：速度压到 15 档，但方向仍然交给方向环。 */
    if (Corner_Armed && (base_speed > TURN_ARM_SPEED))
    {
        base_speed = TURN_ARM_SPEED;
    }
    /* 但这段是整条赛道上视觉最不可靠的时候（弯口附近九行里常常只剩 1~3 行）， */
    if (Corner_Armed)
    {
        if (turn >  TURN_ARM_TURN_MAX) turn =  TURN_ARM_TURN_MAX;
        if (turn < -TURN_ARM_TURN_MAX) turn = -TURN_ARM_TURN_MAX;
    }

    if (turn >  TURN_STRAIGHT_DIFF_MAX) turn =  TURN_STRAIGHT_DIFF_MAX;
    if (turn < -TURN_STRAIGHT_DIFF_MAX) turn = -TURN_STRAIGHT_DIFF_MAX;

    target_left = base_speed + turn;
    target_right = base_speed - turn;
}


static void Task_Indicate(void)
{
    uint32_t now_ms = App_Clock();
    uint8_t  blink_ticks;

    // 台架标定模式:双闪(闪两下、停一下,周期 1.2 秒)。
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


/* 启停状态的唯一写入口。 */
static void CarRun_Set(uint8_t run)
{
    CarRun_State = (run != 0) ? 1 : 0;
}
