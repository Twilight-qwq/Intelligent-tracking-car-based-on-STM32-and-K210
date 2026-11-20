#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "Service.h"

static void Task_StartStopCmd(void);
static void Task_CarStop(void);
static void Task_BtCommand(void);
static void Task_SendTelemetry(void);
static uint8_t Task_FetchVisionData(VisionData_t *vis_data);
static void Task_CalculateTargetSpeed(VisionData_t *vis_data);
static void Task_Indicate(void);
static void CarRun_Set(uint8_t run);

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
#define TURN_SPEED         15.0f    // 转弯时基速(= Speed_LargeAngle)
#define TURN_VALUE         11.55f   // 转弯时差速(= Kp 0.231 * 视觉固定值 50)
#define TURN_TIMEOUT_MS    800      // 兜底:0.8秒还没转够就强制结束(卡住了也不至于一直转)
#define TURN_ARM_CM        20.0f    // 视觉报弯后,还要再直走这么多"换算厘米"才真的开始转
                                    // 标定记录:填45时实测只走了约28厘米(比例0.62)
                                    // 2026-11-20 夜里实车:填30时转弯略晚(压线后能自己纠回来),改20试
#define CM_PER_COUNT       0.01405f // 1 个编码器计数 = 多少厘米(30档=30counts/5ms=84.3cm/s)
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
static uint32_t Corner_DoneTick = 0;   // 转完90度的时刻, 供"限时交回视觉"用
static float    Corner_Cool     = 99999.0f; // 冷却已走的距离(厘米), 大数=不冷却

static uint8_t  Indicator_Tick    = 0;   //闪烁计数
static uint8_t  Indicator_Fault   = 0;   //视觉断线锁存, 收到新数据才清除
static uint32_t Vision_LastTick   = 0;   //最后一次收到视觉数据的时刻
static uint8_t  CarRun_PrevState  = 0;   //上一次的运行状态, 用于捕捉启动瞬间
static uint8_t  Vision_EverSeen   = 0;   //上电后有没有收到过视觉数据(没就绪前不许启动)

uint32_t Service_Clock(void)
{
	return SysTimer_GetTick();
}

void Service_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Encoder_Init();
	Motor_Init();
	Key_Init();//后面删掉换成语音模块
	Voice_Init();
	BlueTooth_Init();
	Debug_Init();
	Indicator_Init();
	k210rcv_Init();
	SysTimer_Init();
	PID_VelocityLoop_Init(&pid_v_left,100.0f,10.0f,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//台架调定 Kp=100 Ki=10
	PID_VelocityLoop_Init(&pid_v_right,100.0f,10.0f,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//台架调定，左右同参数
	PID_LineLoop_Init(&pid_l,0.231f,0.53f,Speed_Straight,-(float)Speed_Straight);//方向环 Kp=0.231 Kd=0.53（2026-11-20 实车调定：0.5 画龙、0.1 偏钝，0.231/0.53 直道可用）
	//Kp 上限 ≈ 1 / (k × 死区时间)
    WatchDog_Init();
}


void ServiceTask_Idle(void)
{
	WatchDog_FeedDog();
}


void ServiceTask_5ms(uint32_t curr_ms)//内环速度环控制
{
	if(curr_ms - task_last_5ms < 5) return;
    task_last_5ms = curr_ms; //重置闹钟

	Encoder_Update();
	actual_left  = (float)Encoder_GetDeltaLeft();
	actual_right = (float)Encoder_GetDeltaRight();

	if(CarRun_State == 0) return;

	//弯道冷却:转完一个弯之后要再走够 CORNER_COOLDOWN_CM 才允许下一个弯
	if(Corner_Cool < CORNER_COOLDOWN_CM)
	{
		Corner_Cool += ((actual_left + actual_right) * 0.5f) * CM_PER_COUNT;
	}

	//弯道转向:每5毫秒把(左轮-右轮)的增量加进累计值,加满目标角度就停
	if(Corner_Active)
	{
		Corner_Accum += (actual_left - actual_right);
		if(Corner_Accum >= TURN_DIFF_TARGET || (curr_ms - Corner_Tick) > TURN_TIMEOUT_MS)
		{
			Corner_Active = 0;//转够90度了(或超时兜底),不再按弯道差速走
			Corner_Done   = 1;//锁存:这个弯已经转过,视觉再喊弯也不转第二次
			Corner_DoneTick = curr_ms;//记下转完的时刻,供"限时交回视觉"用
			Corner_Cool   = 0.0f;//冷却从这一刻开始重新算
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
		}
	}

	float pwm_left = PID_V_Calc(&pid_v_left,actual_left,target_left);
	float pwm_right = PID_V_Calc(&pid_v_right,actual_right,target_right);
	Motor_SetDuty((int16_t)pwm_left,(int16_t)pwm_right);
}


void ServiceTask_30ms(uint32_t curr_ms)//外环方向环控制
{
	if(curr_ms - task_last_30ms < 30) return;
    task_last_30ms = curr_ms;
    if(CarRun_State == 0) return;

    VisionData_t vis_data;
    if(Task_FetchVisionData(&vis_data) == 0) return;//没拿到新数据直接结束避免用旧数据算
    Task_CalculateTargetSpeed(&vis_data);
}


void ServiceTask_10ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_10ms < 10) return;

    task_last_10ms = curr_ms;
    Key_CheckRequest(curr_ms);
}


void ServiceTask_100ms(uint32_t curr_ms)//开关控制与数据发送
{
	
    if(curr_ms - task_last_100ms < 100) return;
    task_last_100ms = curr_ms;

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

    /* 未来在这里直接加上语音的代码，无需改动其他任何地方：
    uint8_t voice_cmd = Voice_GetCommand();
    if (voice_cmd == 0x01) CarRun_State = 1;
    else if (voice_cmd == 0x02) CarRun_State = 0;
    */
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
    Corner_Cool   = 99999.0f;//停车复位:下次启动第一个弯不该被冷却挡住
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

//4.发送 VOFA+调试波形(100ms)
static void Task_SendTelemetry(void)
{
    debugdata_t data;
    data.target_left  = target_left;
    data.actual_left  = actual_left;
    data.target_right = target_right;
    data.actual_right = actual_right;
    data.error        = last_vision_error; // 使用刚才记录的误差
    Debug_log(&data);
}

// 5. 获取视觉数据与丢包保护(30ms)
static uint8_t Task_FetchVisionData(VisionData_t *vis_data)
{
    static uint8_t lostdata_time = 0; 
    
    if(Protocol_GetVision(vis_data) == 0)//没有新数据
    {
        lostdata_time++;
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

    /* ---------- 弯道转向状态机（三段） ----------
     * ① 视觉报弯：它看得比车走得远，这时候车离弯道还有一段，先直行过去
     * ② 走够 TURN_ARM_CM：开始按固定差速转
     * ③ 累计(左轮-右轮)达到 TURN_DIFF_TARGET：转够了，立刻停并锁存
     * 视觉只负责"这里有弯"；什么时候开始转、转到什么时候，由轮子说了算。 */
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
                target_left  = Speed_Straight;
                target_right = Speed_Straight;
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
            if (Corner_Cool < CORNER_COOLDOWN_CM)
            {
                // 冷却中:上一个弯刚转完,车还没走够距离。
                // 这时候视觉再喊弯,多半是同一个弯还没从画面里退出去 —— 当直道走,不转。
                target_left  = Speed_Straight;
                target_right = Speed_Straight;
                return;
            }
            Corner_Armed = 1;
            target_left  = TURN_ARM_SPEED;
            target_right = TURN_ARM_SPEED;
            return;
        }
    }
    else
    {
        //要连续 3 帧(300毫秒)都说"不是弯",才算这个弯真的过去了。
        //只掉一帧就清空的话:里程被清零要重走、锁存被清掉会再转一次,就是"反复画圈"的来源。
        if (Corner_Clear < 3) Corner_Clear++;
        if (Corner_Clear >= 3)
        {
            Corner_Armed   = 0;
            Corner_Active  = 0;
            Corner_Done    = 0;
            Corner_ArmDist = 0.0f;
            Corner_Accum   = 0.0f;
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

    //启动瞬间重新计时, 避免上电期间的空窗被判成断线
    if (CarRun_State == 1 && CarRun_PrevState == 0)
    {
        Vision_LastTick = now_ms;
    }
    CarRun_PrevState = CarRun_State;

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
