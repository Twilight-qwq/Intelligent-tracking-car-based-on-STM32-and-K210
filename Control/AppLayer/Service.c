#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "Service.h"
static void Handle_StartStopCmd(void);
static void Handle_CarStop(void);
static void Handle_BtTuning(void);
static void Send_Telemetry(void);
static uint8_t Fetch_VisionData(VisionData_t *vis_data);
static void Calculate_TargetSpeed(VisionData_t *vis_data);

static uint32_t task_last_5ms = 0;
static uint32_t task_last_10ms = 0;
static uint32_t task_last_30ms = 0;
static uint32_t task_last_100ms = 0;

static pid_velocity_t pid_v_left;
static pid_velocity_t pid_v_right;
static pid_line_t pid_l;

static uint8_t CarRun_State = 0;
static float Speed_Straight   = 40.0f; //直道基速
static float Speed_SmallAngle = 25.0f; //小角度(直角,钝角)基速
static float Speed_LargeAngle = 15.0f; //大角度(锐角)基速
static float target_left = 0.0f;
static float target_right = 0.0f;
static float actual_left = 0.0f;
static float actual_right = 0.0f;
static float last_vision_error = 0.0f; // 用于记录上一次的视觉误差，给 100ms 发波形

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
	k210rcv_Init();
	SysTimer_Init();
	PID_VelocityLoop_Init(&pid_v_left,0,0,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//后面再调参
	PID_VelocityLoop_Init(&pid_v_right,0,0,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//后面再调参
	PID_LineLoop_Init(&pid_l,0,0,Speed_Straight,-(float)Speed_Straight);//后面再调参
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
    if(Fetch_VisionData(&vis_data) == 0) return;//没拿到新数据直接结束避免用旧数据算
    Calculate_TargetSpeed(&vis_data);
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

    Handle_StartStopCmd();//1.获取启停指令
    if(CarRun_State == 0) Handle_CarStop();//2.如果停车了执行安全清理
    Handle_BtTuning();//3.检查手机蓝牙有没有发来调参指令
    Send_Telemetry();//4.往VOFA+发送波形数据
}














// 1. 处理物理按键启停(100ms)
static void Handle_StartStopCmd(void)
{
    // 目前：处理物理按键
    if(Key_GetRequest() == 1)
    {
        CarRun_State = !CarRun_State;   
    }

    /* 未来在这里直接加上语音的代码，无需改动其他任何地方：
    uint8_t voice_cmd = Voice_GetCommand();
    if (voice_cmd == 0x01) CarRun_State = 1;
    else if (voice_cmd == 0x02) CarRun_State = 0;
    */
}

//2.停车时的安全清理,防止积分饱和(100ms)
static void Handle_CarStop(void)
{
    Motor_SetDuty(0,0);
    target_left = 0;
    target_right = 0;
    pid_v_left.err_integral = 0;
    pid_v_left.err = 0;
    pid_v_right.err_integral = 0;
    pid_v_right.err = 0;
    pid_l.err_last = 0;
    pid_l.err = 0;
}


//蓝牙调参范围限制(参数上限/下限)
#define LIMIT_V_KP_MAX      10.0f  //内环速度环Kp上限
#define LIMIT_V_KI_MAX      10.0f  //内环速度环Ki上限
#define LIMIT_LINE_KP_MAX   50.0f  //外环方向环Kp上限
#define LIMIT_LINE_KD_MAX   100.0f //外环方向环Kd上限
#define LIMIT_SPEED_MAX     100.0f //基础速度上限
//3.处理蓝牙调参指令(100ms)
static void Handle_BtTuning(void)
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
static void Send_Telemetry(void)
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
static uint8_t Fetch_VisionData(VisionData_t *vis_data)
{
    static uint8_t lostdata_time = 0; 
    
    if(Protocol_GetVision(vis_data) == 0)//没有新数据
    {
        lostdata_time++;
        if(lostdata_time > 3)
        {
            CarRun_State = 0;//防止摄像头出问题了小车还在跑
            Motor_SetDuty(0,0);
            lostdata_time = 0;
        }
        return 0; // 没拿到
    }
    
    lostdata_time = 0;//收到数据,清空超时计数
    return 1; // 拿到了
}

// 6. 计算方向环与控制输出(30ms)
static void Calculate_TargetSpeed(VisionData_t *vis_data)
{
    // 更新用于Debug发波形的误差
    float error = (float)vis_data->error;
    last_vision_error = error;

    if(fabsf(error)<=3)
    {
        error = 0.0f;//忽略细微抖动
    }

    float turn = PID_L_Calc(&pid_l,error);//差速大小
    float base_speed = Speed_Straight;

    switch (vis_data->car_state)
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
