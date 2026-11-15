/* ==========================================================================
 * main.c  ——  速度环台架调试程序（不需要摄像头）
 *
 * 【这个程序干什么】
 *   只跑"编码器 -> 速度环PI -> 电机"这一条链路，不跑视觉、不跑方向环。
 *   因为没有视觉数据，所以这里【写死一个目标速度】(默认 40)，
 *   让轮子转起来，你才好调 Kp / Ki。
 *
 * 【调好的参数怎么用】
 *   记进 Doc/调参记录表.xlsx
 *   -> 最后写回 Service.c 的 PID_VelocityLoop_Init(...) 和重新编译
 *
 * 【VOFA+ 5 个通道】
 *   ch0  左轮目标速度（正常应该是一条 40 的横线）
 *   ch1  左轮实际速度（调 Kp/Ki 就是让它追上 ch0）
 *   ch2  右轮目标速度
 *   ch3  右轮实际速度
 *   ch4  软件算出的占空比 PWM(‰)  <== 排查"轮子不转"靠这一条
 *
 *        ch4 = 0     -> 软件没输出：要么没进运行状态，要么 PID 输出是 0
 *        ch4 有数    -> 软件在输出，轮子不转就是硬件问题（电池没接/驱动没使能）
 *
 * 【设备载灯】
 *   运行中 = 常亮      停车 = 慢闪(500ms)
 *
 * 【蓝牙指令】
 *   0x01 左轮 Kp     0x02 左轮 Ki
 *   0x03 右轮 Kp     0x04 右轮 Ki
 *   0x07 目标速度（左右轮一起改，0~100）
 *   0x0A 启停（1=跑 0=停）
 *
 * 【怎么开始】
 *   1.把两个轮子架空（离地）
 *   2.烧录，上电
 *   3.按一下板上的按键 -> 轮子开始转（灯变常亮）
 *   4.用 VOFA+ 发 0x01~0x04 调 Kp/Ki，看 ch1 追 ch0
 *   5.再按一下按键停车
 * ========================================================================== */

#include <stdint.h>
#include "stm32f10x.h"

#include "SysTimer.h"
#include "BlueTooth.h"
#include "Debug.h"
#include "Protocol.h"
#include "IndicatorLED.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "Key.h"

/* ---------------- 可调默认值 ---------------- */
//直线目标33是合理的
#define DEFAULT_TARGET      33.0f   /* 默认目标速度 counts/5ms，和正式固件的直道速度一致 */
#define DEFAULT_KP          30.0f   /* 速度环 Kp 起始值（先从最大推，保证轮子能转起来） */
#define DEFAULT_KI          20.0f   /* 速度环 Ki 起始值 */

/* ---------------- 任务周期(ms) ---------------- */
#define PERIOD_5MS          5
#define PERIOD_10MS         10
#define PERIOD_100MS        100

/* ---------------- 调参指令码(与 Protocol.h 一致) ---------------- */
#define CMD_L_KP            0x01
#define CMD_L_KI            0x02
#define CMD_R_KP            0x03
#define CMD_R_KI            0x04
#define CMD_TARGET_SPEED    0x07
#define CMD_RUN_STATE       0x0A

/* ---------------- 全局状态 ---------------- */
static pid_velocity_t pid_left;
static pid_velocity_t pid_right;

static uint8_t  run_state    = 0;                 /* 0=停车 1=运行 */
static float    target_speed = DEFAULT_TARGET;    /* 目标速度 counts/5ms */
static float    actual_left  = 0.0f;
static float    actual_right = 0.0f;
static float    pwm_out      = 0.0f;              /* 最近一次真正送进电机的占空比（‰） */

static uint32_t last_5ms   = 0;
static uint32_t last_10ms  = 0;
static uint32_t last_100ms = 0;

static uint8_t  blink_cnt  = 0;   /* 停车慢闪计数器：5 次 100ms = 500ms 翻转一次 */

/* 功能：停车时的清理，防止积分饱和
 * 说明：这和正式固件里 Task_CarStop() 做的事一样 */
static void Bench_Stop(void)
{
    Motor_SetDuty(0, 0);
    pid_left.err_integral  = 0;
    pid_left.err           = 0;
    pid_right.err_integral = 0;
    pid_right.err          = 0;
}

/* 功能：处理一条蓝牙调参指令
 * 参数：cmd 解析出来的指令（指令码 + 数值） */
/* Kp / Ki 调参上限。
 * 正式固件 Service.c 里是 10（LIMIT_V_KP_MAX / LIMIT_V_KI_MAX），
 * 台架这里放宽到 30，是为了先试出"到底需要多大 Kp 才能把误差压进 20"。
 * 测出来超过 10 的话，正式固件那个上限也得跟着改。 */
#define BENCH_LIMIT_KP      30.0f
#define BENCH_LIMIT_KI      30.0f

/* 功能：处理一条蓝牙调参指令
 * 参数：cmd 解析出来的指令（指令码 + 数值） */
static void Bench_ApplyCmd(BtCommand_t *cmd)
{
    /* 第一道安检：拦截 NaN / Inf 这种乱码 */
    if (!(cmd->param_val > -1e6f && cmd->param_val < 1e6f)) return;

    switch (cmd->cmd_type)
    {
    case CMD_L_KP:                                  /* 左轮 Kp，限幅 0~30 */
        if (cmd->param_val >= 0.0f && cmd->param_val <= BENCH_LIMIT_KP) pid_left.Kp = cmd->param_val;
        break;
    case CMD_L_KI:                                  /* 左轮 Ki，限幅 0~30 */
        if (cmd->param_val >= 0.0f && cmd->param_val <= BENCH_LIMIT_KI) pid_left.Ki = cmd->param_val;
        break;
    case CMD_R_KP:                                  /* 右轮 Kp，限幅 0~30 */
        if (cmd->param_val >= 0.0f && cmd->param_val <= BENCH_LIMIT_KP) pid_right.Kp = cmd->param_val;
        break;
    case CMD_R_KI:                                  /* 右轮 Ki，限幅 0~30 */
        if (cmd->param_val >= 0.0f && cmd->param_val <= BENCH_LIMIT_KI) pid_right.Ki = cmd->param_val;
        break;
    case CMD_TARGET_SPEED:                          /* 目标速度，限幅 0~100 */
        if (cmd->param_val >= 0.0f && cmd->param_val <= 100.0f) target_speed = cmd->param_val;
        break;
    case CMD_RUN_STATE:                             /* 启停：>0.5 视为启动 */
        run_state = (cmd->param_val > 0.5f) ? 1 : 0;
        break;
    default:
        break;                                      /* 不认识的指令直接忽略 */
    }
}

int main(void)
{
    /* ---------------- 初始化 ---------------- */
    SysTimer_Init();        /* 时基 */
    BlueTooth_Init();       /* PA3 接收 + USART2 参数(必须排在 Debug_Init 之前) */
    Debug_Init();           /* PA2 发送波形 */
    Indicator_Init();       /* 板载灯 */
    Key_Init();             /* 板上按键，用来启停 */
    Encoder_Init();         /* 左右编码器 */
    Motor_Init();           /* 电机 PWM + 方向 */

    /* 速度环 PI：Kp/Ki 用默认值，输出限幅 ±1000‰ */
    PID_VelocityLoop_Init(&pid_left,  DEFAULT_KP, DEFAULT_KI,
                          (float)MOTOR_DUTY_FULLSCALE, -(float)MOTOR_DUTY_FULLSCALE);
    PID_VelocityLoop_Init(&pid_right, DEFAULT_KP, DEFAULT_KI,
                          (float)MOTOR_DUTY_FULLSCALE, -(float)MOTOR_DUTY_FULLSCALE);

    last_5ms   = SysTimer_GetTick();
    last_10ms  = last_5ms;
    last_100ms = last_5ms;

    /* ---------------- 主循环 ---------------- */
    while (1)
    {
        uint32_t now = SysTimer_GetTick();

        /* ===== 5ms：速度环（内环）===== */
        if ((now - last_5ms) >= PERIOD_5MS)
        {
            last_5ms = now;

            Encoder_Update();                               /* 采样编码器 */
            actual_left  = (float)Encoder_GetDeltaLeft();
            actual_right = (float)Encoder_GetDeltaRight();

            if (run_state == 1)
            {
                float pwm_left  = PID_V_Calc(&pid_left,  actual_left,  target_speed);
                float pwm_right = PID_V_Calc(&pid_right, actual_right, target_speed);
                Motor_SetDuty((int16_t)pwm_left, (int16_t)pwm_right);
                pwm_out = pwm_left;                         /* 记下来给 ch4 看 */
            }
            else
            {
                Bench_Stop();                               /* 停车时每 5ms 清一次，防止积分堆积 */
                pwm_out = 0.0f;
            }
        }

        /* ===== 10ms：按键扫描 ===== */
        /* ===== 10ms：读蓝牙指令 + 按键 + 发波形 =====
         * 波形原来是 100ms 一帧，但电机从 0 爬到目标只要几十毫秒，
         * 100ms 采一次等于完全看不到过程，只能看到结果。
         * 24 字节在 115200 下约 2ms，10ms 发一帧完全够用。 */
        if ((now - last_10ms) >= PERIOD_10MS)
        {
            debugdata_t dbg = {0};   /* 先全清零：被注释掉的通道会发 0，不会发乱值 */
            uint8_t     rx_byte;
            BtCommand_t cmd;

            last_10ms = now;

            /* 1.收：把环形队列读干净，逐字节喂给协议状态机 */
            while (BlueTooth_ReadByte(&rx_byte) == 1)
            {
                if (Protocol_BtParse(rx_byte, &cmd) == 1)
                {
                    Bench_ApplyCmd(&cmd);
                }
            }

            /* 2.按键扫描 + 启停翻转 */
            Key_CheckRequest(now);
            if (Key_GetRequest() == 1)
            {
                run_state = (run_state == 0) ? 1 : 0;
            }

            /* 3.发波形（5 个通道全发） */
            dbg.target_left  = target_speed;    /* ch0 左轮目标 */
            dbg.actual_left  = actual_left;     /* ch1 左轮实际 */
            dbg.target_right = target_speed;    /* ch2 右轮目标 */
            dbg.actual_right = actual_right;    /* ch3 右轮实际 */
            dbg.error        = pwm_out;         /* ch4 左轮占空比(‰)，看是不是打满 1000 */

            Debug_log(&dbg);
        }

        /* ===== 100ms：刷灯 ===== */
        if ((now - last_100ms) >= PERIOD_100MS)
        {
            last_100ms = now;

            /* 运行=常亮，停车=慢闪(500ms) */
            if (run_state == 1)
            {
                blink_cnt = 0;
                Indicator_TurnOn();
            }
            else
            {
                blink_cnt++;
                if (blink_cnt >= 5)     /* 5 x 100ms = 500ms 翻转一次 */
                {
                    blink_cnt = 0;
                    Indicator_Toggle();
                }
            }
        }
    }
}


/* ==========================================================================
 * 以下是【正式固件入口】，原样注释掉保留。
 * 速度环调完、参数写回 Service.c 之后，把上面整段删掉，再把下面解开即可。
 * ========================================================================== */
// #include "Service.h"
// int main(void)
// {
// 	Service_Init();
// 	while(1)
// 	{
// 		ServiceTask_Idle();
// 		uint32_t timestamp = Service_Clock();
// 		ServiceTask_5ms(timestamp);
// 		ServiceTask_10ms(timestamp);
// 		ServiceTask_30ms(timestamp);
// 		ServiceTask_100ms(timestamp);
// 	}
// }
//调试:VOFA+ JustFloat协议
