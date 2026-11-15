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
 * 【VOFA+ 6 个通道】（顺序固定，改这里必须同步改 VOFA+ 的通道数）
 *   ch0  左轮目标速度  counts/5ms
 *   ch1  左轮实际速度
 *   ch2  左轮占空比 PWM(‰)   <== 真正送进电机的那一个数
 *   ch3  右轮目标速度
 *   ch4  右轮实际速度
 *   ch5  右轮占空比 PWM(‰)
 *
 *   怎么用：
 *     ch0/ch3 是目标，ch1/ch4 是实际 -> 看实际能不能追上目标，这就是 Kp/Ki 好不好
 *     ch2/ch5 是真正给电机的占空比，范围 ±1000‰
 *     ch2 = 0 但 ch0 不为 0 -> 软件没输出：没进运行状态，或 PID 输出是 0
 *     ch2 有数但轮子不转   -> 硬件问题：电池没接 / 驱动没使能 / 线接反
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
#define DEFAULT_TARGET      25.0f   /* 默认目标速度 counts/5ms，和正式固件的直道速度一致 */
#define DEFAULT_KP          100.0f  /* 速度环 Kp 定稿值：20->25 小阶跃能打满 1000‰；
                                     * 再往上只放大噪声，不会更快 */
#define DEFAULT_KI          7.3f    /* 速度环 Ki 定稿值：让积分把最后 1 个 count 追得快，
                                     * 实测 20->25 小阶跃 25ms 到位 */

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
static float    pwm_left_out  = 0.0f;             /* 左轮最近一次送进电机的占空比（‰） */
static float    pwm_right_out = 0.0f;             /* 右轮最近一次送进电机的占空比（‰） */

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
/* Kp / Ki 调参上限（蓝牙下发的数值超过这个范围会被直接丢掉）。
 * 正式固件 Service.c 里 LIMIT_V_KP_MAX = 30、LIMIT_V_KI_MAX = 20，
 * 台架这里把 Kp 放宽到 100，是为了试出"到底多大 Kp 才能让小阶跃变快"。
 * 注意：Kp 超过 ~100 后，1 个编码器 count 的量化噪声会被放大成 100‰ 的占空比抖动，
 *       再往上加基本只是让电机嗡嗡叫，不会更快。
 * 台架上调出来的值，最后要回填 Service.c 里的 PID_VelocityLoop_Init()，
 * 并且把那边的 LIMIT_V_KP_MAX 一起放开。 */
#define BENCH_LIMIT_KP      100.0f
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
                pwm_left_out  = pwm_left;                   /* 记下来给 ch2 看 */
                pwm_right_out = pwm_right;                  /* 记下来给 ch5 看 */
            }
            else
            {
                Bench_Stop();                               /* 停车时每 5ms 清一次，防止积分堆积 */
                pwm_left_out  = 0.0f;
                pwm_right_out = 0.0f;
            }

            /* 两路占空比不走结构体，直接交给 Debug.c（见 Debug_SetPwm 的注释） */
            Debug_SetPwm(pwm_left_out, pwm_right_out);
        }

        /* ===== 10ms：按键扫描 ===== */
        /* ===== 10ms：读蓝牙指令 + 按键 + 发波形 =====
         * 波形原来是 100ms 一帧，但电机从 0 爬到目标只要几十毫秒，
         * 100ms 采一次等于完全看不到过程，只能看到结果。
         * 28 字节在 115200 下约 2.4ms，10ms 发一帧完全够用。 */
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

            /* 3.发波形（6 个通道全发，一帧 28 字节） */
            /* ch2 / ch5（两路占空比）不走结构体，由上面的 Debug_SetPwm() 直接给 Debug.c */
            dbg.target_left  = target_speed;    /* ch0 左轮目标 */
            dbg.actual_left  = actual_left;     /* ch1 左轮实际 */
            dbg.target_right = target_speed;    /* ch3 右轮目标 */
            dbg.actual_right = actual_right;    /* ch4 右轮实际 */
            dbg.error        = 0.0f;            /* 这个通道已经不发了，字段保留兼容正式固件 */

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
