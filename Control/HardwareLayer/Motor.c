#include "stm32f10x.h"
#include "Motor.h"
/* ==================== 引脚宏定义 ==================== */
// 时钟
#define MOTOR_GPIO_CLK          (RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB)
#define MOTOR_TIM_CLK           RCC_APB2Periph_TIM1
#define MOTOR_TIM               TIM1

// PWM参数：72MHz / (PSC+1) / (ARR+1) = 72M/36/100 = 20kHz
#define MOTOR_PWM_PSC           35
#define MOTOR_PWM_ARR           99

// 占空比指令满量程（‰），与PID速度环输出量纲一致
#define MOTOR_DUTY_FULLSCALE    1000

// 左轮PWM->TB6612PWMA
#define MOTOR_LEFT_PWM_PORT     GPIOA
#define MOTOR_LEFT_PWM_PIN      GPIO_Pin_8    // TIM1_CH1

// 右轮PWM->TB6612PWMB
#define MOTOR_RIGHT_PWM_PORT    GPIOA
#define MOTOR_RIGHT_PWM_PIN     GPIO_Pin_11   // TIM1_CH4

// 左轮方向,都拉低静止,都拉高制动
#define MOTOR_LEFT_DIR_PORT     GPIOB
#define MOTOR_LEFT_AIN1_PIN     GPIO_Pin_12
#define MOTOR_LEFT_AIN2_PIN     GPIO_Pin_13

// 右轮方向,都拉低静止,都拉高制动
#define MOTOR_RIGHT_DIR_PORT    GPIOB
#define MOTOR_RIGHT_BIN1_PIN    GPIO_Pin_14
#define MOTOR_RIGHT_BIN2_PIN    GPIO_Pin_15

// STBY使能引脚,上电拉低,严重丢线拉低
#define MOTOR_STBY_PORT         GPIOB
#define MOTOR_STBY_PIN          GPIO_Pin_5

/* ==================== 初始化 ==================== */
/* tb6612接线说明：
 *   左轮：AO1->红线, AO2->黑线, AIN1->PB12, AIN2->PB13, PWMA->PA8
 *   右轮：BO1->红线, BO2->黑线, BIN1->PB14, BIN2->PB15, PWMB->PA11
 *   电源：VM->电池12.6V(3S直通), VCC->3.3V, GND->共地, STBY->PB5
 *        //FIX 2：VCC必须接3.3V，接5V会导致输入高电平门限不匹配
 * PWM频率 20kHz
 */
void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(MOTOR_GPIO_CLK,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin=MOTOR_LEFT_PWM_PIN|MOTOR_RIGHT_PWM_PIN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_LEFT_PWM_PORT,&GPIO_InitStruct);//开启左右轮pwm对应gpio


	/* 先在ODR写死电平再配成输出，避免切换瞬间出毛刺 */
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin=MOTOR_LEFT_AIN1_PIN|MOTOR_LEFT_AIN2_PIN|
	                         MOTOR_RIGHT_BIN1_PIN|MOTOR_RIGHT_BIN2_PIN;
	GPIO_SetBits(MOTOR_LEFT_DIR_PORT,MOTOR_LEFT_AIN2_PIN|MOTOR_RIGHT_BIN2_PIN);//AIN2=1,BIN2=1，默认正转
	GPIO_ResetBits(MOTOR_LEFT_DIR_PORT,MOTOR_LEFT_AIN1_PIN|MOTOR_RIGHT_BIN1_PIN);//AIN1=0,BIN1=0
	GPIO_Init(MOTOR_LEFT_DIR_PORT,&GPIO_InitStruct);

	
	/* STBY先拉低，待机，防止初始化期间电机抽动 */
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin=MOTOR_STBY_PIN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_ResetBits(MOTOR_STBY_PORT,MOTOR_STBY_PIN);
	GPIO_Init(MOTOR_STBY_PORT,&GPIO_InitStruct);


	RCC_APB2PeriphClockCmd(MOTOR_TIM_CLK,ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=MOTOR_PWM_ARR;//FIX 3：用宏，避免和换算常数脱节
	TIM_TimeBaseInitStruct.TIM_Prescaler=MOTOR_PWM_PSC;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(MOTOR_TIM,&TIM_TimeBaseInitStruct);


	/* TIM1是高级定时器，N通道默认映射在PB13~PB15，必须显式清零
	   防止互补输出抢走方向引脚 */
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse=0;
	TIM_OCInitStruct.TIM_OutputNState=TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState=TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	TIM_OC1Init(MOTOR_TIM,&TIM_OCInitStruct);
	TIM_OC1PreloadConfig(MOTOR_TIM,TIM_OCPreload_Enable);
	TIM_OC4Init(MOTOR_TIM,&TIM_OCInitStruct);
	TIM_OC4PreloadConfig(MOTOR_TIM,TIM_OCPreload_Enable);


	TIM_Cmd(MOTOR_TIM,ENABLE);
	TIM_CtrlPWMOutputs(MOTOR_TIM,ENABLE);


	/* 所有寄存器配置完毕，最后使能TB6612 */
	GPIO_SetBits(MOTOR_STBY_PORT,MOTOR_STBY_PIN);
}

/* ==================== 方向控制 ==================== */
//控制ain1,ain2来控制旋转方向
static void MotorLeft_SetDirection(uint8_t isForward)
{
	if(isForward == 1)
	{
		GPIO_ResetBits(MOTOR_LEFT_DIR_PORT,MOTOR_LEFT_AIN1_PIN);//拉低AIN1
		GPIO_SetBits(MOTOR_LEFT_DIR_PORT,MOTOR_LEFT_AIN2_PIN);//拉高AIN2
	}
	else if(isForward==0)
	{
		GPIO_SetBits(MOTOR_LEFT_DIR_PORT,MOTOR_LEFT_AIN1_PIN);//拉高AIN1
		GPIO_ResetBits(MOTOR_LEFT_DIR_PORT,MOTOR_LEFT_AIN2_PIN);//拉低AIN2
	}
}

/* 右轮方向控制
 * isForward: 1=正转，0=反转
 * PB14->BIN1, PB15->BIN2
 */
static void MotorRight_SetDirection(uint8_t isForward)
{
	if(isForward == 1)
	{
		GPIO_ResetBits(MOTOR_RIGHT_DIR_PORT,MOTOR_RIGHT_BIN1_PIN);//拉低BIN1
		GPIO_SetBits(MOTOR_RIGHT_DIR_PORT,MOTOR_RIGHT_BIN2_PIN);//拉高BIN2
	}
	else if(isForward==0)
	{
		GPIO_SetBits(MOTOR_RIGHT_DIR_PORT,MOTOR_RIGHT_BIN1_PIN);//拉高BIN1
		GPIO_ResetBits(MOTOR_RIGHT_DIR_PORT,MOTOR_RIGHT_BIN2_PIN);//拉低BIN2
	}
}


/* ==================== 占空比输出 ==================== */
/* 功能：占空比指令（‰）换算成CCR比较值
 * 参数：duty绝对值，输入范围0-1000，把按百分制计算的占空比取数字后*10
 * 返回：CCR比较值，范围0-ARR
 */
static uint16_t Motor_DutyToCCR(int16_t duty)
{
	if(duty > MOTOR_DUTY_FULLSCALE)
	{
		duty = MOTOR_DUTY_FULLSCALE;
	}
	return (uint16_t)((int32_t)duty * MOTOR_PWM_ARR / MOTOR_DUTY_FULLSCALE);
}


/* 功能：设置左右轮占空比指令
 * 参数1：左轮指令（‰），范围-1000~1000，正=正转，负=反转，0=停
 * 参数2：右轮指令（‰），范围同上
 * 参数来源：encoder获取counter增量作为速度参照然后丢个pid速度环计算
 * 返回：无
 */


void Motor_SetDuty(int16_t left_duty , int16_t right_duty)
{
	//1.左轮
	if(left_duty >= 0)
	{
		MotorLeft_SetDirection(1);
		TIM_SetCompare1(MOTOR_TIM,Motor_DutyToCCR(left_duty));
	}
	else
	{
		MotorLeft_SetDirection(0);
		TIM_SetCompare1(MOTOR_TIM,Motor_DutyToCCR(-left_duty));
	}

	//2.右轮
	if(right_duty >= 0)
	{
		MotorRight_SetDirection(1);
		TIM_SetCompare4(MOTOR_TIM,Motor_DutyToCCR(right_duty));
	}
	else
	{
		MotorRight_SetDirection(0);
		TIM_SetCompare4(MOTOR_TIM,Motor_DutyToCCR(-right_duty));
	}
}
