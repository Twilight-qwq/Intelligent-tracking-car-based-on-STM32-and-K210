#include "stm32f10x.h"
#include "Gyro.h"

/* =========================================================================
 * 陀螺仪 ICM-20602（六轴，软件 I2C，PB0 = SCL，PB1 = SDA）
 *
 * 芯片确认：用户实际用的是 ICM-20602（不是 MPU6050）。
 *   和 MPU6050 的寄存器基本兼容（PWR_MGMT_1 / GYRO_CONFIG 地址一样），
 *   但有三处不同，见下面的 ICM_ 开头的注释。
 * ========================================================================= */

#define SI2C_PORT         GPIOB
#define SI2C_SCL_PIN      GPIO_Pin_0
#define SI2C_SDA_PIN      GPIO_Pin_1

#define MPU_ADDR          0x68     /* SAO/AD0 接地时是 0x68（悬空也可能是 0x68，建议直接接地） */
#define MPU_REG_SMPLRT    0x19
#define MPU_REG_CONFIG    0x1A
#define MPU_REG_GYROCFG   0x1B

/* 陀螺仪三轴数据从 0x43 开始，连续 6 个字节：
 *   0x43/0x44 = X 轴高/低字节    0x45/0x46 = Y 轴    0x47/0x48 = Z 轴
 * 出处：本地手册 RM-MPU-6000A 寄存器表（_手册简化/text/RM-MPU-6000A.p1-50.dump.md
 *       第 568~596 行：43=GYRO_XOUT_H、45=GYRO_YOUT_H、47=GYRO_ZOUT_H）
 * 2026-11-21 修正：原来这里写的是 0x43，那是【X 轴】不是 Z 轴。
 *   表现就是"手转 90° 角度几乎不动"（用户实测只变了 3 度）——
 *   因为模块平放时 X 轴感受不到绕竖直轴的转动。 */
#define MPU_REG_GYRO_XYZ  0x43     /* 三轴起始地址：一次读 6 字节拿全 X/Y/Z */
#define MPU_REG_PWRMGMT   0x6B
#define MPU_REG_WHOAMI    0x75

/* ICM_ 第 1 处不同：WHO_AM_I 的返回值。
 *   ICM-20602 = 0x12（18）
 *   MPU6050   = 0x68（104）
 * 两个都认，认到哪个都能用。 */
#define GYRO_ID_ICM20602  0x12
#define GYRO_ID_MPU6050   0x68

#define GYRO_LSB_PER_DPS  131.0f   /* ±250°/s 档的分辨率，两者相同 */
#define GYRO_DT_SEC       0.005f   /* 调用周期：5 毫秒 = 0.005 秒 */
#define GYRO_BIAS_SAMPLES 200      /* 200 × 5 毫秒 = 1 秒，要求这 1 秒车是静止的 */

static uint8_t  Gyro_WhoAmI   = 0;     /* 读回来的 0x75：ICM-20602 = 18，MPU6050 = 104 */
static uint8_t  Gyro_Ok       = 0;     /* 1 = 模块活着 */
static int16_t  Gyro_Xraw     = 0;     /* X 轴原始读数（台架诊断用） */
static int16_t  Gyro_Yraw     = 0;     /* Y 轴原始读数（台架诊断用） */
static int16_t  Gyro_Zraw     = 0;     /* Z 轴原始读数 = 竖直轴 */
static float    Gyro_Bias     = 0.0f;  /* 静止零偏（Z 轴） */
static float    Gyro_Angle    = 0.0f;  /* 累计角度（度） */
static float    Gyro_BiasSum  = 0.0f;
static uint16_t Gyro_BiasN    = 0;
static uint8_t  Gyro_BiasDone = 0;
static uint16_t Gyro_ErrCnt   = 0;

#define SCL_H()    GPIO_SetBits(SI2C_PORT,   SI2C_SCL_PIN)
#define SCL_L()    GPIO_ResetBits(SI2C_PORT, SI2C_SCL_PIN)
#define SDA_H()    GPIO_SetBits(SI2C_PORT,   SI2C_SDA_PIN)
#define SDA_L()    GPIO_ResetBits(SI2C_PORT, SI2C_SDA_PIN)
#define SDA_READ() GPIO_ReadInputDataBit(SI2C_PORT, SI2C_SDA_PIN)

/* 位延时：72MHz 下大约 1 微秒，凑成 100kHz 左右 */
static void I2C_Delay(void)
{
	volatile uint8_t i;
	for(i = 0; i < 25; i++) { }
}

static void I2C_InitPins(void)
{
	GPIO_InitTypeDef g;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	g.GPIO_Pin   = SI2C_SCL_PIN | SI2C_SDA_PIN;
	g.GPIO_Mode  = GPIO_Mode_Out_OD;   /* 开漏：写 1 = 释放，靠模块上的上拉电阻拉高 */
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SI2C_PORT, &g);
	SCL_H();
	SDA_H();
}

static void I2C_Start(void)
{
	SDA_H(); SCL_H(); I2C_Delay();
	SDA_L(); I2C_Delay();
	SCL_L(); I2C_Delay();
}

static void I2C_Stop(void)
{
	SDA_L(); I2C_Delay();
	SCL_H(); I2C_Delay();
	SDA_H(); I2C_Delay();
}

/* 返回 1 = 收到应答，0 = 没应答（器件没接或坏了） */
static uint8_t I2C_WriteByte(uint8_t b)
{
	uint8_t i, ack;
	for(i = 0; i < 8; i++)
	{
		if(b & 0x80) { SDA_H(); } else { SDA_L(); }
		b <<= 1;
		I2C_Delay();
		SCL_H(); I2C_Delay();
		SCL_L(); I2C_Delay();
	}
	SDA_H();                 /* 释放 SDA，等从机拉低表示应答 */
	I2C_Delay();
	SCL_H(); I2C_Delay();
	ack = (SDA_READ() == 0) ? 1 : 0;
	SCL_L(); I2C_Delay();
	return ack;
}

static uint8_t I2C_ReadByte(uint8_t give_ack)
{
	uint8_t i, b = 0;
	SDA_H();
	for(i = 0; i < 8; i++)
	{
		I2C_Delay();
		SCL_H(); I2C_Delay();
		b <<= 1;
		if(SDA_READ()) { b |= 1; }
		SCL_L(); I2C_Delay();
	}
	if(give_ack) { SDA_L(); } else { SDA_H(); }
	I2C_Delay();
	SCL_H(); I2C_Delay();
	SCL_L(); I2C_Delay();
	SDA_H();
	return b;
}

static uint8_t MPU_WriteReg(uint8_t reg, uint8_t val)
{
	I2C_Start();
	if(!I2C_WriteByte((uint8_t)(MPU_ADDR << 1))) { I2C_Stop(); return 0; }
	if(!I2C_WriteByte(reg))                      { I2C_Stop(); return 0; }
	if(!I2C_WriteByte(val))                      { I2C_Stop(); return 0; }
	I2C_Stop();
	return 1;
}

static uint8_t MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t n)
{
	uint8_t i;
	I2C_Start();
	if(!I2C_WriteByte((uint8_t)(MPU_ADDR << 1)))       { I2C_Stop(); return 0; }
	if(!I2C_WriteByte(reg))                             { I2C_Stop(); return 0; }
	I2C_Start();                                        /* 重复起始 */
	if(!I2C_WriteByte((uint8_t)((MPU_ADDR << 1) | 1))) { I2C_Stop(); return 0; }
	for(i = 0; i < n; i++)
	{
		buf[i] = I2C_ReadByte((uint8_t)(i < (uint8_t)(n - 1)));
	}
	I2C_Stop();
	return 1;
}

/* 简单毫秒延时：72MHz 下大约 12000 次空循环 = 1 毫秒 */
static void Gyro_DelayMs(uint16_t ms)
{
	volatile uint32_t i;
	uint32_t n = (uint32_t)ms * 12000u;
	for(i = 0; i < n; i++) { }
}

/* 只做三件事：确认模块活着、唤醒、配置量程。很快，不阻塞。 */
void Gyro_Init(void)
{
	uint8_t b = 0, try;

	/* ICM_ 第 2 处不同：ICM-20602 上电后要约 100 毫秒才开始应答 I2C，
	 * 不等这一下，第一次读会全军覆没（表现为"模块没接上"）。 */
	Gyro_DelayMs(100);

	I2C_InitPins();
	for(try = 0; try < 5; try++)
	{
		if(MPU_ReadRegs(MPU_REG_WHOAMI, &b, 1))
		{
			if((b == GYRO_ID_ICM20602) || (b == GYRO_ID_MPU6050))
			{
				Gyro_Ok = 1;
				break;
			}
		}
		Gyro_DelayMs(20);          /* 没认出来就等 20 毫秒再试，最多 5 次 */
	}
	Gyro_WhoAmI = b;
	if(!Gyro_Ok) { return; }       /* 模块不在，后面全部跳过，不影响跑车 */

	/* ICM_ 第 3 处不同：写 0x01 而不是 0x00。
	 *   0x00 = 唤醒，但时钟用内部 RC（会漂）
	 *   0x01 = 唤醒 + 时钟切到陀螺仪 PLL（ICM-20602 推荐这个）
	 * 如果读出来角速度一直是 0，把它改回 0x00 再试。 */
	MPU_WriteReg(MPU_REG_PWRMGMT, 0x01);
	Gyro_DelayMs(10);
	MPU_WriteReg(MPU_REG_SMPLRT,  0x04);/* 采样率 1kHz/(1+4) = 200Hz，和 5 毫秒对齐 */
	MPU_WriteReg(MPU_REG_CONFIG,  0x03);/* 低通 44Hz */
	MPU_WriteReg(MPU_REG_GYROCFG, 0x00);/* ±250°/s */
}

/* 每 5 毫秒调一次：前 1 秒测零偏，之后按 Z 轴积分
 * 一次读 6 个字节把 X/Y/Z 全拿回来：模块要是侧着装，绕竖直轴的转动
 * 就落在 X 或 Y 上，只读 Z 会永远是 0（2026-11-21 踩过这个坑）。 */
void Gyro_Update(void)
{
	uint8_t buf[6];
	int16_t x, y, z;

	if(!Gyro_Ok) { return; }

	if(!MPU_ReadRegs(MPU_REG_GYRO_XYZ, buf, 6))
	{
		if(Gyro_ErrCnt < 60000) { Gyro_ErrCnt++; }
		return;
	}
	x = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);   /* 0x43/0x44 = X 轴 */
	y = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);   /* 0x45/0x46 = Y 轴 */
	z = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);   /* 0x47/0x48 = Z 轴 */
	Gyro_Xraw = x;
	Gyro_Yraw = y;
	Gyro_Zraw = z;

	if(!Gyro_BiasDone)
	{
		Gyro_BiasSum += (float)z;
		Gyro_BiasN++;
		if(Gyro_BiasN >= GYRO_BIAS_SAMPLES)
		{
			Gyro_Bias     = Gyro_BiasSum / (float)Gyro_BiasN;
			Gyro_BiasDone = 1;
		}
		return;
	}

	Gyro_Angle += ((float)z - Gyro_Bias) / GYRO_LSB_PER_DPS * GYRO_DT_SEC;
}

/* ==================== 给上层读的接口 ==================== */

uint8_t Gyro_IsOk(void)
{
	return Gyro_Ok;
}

uint8_t Gyro_GetWhoAmI(void)
{
	return Gyro_WhoAmI;
}

int16_t Gyro_GetXraw(void)
{
	return Gyro_Xraw;
}

int16_t Gyro_GetYraw(void)
{
	return Gyro_Yraw;
}

int16_t Gyro_GetZraw(void)
{
	return Gyro_Zraw;
}

float Gyro_GetBias(void)
{
	return Gyro_Bias;
}

float Gyro_GetAngle(void)
{
	return Gyro_Angle;
}

void Gyro_ResetAngle(void)
{
	Gyro_Angle = 0.0f;
}

uint16_t Gyro_GetErrCnt(void)
{
	return Gyro_ErrCnt;
}
