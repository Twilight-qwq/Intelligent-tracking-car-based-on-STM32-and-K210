#include "stm32f10x.h"
#include "Gyro.h"

/* =========================================================================
 * 陀螺仪 ICM-20602（六轴，软件 I2C，SCL = PB0，SDA = PB1）
 *
 * 芯片确认：用户实际用的是 ICM-20602（不是 MPU6050）。
 *   和 MPU6050 的寄存器基本兼容（PWR_MGMT_1 / GYRO_CONFIG 地址一样），
 *   但有三处不同，见下面 GYRO_CHIP_NOTE_ 开头的注释。
 *
 * 命名约定：本文件里没有单字母变量名。
 *   寄存器地址一律以 GYRO_CHIP_REG_ 开头，芯片常量以 GYRO_CHIP_ 开头，
 *   本模块的状态变量以 Gyro_ 开头，参数一律写全拼。
 * ========================================================================= */

/* ==================== 引脚与地址 ==================== */

#define SOFT_I2C_PORT               GPIOB
#define SOFT_I2C_SCL_PIN            GPIO_Pin_0
#define SOFT_I2C_SDA_PIN            GPIO_Pin_1

/* SAO/AD0 接地时是 0x68（悬空也可能是 0x68，建议直接接地） */
#define GYRO_CHIP_I2C_ADDRESS       0x68

#define GYRO_CHIP_REG_SAMPLE_RATE   0x19
#define GYRO_CHIP_REG_LOW_PASS      0x1A
#define GYRO_CHIP_REG_GYRO_CONFIG   0x1B
#define GYRO_CHIP_REG_POWER_MGMT    0x6B
#define GYRO_CHIP_REG_WHO_AM_I      0x75

/* 陀螺仪三轴数据从 0x43 开始，连续 6 个字节：
 *   0x43/0x44 = X 轴高/低字节    0x45/0x46 = Y 轴    0x47/0x48 = Z 轴
 * 出处：本地手册 RM-MPU-6000A 寄存器表（_手册简化/text/RM-MPU-6000A.p1-50.dump.md
 *       第 568~596 行：43=GYRO_XOUT_H、45=GYRO_YOUT_H、47=GYRO_ZOUT_H）
 * 2026-11-21 修正：原来这里写的是 0x43，那是【X 轴】不是 Z 轴。
 *   表现就是"手转 90° 角度几乎不动"（用户实测只变了 3 度）——
 *   因为模块平放时 X 轴感受不到绕竖直轴的转动。 */
#define GYRO_CHIP_REG_AXIS_START    0x43

/* GYRO_CHIP_NOTE_1：WHO_AM_I 的返回值。
 *   ICM-20602 = 0x12（18）
 *   MPU6050   = 0x68（104）
 * 两个都认，认到哪个都能用。 */
#define GYRO_CHIP_ID_ICM20602       0x12
#define GYRO_CHIP_ID_MPU6050        0x68

/* GYRO_CHIP_NOTE_2：上电后要约 100 毫秒芯片才开始应答 I2C，
 *                  不等这一下，第一次读会全军覆没（表现为"模块没接上"）。 */
#define GYRO_CHIP_POWER_ON_WAIT_MS  100
#define GYRO_CHIP_DETECT_RETRY      5
#define GYRO_CHIP_DETECT_RETRY_MS   20

/* GYRO_CHIP_NOTE_3：唤醒写 0x01 而不是 0x00。
 *   0x00 = 唤醒，但时钟用内部 RC（会漂）
 *   0x01 = 唤醒 + 时钟切到陀螺仪 PLL（ICM-20602 推荐这个）
 * 如果读出来角速度一直是 0，把它改回 0x00 再试。 */
#define GYRO_CHIP_POWER_ON_VALUE    0x01
#define GYRO_CHIP_SAMPLE_RATE_DIV   0x04    /* 1kHz/(1+4) = 200Hz，和 5 毫秒对齐 */
#define GYRO_CHIP_LOW_PASS_VALUE    0x03    /* 低通 44Hz */

/* 量程 ±500 度/秒这一档：1 度/秒 = 65.5 个数字。
 * 为什么不用 ±250：上车转 90° 平均 240 度/秒，±250 只剩 4% 余量，
 *   加速段会削顶，读数被削平后陀螺仪会以为只转了 70 度。
 * 出处：本地手册 RM-MPU-6000A 第 2413~2421 行的 FS_SEL 表
 *       （FS_SEL=1 → ±500 度/秒 → 65.5 LSB/(度/秒)）；
 *       FS_SEL 是 GYRO_CONFIG(0x1B) 的 bit4~3，写 0x08 就是 FS_SEL=1。 */
#define GYRO_CHIP_RANGE_500_VALUE           0x08
#define GYRO_COUNTS_PER_DEGREE_PER_SECOND   65.5f

/* 调用周期：5 毫秒 = 0.005 秒 */
#define GYRO_SAMPLE_PERIOD_SECONDS          0.005f

/* 死区：转速绝对值小于这个数，就当"没在转"，不往角度里加。
 * 为什么需要：零偏切得不干净时，静止也会以 0.476 度/秒慢慢往上爬
 *   （用户实测：21 秒加了 10 度，10/21 = 0.476 度/秒）。
 * 为什么定 1.0：转弯真转速是 240 度/秒，差 240 倍，绝不可能把真转弯判成没转；
 *   代价只是转弯刚开始、转速还没升上来那一下不计，实测丢不到 0.02 度。 */
#define GYRO_RATE_DEADBAND_DPS              1.0f

/* 上电后要扔掉的开头：芯片刚醒的头几十毫秒读数是乱的，
 * 混进零偏会把零偏整体拉偏（这就是那 0.476 度/秒的来源）。
 * 先等 50 毫秒，再丢掉 40 个样本（40 × 5 毫秒 = 200 毫秒），之后才开始采零偏。 */
#define GYRO_CHIP_STARTUP_WAIT_MS           50
#define GYRO_BIAS_SKIP_SAMPLES              40

/* 200 × 5 毫秒 = 1 秒，要求这 1 秒车是静止的 */
#define GYRO_ZERO_BIAS_SAMPLE_TARGET        200

/* ==================== 本模块状态 ==================== */

static uint8_t  Gyro_WhoAmIValue           = 0;      /* 0x75 读回来的身份号 */
static uint8_t  Gyro_IsAvailable           = 0;      /* 1 = 芯片活着 */
static int16_t  Gyro_AxisXRaw              = 0;      /* X 轴原始读数（台架诊断用） */
static int16_t  Gyro_AxisYRaw              = 0;      /* Y 轴原始读数（台架诊断用） */
static int16_t  Gyro_AxisZRaw              = 0;      /* Z 轴原始读数 = 竖直轴 */
static float    Gyro_ZeroBias              = 0.0f;   /* 静止零偏（Z 轴） */
static float    Gyro_TotalAngleDegrees     = 0.0f;   /* 累计角度（度） */
static float    Gyro_ZeroBiasSum           = 0.0f;
static uint16_t Gyro_ZeroBiasSampleCount   = 0;
static uint8_t  Gyro_ZeroBiasReady         = 0;
static uint8_t  Gyro_BiasSkipCount         = 0;      /* 还要丢掉多少个上电初期的样本 */
static uint16_t Gyro_ReadErrorCount        = 0;

/* ==================== 软件 I2C 的引脚操作 ==================== */

#define SOFT_I2C_SCL_HIGH()   GPIO_SetBits(SOFT_I2C_PORT,   SOFT_I2C_SCL_PIN)
#define SOFT_I2C_SCL_LOW()    GPIO_ResetBits(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN)
#define SOFT_I2C_SDA_HIGH()   GPIO_SetBits(SOFT_I2C_PORT,   SOFT_I2C_SDA_PIN)
#define SOFT_I2C_SDA_LOW()    GPIO_ResetBits(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)
#define SOFT_I2C_SDA_READ()   GPIO_ReadInputDataBit(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)

/* 位延时：72MHz 下大约 1 微秒，凑成 100kHz 左右 */
static void SoftI2C_WaitOneBit(void)
{
	volatile uint8_t delay_counter;

	for (delay_counter = 0; delay_counter < 25; delay_counter++)
	{
	}
}

static void SoftI2C_InitPins(void)
{
	GPIO_InitTypeDef gpio_init_struct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	gpio_init_struct.GPIO_Pin   = SOFT_I2C_SCL_PIN | SOFT_I2C_SDA_PIN;
	gpio_init_struct.GPIO_Mode  = GPIO_Mode_Out_OD;   /* 开漏：写 1 = 释放，靠模块上的上拉电阻拉高 */
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SOFT_I2C_PORT, &gpio_init_struct);

	SOFT_I2C_SCL_HIGH();
	SOFT_I2C_SDA_HIGH();
}

static void SoftI2C_Start(void)
{
	SOFT_I2C_SDA_HIGH();
	SOFT_I2C_SCL_HIGH();
	SoftI2C_WaitOneBit();

	SOFT_I2C_SDA_LOW();
	SoftI2C_WaitOneBit();

	SOFT_I2C_SCL_LOW();
	SoftI2C_WaitOneBit();
}

static void SoftI2C_Stop(void)
{
	SOFT_I2C_SDA_LOW();
	SoftI2C_WaitOneBit();

	SOFT_I2C_SCL_HIGH();
	SoftI2C_WaitOneBit();

	SOFT_I2C_SDA_HIGH();
	SoftI2C_WaitOneBit();
}

/* 返回 1 = 收到应答，0 = 没应答（器件没接或坏了） */
static uint8_t SoftI2C_WriteByte(uint8_t write_byte)
{
	uint8_t bit_index;
	uint8_t acknowledge;

	for (bit_index = 0; bit_index < 8; bit_index++)
	{
		if (write_byte & 0x80)
		{
			SOFT_I2C_SDA_HIGH();
		}
		else
		{
			SOFT_I2C_SDA_LOW();
		}
		write_byte <<= 1;

		SoftI2C_WaitOneBit();
		SOFT_I2C_SCL_HIGH();
		SoftI2C_WaitOneBit();
		SOFT_I2C_SCL_LOW();
		SoftI2C_WaitOneBit();
	}

	/* 释放 SDA，等从机拉低表示应答 */
	SOFT_I2C_SDA_HIGH();
	SoftI2C_WaitOneBit();

	SOFT_I2C_SCL_HIGH();
	SoftI2C_WaitOneBit();
	acknowledge = (SOFT_I2C_SDA_READ() == 0) ? 1 : 0;
	SOFT_I2C_SCL_LOW();
	SoftI2C_WaitOneBit();

	return acknowledge;
}

static uint8_t SoftI2C_ReadByte(uint8_t send_acknowledge)
{
	uint8_t bit_index;
	uint8_t received_byte = 0;

	SOFT_I2C_SDA_HIGH();

	for (bit_index = 0; bit_index < 8; bit_index++)
	{
		SoftI2C_WaitOneBit();
		SOFT_I2C_SCL_HIGH();
		SoftI2C_WaitOneBit();

		received_byte <<= 1;
		if (SOFT_I2C_SDA_READ())
		{
			received_byte |= 1;
		}

		SOFT_I2C_SCL_LOW();
		SoftI2C_WaitOneBit();
	}

	if (send_acknowledge)
	{
		SOFT_I2C_SDA_LOW();
	}
	else
	{
		SOFT_I2C_SDA_HIGH();
	}
	SoftI2C_WaitOneBit();

	SOFT_I2C_SCL_HIGH();
	SoftI2C_WaitOneBit();
	SOFT_I2C_SCL_LOW();
	SoftI2C_WaitOneBit();
	SOFT_I2C_SDA_HIGH();

	return received_byte;
}

/* ==================== 芯片寄存器读写 ==================== */

static uint8_t GyroChip_WriteRegister(uint8_t register_address, uint8_t register_value)
{
	SoftI2C_Start();
	if (!SoftI2C_WriteByte((uint8_t)(GYRO_CHIP_I2C_ADDRESS << 1)))
	{
		SoftI2C_Stop();
		return 0;
	}
	if (!SoftI2C_WriteByte(register_address))
	{
		SoftI2C_Stop();
		return 0;
	}
	if (!SoftI2C_WriteByte(register_value))
	{
		SoftI2C_Stop();
		return 0;
	}
	SoftI2C_Stop();

	return 1;
}

static uint8_t GyroChip_ReadRegisters(uint8_t start_register, uint8_t *read_buffer, uint8_t read_count)
{
	uint8_t byte_index;

	SoftI2C_Start();
	if (!SoftI2C_WriteByte((uint8_t)(GYRO_CHIP_I2C_ADDRESS << 1)))
	{
		SoftI2C_Stop();
		return 0;
	}
	if (!SoftI2C_WriteByte(start_register))
	{
		SoftI2C_Stop();
		return 0;
	}

	/* 重复起始 */
	SoftI2C_Start();
	if (!SoftI2C_WriteByte((uint8_t)((GYRO_CHIP_I2C_ADDRESS << 1) | 1)))
	{
		SoftI2C_Stop();
		return 0;
	}

	for (byte_index = 0; byte_index < read_count; byte_index++)
	{
		/* 最后一个字节回"不应答"告诉从机别再发了，前面都回"应答" */
		read_buffer[byte_index] = SoftI2C_ReadByte((uint8_t)(byte_index < (uint8_t)(read_count - 1)));
	}

	SoftI2C_Stop();

	return 1;
}

/* 简单毫秒延时：72MHz 下大约 12000 次空循环 = 1 毫秒 */
static void Gyro_WaitMilliseconds(uint16_t delay_milliseconds)
{
	volatile uint32_t loop_counter;
	uint32_t total_loops;

	total_loops = (uint32_t)delay_milliseconds * 12000u;

	for (loop_counter = 0; loop_counter < total_loops; loop_counter++)
	{
	}
}

/* ==================== 对外接口 ==================== */

/* 只做三件事：确认芯片活着、唤醒、配置量程。很快，不阻塞。 */
void Gyro_Init(void)
{
	uint8_t who_am_i_value = 0;
	uint8_t retry_times;

	Gyro_WaitMilliseconds(GYRO_CHIP_POWER_ON_WAIT_MS);
	SoftI2C_InitPins();

	for (retry_times = 0; retry_times < GYRO_CHIP_DETECT_RETRY; retry_times++)
	{
		if (GyroChip_ReadRegisters(GYRO_CHIP_REG_WHO_AM_I, &who_am_i_value, 1))
		{
			if ((who_am_i_value == GYRO_CHIP_ID_ICM20602) || (who_am_i_value == GYRO_CHIP_ID_MPU6050))
			{
				Gyro_IsAvailable = 1;
				break;
			}
		}

		/* 没认出来就等一会儿再试 */
		Gyro_WaitMilliseconds(GYRO_CHIP_DETECT_RETRY_MS);
	}

	Gyro_WhoAmIValue = who_am_i_value;

	if (!Gyro_IsAvailable)
	{
		return;                      /* 芯片不在，后面全部跳过，不影响跑车 */
	}

	GyroChip_WriteRegister(GYRO_CHIP_REG_POWER_MGMT,  GYRO_CHIP_POWER_ON_VALUE);
	Gyro_WaitMilliseconds(10);
	GyroChip_WriteRegister(GYRO_CHIP_REG_SAMPLE_RATE, GYRO_CHIP_SAMPLE_RATE_DIV);
	GyroChip_WriteRegister(GYRO_CHIP_REG_LOW_PASS,    GYRO_CHIP_LOW_PASS_VALUE);
	GyroChip_WriteRegister(GYRO_CHIP_REG_GYRO_CONFIG, GYRO_CHIP_RANGE_500_VALUE);

	/* 配置完再等一会儿，让芯片彻底稳下来，才开始采零偏 */
	Gyro_WaitMilliseconds(GYRO_CHIP_STARTUP_WAIT_MS);

	Gyro_BiasSkipCount = GYRO_BIAS_SKIP_SAMPLES;
}

/* 每 5 毫秒调一次：前 1 秒测零偏，之后按 Z 轴积分
 * 一次读 6 个字节把 X/Y/Z 全拿回来：模块要是侧着装，绕竖直轴的转动
 * 就落在 X 或 Y 上，只读 Z 会永远是 0（2026-11-21 踩过这个坑）。 */
void Gyro_Update(void)
{
	uint8_t axis_raw_bytes[6];
	int16_t axis_x_raw;
	int16_t axis_y_raw;
	int16_t axis_z_raw;
	float   rate_degrees_per_second;

	if (!Gyro_IsAvailable)
	{
		return;
	}

	if (!GyroChip_ReadRegisters(GYRO_CHIP_REG_AXIS_START, axis_raw_bytes, 6))
	{
		if (Gyro_ReadErrorCount < 60000)
		{
			Gyro_ReadErrorCount++;
		}
		return;
	}

	axis_x_raw = (int16_t)(((uint16_t)axis_raw_bytes[0] << 8) | axis_raw_bytes[1]);  /* 0x43/0x44 = X 轴 */
	axis_y_raw = (int16_t)(((uint16_t)axis_raw_bytes[2] << 8) | axis_raw_bytes[3]);  /* 0x45/0x46 = Y 轴 */
	axis_z_raw = (int16_t)(((uint16_t)axis_raw_bytes[4] << 8) | axis_raw_bytes[5]);  /* 0x47/0x48 = Z 轴 */

	Gyro_AxisXRaw = axis_x_raw;
	Gyro_AxisYRaw = axis_y_raw;
	Gyro_AxisZRaw = axis_z_raw;

	if (!Gyro_ZeroBiasReady)
	{
		/* 上电初期这几个样本先扔掉，不参与零偏 */
		if (Gyro_BiasSkipCount > 0)
		{
			Gyro_BiasSkipCount--;
			return;
		}

		Gyro_ZeroBiasSum += (float)axis_z_raw;
		Gyro_ZeroBiasSampleCount++;

		if (Gyro_ZeroBiasSampleCount >= GYRO_ZERO_BIAS_SAMPLE_TARGET)
		{
			Gyro_ZeroBias      = Gyro_ZeroBiasSum / (float)Gyro_ZeroBiasSampleCount;
			Gyro_ZeroBiasReady = 1;
		}

		return;
	}

	rate_degrees_per_second = ((float)axis_z_raw - Gyro_ZeroBias)
	                          / GYRO_COUNTS_PER_DEGREE_PER_SECOND;

	/* 死区：小于 1 度/秒当作没转，这样静止时角度不会慢慢往上爬 */
	if ((rate_degrees_per_second > GYRO_RATE_DEADBAND_DPS)
	    || (rate_degrees_per_second < -GYRO_RATE_DEADBAND_DPS))
	{
		Gyro_TotalAngleDegrees += rate_degrees_per_second * GYRO_SAMPLE_PERIOD_SECONDS;
	}
}

uint8_t Gyro_IsOk(void)
{
	return Gyro_IsAvailable;
}

uint8_t Gyro_GetWhoAmIValue(void)
{
	return Gyro_WhoAmIValue;
}

int16_t Gyro_GetAxisXRaw(void)
{
	return Gyro_AxisXRaw;
}

int16_t Gyro_GetAxisYRaw(void)
{
	return Gyro_AxisYRaw;
}

int16_t Gyro_GetAxisZRaw(void)
{
	return Gyro_AxisZRaw;
}

float Gyro_GetZeroBias(void)
{
	return Gyro_ZeroBias;
}

float Gyro_GetTotalAngle(void)
{
	return Gyro_TotalAngleDegrees;
}

void Gyro_ResetTotalAngle(void)
{
	Gyro_TotalAngleDegrees = 0.0f;
}

uint16_t Gyro_GetReadErrorCount(void)
{
	return Gyro_ReadErrorCount;
}
