#include "stm32f10x.h"
#include "mid_delay.h"
#include "bsp_gyro.h"

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
#define GYRO_CHIP_REG_AXIS_Z        0x47    /* GYRO_ZOUT_H，Z 轴高字节 */

/* 读几个轴（2026-11-21 加）
 * 0 = 只读 Z 轴 2 个字节（默认）：Z 是竖直轴，跑车只用它。
 *     一次读约 48 位，比读三轴少 43% 的时间（0.35 毫秒 → 0.20 毫秒）。
 * 1 = 读 X/Y/Z 三轴 6 个字节：只在需要重新确认「哪根是竖直轴」时用
 *     （比如模块重新装过、换了一个模块）。这时台架模式的 I1/I2 才有数。
 * 改完记着：读 0 的时候台架 I1、I2 会恒为 0，这是正常的，不是坏。 */
#define GYRO_READ_ALL_AXES          0

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

/* 调用周期：5 毫秒 = 0.005 秒。只在第一次调用时当默认值用，
 * 之后一律用「这次和上次隔了多少毫秒」的真实间隔。 */
#define GYRO_SAMPLE_PERIOD_SECONDS          0.005f

/* 真实间隔的上限：超过这个就当卡死过，按 50 毫秒算，别把一大段时间乘进角度里 */
#define GYRO_DT_MAX_MS                      50u

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
/* 零偏采样期间，最大值和最小值差超过这么多，就认为车被人动过，这一批作废重采。
 * 为什么需要：零偏要求车静止，而 Gyro_Update() 在停车状态也照跑。
 *   上电后 1.2 秒内摆车、对线，那些真实转动会被平均进零偏；
 *   零偏一偏，之后每帧都减错 —— 偏大就转过量（最坏转到 800ms 超时 ≈190 度）。
 * 300 个数字 ≈ 4.6 度/秒，静止时的噪声只有几个数字，手转一下是几千，很好区分。 */
#define GYRO_BIAS_MAX_SPREAD                300
/* 最多重采几次。不加这个上限的话，人一直晃车就永远采不完零偏。 */
#define GYRO_BIAS_MAX_RESTART               3

/* ==================== 本模块状态 ==================== */

static uint8_t  Gyro_WhoAmIValue           = 0;      /* 0x75 读回来的身份号 */
static uint8_t  Gyro_IsAvailable           = 0;      /* 1 = 芯片活着 */
static int16_t  Gyro_AxisXRaw              = 0;      /* X 轴原始读数（台架诊断用） */
static int16_t  Gyro_AxisYRaw              = 0;      /* Y 轴原始读数（台架诊断用） */
static int16_t  Gyro_AxisZRaw              = 0;      /* Z 轴原始读数 = 竖直轴 */
static float    Gyro_ZeroBias              = 0.0f;   /* 静止零偏（Z 轴原始读数的平均值）。
                                                       * 车静止时陀螺仪读数不是 0，而是一个固定的小偏置；
                                                       * 每次算转速都要先减掉它，否则静止也会慢慢积分出角度。 */
static float    Gyro_TotalAngleDegrees     = 0.0f;   /* 累计角度（度）：从上次清零到现在一共转了多少度。
                                                       * 右转为负、左转为正（芯片的符号约定，用的时候取绝对值）。
                                                       * 转弯「转够了没有」就是靠它判断。 */
static float    Gyro_ZeroBiasSum           = 0.0f;   /* 测零偏期间的累加和：先把 200 个样本的值加起来，
                                                       * 采够之后除以样本个数，才是平均值 Gyro_ZeroBias。 */
static uint16_t Gyro_ZeroBiasSampleCount   = 0;      /* 零偏已经采了几个样本，到 200 就停。
                                                       * 200 × 5 毫秒 = 1 秒，要求这 1 秒里车是静止的。 */
static uint8_t  Gyro_ZeroBiasReady         = 0;      /* 1 = 零偏已经测完，可以正常积分角度了；
                                                       * 0 = 还在测零偏，这期间 Gyro_TotalAngleDegrees 一直不动。 */
static uint8_t  Gyro_BiasSkipCount         = 0;      /* 还要丢掉多少个上电初期的样本。
                                                       * 芯片刚醒的头几十毫秒读数是乱的，混进零偏会把平均值带偏。
                                                       * 从 40 开始往下倒数，减到 0 才开始真正采零偏。 */
static uint16_t Gyro_ReadErrorCount        = 0;      /* I2C 读失败的总次数，只增不减，封顶 60000 防止溢出。
                                                       * 纯诊断用：台架上看它一直涨，说明接线或模块有问题。
                                                       * 跑车逻辑不依赖它。 */
static uint32_t Gyro_LastUpdateMs          = 0;      /* 上次调用 Gyro_Update 的时刻 */
static uint8_t  Gyro_DtReady               = 0;      /* 1 = 已经记过时刻，可以算真实间隔了 */
static uint16_t Gyro_ConsecutiveFailCount  = 0;      /* 连续读失败次数，成功一次就清零 */
static int16_t  Gyro_BiasSampleMin         = 0;      /* 采零偏期间，这一批样本的最小值 */
static int16_t  Gyro_BiasSampleMax         = 0;      /* 采零偏期间，这一批样本的最大值 */
static uint8_t  Gyro_BiasRestartCount      = 0;      /* 已经因为「车在动」重采了几次 */

/* 连续失败这么多次（3 × 5 毫秒 = 15 毫秒）就试着救一次总线。
 * 原来是 10 次（50 毫秒）：这 50 毫秒里每次读失败都直接 return，角度完全不积分，
 * 转弯中撞上一次就是 240 度/秒 × 0.05 秒 ≈ 12 度没了。
 * 降到 3 次只损失约 3.6 度。救总线本身很便宜：SDA 是高电平（正常总线）就直接返回。 */
#define GYRO_RECOVERY_AFTER_FAILS           3

/* ==================== 软件 I2C 的引脚操作 ==================== */

#define SOFT_I2C_SCL_HIGH()   GPIO_SetBits(SOFT_I2C_PORT,   SOFT_I2C_SCL_PIN)
#define SOFT_I2C_SCL_LOW()    GPIO_ResetBits(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN)
#define SOFT_I2C_SDA_HIGH()   GPIO_SetBits(SOFT_I2C_PORT,   SOFT_I2C_SDA_PIN)
#define SOFT_I2C_SDA_LOW()    GPIO_ResetBits(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)
#define SOFT_I2C_SDA_READ()   GPIO_ReadInputDataBit(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)

/* 位延时：72MHz 下大约 1 微秒，凑成 100kHz 左右 */
static void SoftI2C_WaitOneBit(void)
{
	/* delay_counter：空转计数用的临时变量。
	 * 为什么必须加 volatile：不加的话编译器一看"这个循环什么也没干"，
	 *   会把它整个优化掉，延时直接变成 0，I2C 时序就乱了。 */
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
	uint8_t bit_index;      /* 一个字节 8 位，从最高位开始一位一位发，这是第几位（0~7） */
	uint8_t acknowledge;    /* 发完 8 位后从机回的那 1 位：SDA 被拉低 = 应答 = 成功。
	                         * 本函数把「收到应答」记成 1、「没应答」记成 0，
	                         * 这样上层写 if(!SoftI2C_WriteByte(x)) 就是在问"失败了吗"。 */

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
	uint8_t bit_index;              /* 读第几位（0~7），一共读 8 位拼成一个字节 */
	uint8_t received_byte = 0;      /* 拼出来的那个字节：每读一位先整体左移，再把新的一位放到最低位 */

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

/* ==================== 总线恢复（2026-11-21 新增） ====================
 * 干什么：如果从机把 SDA 一直拉低（上电时序乱了、突然掉电、被静电打懵），
 *         主机会永远收到"不应答"，角度再也不涨。
 * 标准救法：主机自己打最多 9 个时钟，让从机把没发完的字节发完、松开 SDA，
 *         再补一个停止条件，总线就自由了。
 * 什么时候调：① 上电初始化时先来一次；② 连续读失败 10 次（50 毫秒）时救一次。
 * 为什么不会干扰正常通信：正常总线上 SDA 是高的，函数第一句就 return，什么都不做。
 * 代价：每次读失败多一次比较；只有真失败时才打那 9 个时钟（约 0.2 毫秒）。 */
static void SoftI2C_BusRecovery(void)
{
	uint8_t pulse_count;    /* 已经打了几个时钟脉冲。最多打 9 个 ——
	                         * 一个字节 8 位，再多 1 个就足够让从机把没发完的字节发完并松开 SDA */

	SOFT_I2C_SDA_HIGH();
	SOFT_I2C_SCL_HIGH();
	SoftI2C_WaitOneBit();

	if (SOFT_I2C_SDA_READ())
	{
		return;                     /* SDA 是高的 → 总线正常，不用管 */
	}

	for (pulse_count = 0; pulse_count < 9; pulse_count++)
	{
		SOFT_I2C_SCL_LOW();
		SoftI2C_WaitOneBit();
		SOFT_I2C_SCL_HIGH();
		SoftI2C_WaitOneBit();

		if (SOFT_I2C_SDA_READ())
		{
			break;                  /* 从机松手了 */
		}
	}

	SoftI2C_Stop();
}

/* ==================== 芯片寄存器读写 ==================== */

/* 往芯片的某一个寄存器写一个字节。
 * register_address：寄存器地址，比如 0x6B 是 PWR_MGMT_1（电源管理，用来唤醒芯片）
 * register_value  ：要写进去的值，比如 0x01 是"唤醒 + 时钟切到陀螺仪 PLL"
 * 返回 1 = 写成功；0 = 中途从机没应答（模块没接上或者坏了）。 */
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

/* 从芯片连续读若干个寄存器（芯片会自动把寄存器地址往后加，所以只要给起始地址）。
 * start_register：从哪个寄存器开始读。0x47 = Z 轴高字节；0x43 = X 轴高字节（一次读三轴时用）
 * read_buffer   ：读回来的字节放到这个数组里，数组由调用者自己准备
 * read_count    ：要读几个字节。只读 Z 轴 = 2；读 X/Y/Z 三轴 = 6
 * 返回 1 = 全部读完；0 = 中途从机没应答，此时 read_buffer 里的内容是作废的，不要用。 */
static uint8_t GyroChip_ReadRegisters(uint8_t start_register, uint8_t *read_buffer, uint8_t read_count)
{
	uint8_t byte_index;     /* 当前读到第几个字节（0 ~ read_count-1） */

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

/* 毫秒延时用 MiddleLayer 的 Delay_ms（SysTick 轮询，精确到微秒）。
 * 2026-11-21 换掉原来自己写的空循环：那个"1 毫秒 = 12000 次空循环"是估的，
 * 误差可能 30%，而且改编译优化等级就变。现在只在 Gyro_Init 里用，
 * 不在主循环、不在 I2C 位时序里，所以不会占跑车时的时间。
 * 注意：Delay_us 千万不要拿去替换 SoftI2C_WaitOneBit()——
 *   那个位延时是 25 次空循环（约 1 微秒），换成 Delay_us 要写 4 个 SysTick 寄存器，
 *   读一次 6 字节有 90 个位，每次读会从 0.9 毫秒涨到 2~3 毫秒，5 毫秒周期直接撑爆。 */

/* ==================== 对外接口 ==================== */

/* 只做三件事：确认芯片活着、唤醒、配置量程。很快，不阻塞。 */
void Gyro_Init(void)
{
	uint8_t who_am_i_value = 0;   /* 从 0x75 读回来的"身份号"：ICM-20602 = 0x12（十进制 18），
	                               * MPU6050 = 0x68（十进制 104）。
	                               * 读不到、或者读出来既不是 18 也不是 104，就认为模块没接上。 */
	uint8_t retry_times;          /* 重试计数。芯片上电后要过约 100 毫秒才开始应答，
	                               * 所以最多试 GYRO_CHIP_DETECT_RETRY = 5 次，每次隔 20 毫秒。 */

	Delay_ms(GYRO_CHIP_POWER_ON_WAIT_MS);
	SoftI2C_InitPins();
	SoftI2C_BusRecovery();      /* 万一上电时从机拉着 SDA，先把总线放干净 */

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
		Delay_ms(GYRO_CHIP_DETECT_RETRY_MS);
	}

	Gyro_WhoAmIValue = who_am_i_value;

	if (!Gyro_IsAvailable)
	{
		return;                      /* 芯片不在，后面全部跳过，不影响跑车 */
	}

	GyroChip_WriteRegister(GYRO_CHIP_REG_POWER_MGMT,  GYRO_CHIP_POWER_ON_VALUE);
	Delay_ms(10);
	GyroChip_WriteRegister(GYRO_CHIP_REG_SAMPLE_RATE, GYRO_CHIP_SAMPLE_RATE_DIV);
	GyroChip_WriteRegister(GYRO_CHIP_REG_LOW_PASS,    GYRO_CHIP_LOW_PASS_VALUE);
	GyroChip_WriteRegister(GYRO_CHIP_REG_GYRO_CONFIG, GYRO_CHIP_RANGE_500_VALUE);

	/* 配置完再等一会儿，让芯片彻底稳下来，才开始采零偏 */
	Delay_ms(GYRO_CHIP_STARTUP_WAIT_MS);

	Gyro_BiasSkipCount = GYRO_BIAS_SKIP_SAMPLES;
}

/* 每 5 毫秒调一次：前 1 秒测零偏，之后按 Z 轴积分
 * 一次读 6 个字节把 X/Y/Z 全拿回来：模块要是侧着装，绕竖直轴的转动
 * 就落在 X 或 Y 上，只读 Z 会永远是 0（2026-11-21 踩过这个坑）。 */
void Gyro_Update(uint32_t current_ms)
{
	uint8_t axis_raw_bytes[6];          /* 从芯片读回来的原始字节，最多 6 个（X/Y/Z 各 2 个）。
	                                     * 只读 Z 轴时只用到前 2 个。原始值 = 高低两个字节拼起来的 16 位数 */
	int16_t axis_x_raw;                 /* X 轴原始读数（拼好的 16 位数）。只读 Z 轴时恒为 0，台架诊断用 */
	int16_t axis_y_raw;                 /* Y 轴原始读数。只读 Z 轴时恒为 0，台架诊断用 */
	int16_t axis_z_raw;                 /* Z 轴原始读数 = 竖直轴，跑车只认这个 */
	float   rate_degrees_per_second;    /* 这一轮的转速，单位「度/秒」（已经减掉零偏）。
	                                     * 算法：(原始读数 − 零偏) ÷ 65.5 */
	float   sample_period_seconds;      /* 这一轮的周期，单位秒。正常情况下是 0.005 秒，
	                                     * 主循环被打断时会变大（最多按 0.05 秒算） */
	uint32_t delta_ms;                  /* 这次调用和上次调用隔了多少毫秒（0 ~ 50）。
	                                     * 用真实值而不是固定 5 毫秒，理由见下面那段注释 */
	uint8_t start_register;             /* 这次从哪个寄存器开始读：三轴模式 0x43，只读 Z 轴 0x47 */
	uint8_t read_byte_count;            /* 这次读几个字节：三轴 6 个，只读 Z 轴 2 个 */

	if (!Gyro_IsAvailable)
	{
		return;
	}

	/* 真实周期：这次调用和上次隔了多少毫秒。                 2026-11-21 改
	 * 为什么不再固定乘 0.005（5 毫秒）：
	 *   ① 主循环不是精确的 5 毫秒节拍。被中断、被长任务打断，某一轮就会变长；
	 *   ② 波形发送改成 DMA 以后，Debug_log 已经是微秒级返回，
	 *      「被波形阻塞约 2 毫秒」这个理由**不再成立**
	 *      （那是轮询发送时的事：100 毫秒发一次 24 字节，115200 下约 2 毫秒）；
	 *   ③ 但「固定按 5 毫秒积分」本身仍然是错的：
	 *      只要有一轮真的变成 7 毫秒，这一帧的转速贡献就低估约 28%（5 ÷ 7），
	 *      一个 374 毫秒的弯里来几次，累计能少算 1 度多，车会转过头。
	 * 所以做法保留、理由改写：它防的是「主循环被打断」，不是「被 Debug_log 打断」。
	 * 代价：一次减法，不占额外时间。 */
	if (Gyro_DtReady)
	{
		delta_ms = current_ms - Gyro_LastUpdateMs;
		if (delta_ms > GYRO_DT_MAX_MS)
		{
			delta_ms = GYRO_DT_MAX_MS;
		}
		sample_period_seconds = (float)delta_ms * 0.001f;
	}
	else
	{
		sample_period_seconds = GYRO_SAMPLE_PERIOD_SECONDS;
		Gyro_DtReady = 1;
	}
	Gyro_LastUpdateMs = current_ms;

#if GYRO_READ_ALL_AXES
	start_register  = GYRO_CHIP_REG_AXIS_START;   /* 0x43，一次拿 X/Y/Z */
	read_byte_count = 6;
#else
	start_register  = GYRO_CHIP_REG_AXIS_Z;       /* 0x47，只拿 Z */
	read_byte_count = 2;
#endif

	if (!GyroChip_ReadRegisters(start_register, axis_raw_bytes, read_byte_count))
	{
		if (Gyro_ReadErrorCount < 60000)
		{
			Gyro_ReadErrorCount++;
		}

		/* 连续失败到 10 次（50 毫秒）就试着救一次总线，帮从机松开 SDA。 */
		Gyro_ConsecutiveFailCount++;
		if (Gyro_ConsecutiveFailCount >= GYRO_RECOVERY_AFTER_FAILS)
		{
			SoftI2C_BusRecovery();
			Gyro_ConsecutiveFailCount = 0;
		}
		return;
	}

	Gyro_ConsecutiveFailCount = 0;

#if GYRO_READ_ALL_AXES
	axis_x_raw = (int16_t)(((uint16_t)axis_raw_bytes[0] << 8) | axis_raw_bytes[1]);  /* 0x43/0x44 = X 轴 */
	axis_y_raw = (int16_t)(((uint16_t)axis_raw_bytes[2] << 8) | axis_raw_bytes[3]);  /* 0x45/0x46 = Y 轴 */
	axis_z_raw = (int16_t)(((uint16_t)axis_raw_bytes[4] << 8) | axis_raw_bytes[5]);  /* 0x47/0x48 = Z 轴 */
#else
	axis_x_raw = 0;                                                                 /* 没读，置 0 */
	axis_y_raw = 0;                                                                 /* 没读，置 0 */
	axis_z_raw = (int16_t)(((uint16_t)axis_raw_bytes[0] << 8) | axis_raw_bytes[1]);  /* 0x47/0x48 = Z 轴 */
#endif

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

		/* 先记这一批的极值，好判断车是不是在动 */
		if (Gyro_ZeroBiasSampleCount == 0)
		{
			Gyro_BiasSampleMin = axis_z_raw;
			Gyro_BiasSampleMax = axis_z_raw;
		}
		else
		{
			if (axis_z_raw < Gyro_BiasSampleMin) Gyro_BiasSampleMin = axis_z_raw;
			if (axis_z_raw > Gyro_BiasSampleMax) Gyro_BiasSampleMax = axis_z_raw;
		}

		/* 极差太大 = 车被动过，这一批作废，从头再采（最多重采 GYRO_BIAS_MAX_RESTART 次）*/
		if ((Gyro_BiasRestartCount < GYRO_BIAS_MAX_RESTART) &&
		    (((int32_t)Gyro_BiasSampleMax - (int32_t)Gyro_BiasSampleMin) > GYRO_BIAS_MAX_SPREAD))
		{
			Gyro_BiasRestartCount++;
			Gyro_ZeroBiasSum         = 0.0f;
			Gyro_ZeroBiasSampleCount = 0;
			Gyro_BiasSampleMin       = axis_z_raw;
			Gyro_BiasSampleMax       = axis_z_raw;
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
		Gyro_TotalAngleDegrees += rate_degrees_per_second * sample_period_seconds;//角速度积分
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
