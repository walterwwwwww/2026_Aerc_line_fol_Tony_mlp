#include "app_common.h"

/*
 * app_gyro.c
 * 陀螺仪 (IMU) 模块 — ICM-20948 版本
 *
 * ICM-20948 vs ICM-20602 关键差异:
 *   - 寄存器分 4 个 Bank (0~3), 通过 REG_BANK_SEL (0x7F) 切换
 *   - WHO_AM_I = 0x00, 期望值 = 0xEA
 *   - 陀螺仪数据在 Bank 0: GYRO_ZOUT_H = 0x37
 *   - 陀螺仪配置在 Bank 2: GYRO_CONFIG_1 = 0x01
 *   - ±2000°/s 灵敏度 = 16.4 LSB/(°/s)
 *
 * 硬件: SPI1 (PB3/PB4/PB5), CS = PD7
 */

/* ---- 陀螺仪变量 (与原始完全一致) ---- */
float jiaoduz = 0, jiaodux = 0, jiaoduy = 0;
float jiaoduz_l = 0, jiaodux_l = 0, jiaoduy_l = 0;
float wenpiao = 0;
float oncez = 0;
float jiaosudu = 0;

/* 路径记忆变量 */
float length = 0;
float once_L = 0;
float now_x = 0;
float now_y = 0;
int   num_coord = -1;
int   coord[200][2] = {0};
float pi;

/* 角速度环变量 */
float jiaosuderrorl = 0, jiaosuderror = 0;
float duty = 0;
float jiaosudutobe = 0;
float P_A = 15, I_A = 0.5f;

/* ============================ ICM-20948 寄存器定义 ============================ */

/* Bank 0 */
#define ICM_REG_WHO_AM_I        0x00
#define ICM_REG_USER_CTRL       0x03
#define ICM_REG_PWR_MGMT_1      0x06
#define ICM_REG_PWR_MGMT_2      0x07
#define ICM_REG_GYRO_XOUT_H     0x33
#define ICM_REG_GYRO_ZOUT_H     0x37
#define ICM_REG_GYRO_ZOUT_L     0x38

/* Bank 2 */
#define ICM_REG_GYRO_SMPLRT_DIV 0x00
#define ICM_REG_GYRO_CONFIG_1   0x01

/* 公共 */
#define ICM_REG_BANK_SEL        0x7F

#define ICM_WHOAMI_VALUE        0xEA

/* ============================ SPI 底层 ============================ */

static inline void imu_cs_low(void)  { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); }
static inline void imu_cs_high(void) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);   }

static uint8_t imu_spi_rw(uint8_t byte)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx, 1, 10);
    return rx;
}

static void imu_write_reg(uint8_t reg, uint8_t val)
{
    imu_cs_low();
    imu_spi_rw(reg & 0x7F);   /* bit7=0 → 写 */
    imu_spi_rw(val);
    imu_cs_high();
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t val;
    imu_cs_low();
    imu_spi_rw(reg | 0x80);   /* bit7=1 → 读 */
    val = imu_spi_rw(0xFF);
    imu_cs_high();
    return val;
}

/* 切换寄存器 Bank (0~3) */
static void imu_set_bank(uint8_t bank)
{
    imu_write_reg(ICM_REG_BANK_SEL, (bank & 0x03) << 4);
}

/* 读取 16 位陀螺仪 Z 轴数据 (有符号) */
static int16_t imu_read_gyro_z(void)
{
    imu_set_bank(0);
    uint8_t h = imu_read_reg(ICM_REG_GYRO_ZOUT_H);
    uint8_t l = imu_read_reg(ICM_REG_GYRO_ZOUT_L);
    return (int16_t)((h << 8) | l);
}

/*
 * ICM-20948 初始化序列
 */
static int imu_init_sequence(void)
{
    /* 确保在 Bank 0 */
    imu_set_bank(0);

    /* 复位 */
    imu_write_reg(ICM_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 唤醒: 自动选择最佳时钟 */
    imu_set_bank(0);
    imu_write_reg(ICM_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(50);

    /* 检查 WHO_AM_I */
    imu_set_bank(0);
    uint8_t id = imu_read_reg(ICM_REG_WHO_AM_I);
    if (id != ICM_WHOAMI_VALUE)
    {
        return -1;  /* 不是 ICM-20948, 上层会重试 */
    }

    /* 使能所有轴 */
    imu_set_bank(0);
    imu_write_reg(ICM_REG_PWR_MGMT_2, 0x00);
    HAL_Delay(10);

    /* ---- 切到 Bank 2 配置陀螺仪 ---- */
    imu_set_bank(2);

    /* 陀螺仪采样率分频: ODR = 1.1kHz / (1+0) ≈ 1.1kHz */
    imu_write_reg(ICM_REG_GYRO_SMPLRT_DIV, 0x00);

    /*
     * GYRO_CONFIG_1:
     *   bit[5:3] = GYRO_DLPFCFG = 001 (DLPF BW ~150Hz)
     *   bit[2:1] = GYRO_FS_SEL  = 11  (±2000°/s)
     *   bit[0]   = GYRO_FCHOICE = 1   (使能 DLPF)
     *
     *   值 = 0b00_001_11_1 = 0x0F
     */
    imu_write_reg(ICM_REG_GYRO_CONFIG_1, 0x0F);

    /* 切回 Bank 0 (数据读取都在 Bank 0) */
    imu_set_bank(0);

    HAL_Delay(20);
    return 0;
}

/* ============================ 应用层接口 ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU 初始化 + 零漂校准
// 备注信息     与原始 nyy_gyro_init() 对应
//              循环尝试初始化, 最多 20 次 (防止没接 IMU 时死循环)
//              然后采集 1000 次数据计算零漂
//-------------------------------------------------------------------------------------------------------------------
void app_gyro_init(void)
{
    imu_cs_high();
    HAL_Delay(10);

    /* 循环尝试初始化, 最多 20 次 (防止没接 IMU 时死循环) */
    int retry = 0;
    while (imu_init_sequence() != 0)
    {
        retry++;
        if (retry >= 20)
        {
            /* IMU 初始化失败, 跳过零漂校准, 继续运行 */
            wenpiao = 0;
            now_x = 0;
            now_y = 0;
            pi = acosf(-1.0f);
            duty = 0;
            return;
        }
        HAL_Delay(100);
    }

    HAL_Delay(200);

    /* 零漂校准: 采集 1000 次, 计算平均值 */
    wenpiao = 0;
    for (int i = 0; i < 1000; i++)
    {
        int16_t raw_z = imu_read_gyro_z();
        wenpiao += (float)(raw_z) / GYRO_SCALE_FACTOR;
        HAL_Delay(1);
    }
    wenpiao /= 1000.0f;

    now_x = 0;
    now_y = 0;
    pi = acosf(-1.0f);
    duty = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取一次陀螺仪角速度并进行角度积分
// 备注信息     每 1ms 调用, 算法与原始 nyy_gyro_get() 完全一致
//-------------------------------------------------------------------------------------------------------------------
void app_gyro_get(void)
{
    int16_t raw_z = imu_read_gyro_z();

    jiaoduz_l = jiaoduz;
    oncez = (float)(raw_z) / GYRO_SCALE_FACTOR;
    jiaosudu = oncez - wenpiao;
    jiaoduz += (oncez - wenpiao) * 0.001f;

    if (jiaoduz > 180.0f)
        jiaoduz -= 360.0f;
    else if (jiaoduz <= -180.0f)
        jiaoduz += 360.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     路径记忆 — 通过陀螺仪与编码器记录坐标
// 备注信息     算法与原始 path_menmory() 完全一致, 不做任何修改
//-------------------------------------------------------------------------------------------------------------------
void path_menmory(void)
{
    length += (float)(speedl + speedr) / 10.0f;
    once_L += (float)(speedl + speedr) / 10.0f;
    float shift = (float)(speedl + speedr) / 10.0f;

    now_x = now_x - shift * sinf(jiaoduz / 180.0f * pi);
    now_y = now_y + shift * cosf(jiaoduz / 180.0f * pi);

    if (num_coord < 200)
    {
        if (once_L >= 50.0f)
        {
            num_coord++;
            coord[num_coord][0] = (int)now_x;
            coord[num_coord][1] = (int)now_y;
            once_L -= 50.0f;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角速度环 (转向控制)
// 参数说明     mode: 0=关闭, 1=角速度环模式
// 备注信息     算法与原始 angle_ring() 完全一致
//-------------------------------------------------------------------------------------------------------------------
void angle_ring(int mode)
{
    if (mode == 0)
    {
        duty = 0;
    }
    if (mode == 1)
    {
        jiaosuderrorl = jiaosuderror;
        jiaosuderror = jiaosudutobe - jiaosudu;

        if (speed_ <= 30)
            duty += (P_A * (jiaosuderror - jiaosuderrorl) + I_A * jiaosuderror);
        else
            duty += (P_A * (jiaosuderror - jiaosuderrorl) + I_A * jiaosuderror) * speed_ / 40.0f;

        if (duty > 18000)  duty = 18000;
        if (duty < -18000) duty = -18000;

        /* 低速时额外限幅: 防止差速远超前进速度导致原地转圈 */
        {
            float duty_max = (speed_ > 10) ? speed_ * 100.0f : 1000.0f;
            if (duty >  duty_max) duty =  duty_max;
            if (duty < -duty_max) duty = -duty_max;
        }
    }
}