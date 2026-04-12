#include "app_common.h"

/*
 * app_adc.c
 * 14 路 SensorBoard ADC 模块
 *
 * ====== 传感器差异适配说明 ======
 *
 * 原始: 16 路外部 SPI ADC, 通过 SPI_1 读取, 8+8 对称布局
 *   读取方式: spi_read_16bit(SPI_1) × 8 + nyy_adc_read() × 8
 *   索引: adc_test[0..15], 对称配对 (i, 15-i), 权重 adc_quan[8]
 *
 * 现在: 14 路 SensorBoard, 通过 SPI2 + 双 CS 读取, 7+7 对称布局
 *   CS1 (PD8) → 传感器芯片 1: 通道 0~6  (物理左侧)
 *   CS2 (PD9) → 传感器芯片 2: 通道 7~13 (物理右侧)
 *   索引: adc_test[0..13], 对称配对 (i, 13-i), 权重 adc_quan[7]
 *
 * ====== 偏差计算逻辑 (算法不变, 仅调整索引) ======
 *   1) 归一化: adc_test[i] = (raw - 背景值) / (线值 - 背景值) × 10
 *   2) 二值化: adc_test[i] ≤ 6 → 白色(无线), > 6 → 黑色(有线)
 *   3) 从最外侧向内扫描, 找到第一对有线传感器, 计算偏差
 *   4) 10 次滑动平均
 *
 * ⚠️ SPI 传感器协议: 以下使用通用 16 位 SPI ADC 读取流程
 *   如实际芯片 (AD7689/ADS7953/TLC2543 等) 协议不同, 需修改
 *   sensor_spi_read_channel() 函数内部的命令字和时序
 */

/* ---- 校准数据 ---- */
float adc_beijing[SENSOR_COUNT];     // 背景值 (最小值, 无线时)
float adc_xian[SENSOR_COUNT];        // 线值   (最大值, 有线时)

/*
 * 权重数组 — 从最外侧到最内侧
 * 原始 8 权重: {0.1, 0.4, 1.2, 1.9, 2.2, 2.5, 3.0, 3.5}
 * 适配 7 权重: 去掉最外侧一对 (传感器数减少), 保持内侧权重分布
 * ⚠️ 需要根据实际传感器间距微调
 */
float adc_quan[SENSOR_HALF] = {0.3, 1.0, 1.7, 2.1, 2.4, 2.8, 3.5};

/* ---- 传感器数据 ---- */
float adc_test[SENSOR_COUNT] = {0};
float adc_sum = 0;
float adc_error_average[10] = {0};
float adc_error = 0, adc_error_l = 0;
int   if_bai[SENSOR_COUNT] = {0};
int   jiaozhun = 0;

/* ---- 内部: SPI 读取原始值 ---- */
static uint16_t sensor_raw[SENSOR_COUNT];

/*
 * SPI2 片选控制
 */
static inline void sensor_cs1_low(void)  { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); }
static inline void sensor_cs1_high(void) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);   }
static inline void sensor_cs2_low(void)  { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); }
static inline void sensor_cs2_high(void) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);   }

/*
 * 通过 SPI2 读取一个 16 位值
 * ⚠️ 此函数假设 SPI ADC 在拉低 CS 后, 每次 SPI 传输返回一个通道的数据
 *    实际协议可能需要先发送通道选择命令字, 请根据芯片手册修改
 */
static uint16_t sensor_spi_read16(void)
{
    uint16_t tx = 0xFFFF;  // 时钟填充 (有些 ADC 需要特定命令字)
    uint16_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&tx, (uint8_t *)&rx, 1, 10);
    return rx;
}

/*
 * 读取一个芯片的多个通道
 * ⚠️ 以下为通用读取流程:
 *   1) 拉低 CS
 *   2) 连续 SPI 读取 n_channels 次
 *   3) 拉高 CS
 *   实际时序可能需要在每次读取之间插入延时, 或先发送通道配置
 */
static void sensor_read_chip(int cs_id, uint16_t *buf, int n_channels)
{
    if (cs_id == 1) sensor_cs1_low(); else sensor_cs2_low();

    for (int i = 0; i < n_channels; i++)
    {
        buf[i] = sensor_spi_read16();
        /* 如需要通道间延时, 在此添加:
         * 简易延时: for(volatile int d=0; d<20; d++);  // ~1us @168MHz
         */
    }

    if (cs_id == 1) sensor_cs1_high(); else sensor_cs2_high();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     传感器板初始化
//-------------------------------------------------------------------------------------------------------------------
void app_adc_init(void)
{
    /* 确保 CS 引脚初始为高 (CubeMX 已设 PinState=SET) */
    sensor_cs1_high();
    sensor_cs2_high();

    /* 初始化校准数据 */
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        adc_beijing[i] = 60000.0f;    // 背景值初始化为极大值
        adc_xian[i]    = 0.0f;        // 线值初始化为 0
    }

    /* 进行一次空读, 丢弃首次数据 (与原始初始化行为一致) */
    sensor_read_chip(1, sensor_raw, SENSOR_HALF);
    sensor_read_chip(2, sensor_raw + SENSOR_HALF, SENSOR_HALF);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取传感器 + 归一化 + 偏差计算
// 备注信息     每 1ms 由 TIM7 中断调用
//              算法逻辑严格对应原始 nyy_adc_get(), 仅索引范围从 16 → 14
//-------------------------------------------------------------------------------------------------------------------
void app_adc_get(void)
{
    adc_error_l = adc_error;
    adc_sum = 0;

    /*
     * 读取 14 路传感器原始值
     * 芯片1 (CS1/PD8): adc_test[6..0]  — 注意: 原始代码反序存储 (15-i)
     * 芯片2 (CS2/PD9): adc_test[13..7]
     *
     * 为保持与原始代码一致的反序读取行为:
     *   芯片1 读 7 次 → 存入 [6], [5], [4], [3], [2], [1], [0]
     *   芯片2 读 7 次 → 存入 [13], [12], [11], [10], [9], [8], [7]
     */
    sensor_cs1_low();
    for (int i = 0; i < SENSOR_HALF; i++)
    {
        uint16_t val = sensor_spi_read16();
        adc_test[SENSOR_HALF - 1 - i] = (float)val;
        adc_sum += adc_test[SENSOR_HALF - 1 - i];
    }
    sensor_cs1_high();

    sensor_cs2_low();
    for (int i = 0; i < SENSOR_HALF; i++)
    {
        uint16_t val = sensor_spi_read16();
        adc_test[SENSOR_COUNT - 1 - i] = (float)val;
        adc_sum += adc_test[SENSOR_COUNT - 1 - i];
    }
    sensor_cs2_high();

    /* ---- 校准控制 (与原始逻辑一致, 按键索引不变) ---- */
    // 按键2 (anniu_zhuangtai[1]) 下降沿: 进入校准模式
    if (anniu_zhuangtai[1][0] == 0 && anniu_zhuangtai[1][1] == 1 && jiaozhun == 0)
    {
        jiaozhun = 1;
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            adc_xian[i]    = 0.0f;
            adc_beijing[i] = 60000.0f;
        }
    }
    // 按键3 (anniu_zhuangtai[2]) 下降沿: 退出校准模式
    if (anniu_zhuangtai[2][0] == 0 && anniu_zhuangtai[2][1] == 1 && jiaozhun == 1)
    {
        jiaozhun = 0;
    }

    /* 校准 LED 指示 */
    if (jiaozhun == 1) app_LED_set(3, 1);
    if (jiaozhun == 0) app_LED_set(3, 0);

    /* ---- 校准采集阶段 ---- */
    if (jiaozhun == 1)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            if (adc_test[i] > adc_xian[i])    adc_xian[i]    = adc_test[i];
            if (adc_test[i] < adc_beijing[i])  adc_beijing[i] = adc_test[i];
        }
    }
    /* ---- 正常巡线模式: 归一化 + 偏差计算 ---- */
    else if (adc_beijing[3] != 0)  // 索引3 对应中间位置, 检查校准有效性
    {
        /* 归一化到 0~10 */
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            float range = adc_xian[i] - adc_beijing[i];
            if (range > 1.0f)  // 防除零
                adc_test[i] = (adc_test[i] - adc_beijing[i]) / range * 10.0f;
            else
                adc_test[i] = 0.0f;
        }

        /* 滑动平均缓冲区移位 (与原始完全一致) */
        adc_error = 0;
        for (int i = 0; i < 9; i++)
            adc_error_average[i] = adc_error_average[i + 1];
        adc_error_average[9] = 0;

        /* 二值化: ≤6 为白色(无线) */
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            if_bai[i] = (adc_test[i] <= 6.0f) ? 1 : 0;
        }

        /*
         * 偏差计算 — 从最外侧向内扫描
         * 对称配对: sensor[i] ↔ sensor[SENSOR_COUNT-1-i]
         *           即 (0,13), (1,12), (2,11), (3,10), (4,9), (5,8), (6,7)
         *
         * 逻辑与原始完全一致, 仅将 "8" → SENSOR_HALF, "15-i" → SENSOR_COUNT-1-i
         */
        for (int i = 0; i < SENSOR_HALF; i++)
        {
            int j = SENSOR_COUNT - 1 - i;   // 对称位置
            if (if_bai[i] == 0 || if_bai[j] == 0)
            {
                if (if_bai[i] == 0 && if_bai[j] == 0)
                {
                    /* 两侧都检测到线: 偏差 = (左值-右值) × 权重 */
                    adc_error_average[9] = (adc_test[i] - adc_test[j]) * adc_quan[SENSOR_HALF - 1 - i];
                }
                else if (if_bai[i] == 0)
                {
                    /* 仅左侧检测到线: 大幅偏右 */
                    adc_error_average[9] = 10.0f * adc_quan[SENSOR_HALF - 1 - i];
                }
                else /* if_bai[j] == 0 */
                {
                    /* 仅右侧检测到线: 大幅偏左 */
                    adc_error_average[9] = -10.0f * adc_quan[SENSOR_HALF - 1 - i];
                }
                break;  // 只使用最外侧的有效配对
            }
        }

        /* 归一化 (原始除以 16, 14路适当调整保持偏差量级一致) */
        adc_error_average[9] /= 14.0f;

        /* 计算 10 次滑动平均 */
        for (int i = 0; i < 10; i++)
        {
            adc_error += adc_error_average[i] / 10.0f;
        }
    }
}
