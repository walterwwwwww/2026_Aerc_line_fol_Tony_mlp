#include "app_common.h"

/*
 * app_adc.c
 * 14 路 SensorBoard ADC 模块
 *
 * 硬件:
 *   SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI), CS = PD8
 *   SPI2 配置: 16-bit, Master, CPOL=0 CPHA=0
 *
 * 通信协议 (与 ir_sensor_Internal STM32F103 Slave 配合):
 *   1) 拉低 CS
 *   2) Master 发送 uint16_t 2000 (命令字)
 *   3) 短延时等待 Slave 准备 TX
 *   4) Master 接收 14 × uint16_t (28 bytes)
 *   5) 拉高 CS
 *
 * 误差计算算法 (配对扫描法):
 *   1. 14通道归一化到 0~10 (0=背景, 10=线上)
 *   2. 阈值判断黑白 (<=6 判定为白/背景)
 *   3. 从最外侧向内扫描对称配对 (i, 13-i)
 *      找到第一对有线传感器后用 adc_quan 权重计算偏差
 *   4. 10次滑动平均滤波
 */

/* ---- 校准数据 ---- */
float adc_beijing[SENSOR_COUNT];     // 背景值 (最小值, 无线时)
float adc_xian[SENSOR_COUNT];        // 线值   (最大值, 有线时)

/*
 * 权重数组 — 从最外侧到最内侧
 * 原始 8 权重: {0.1, 0.4, 1.2, 1.9, 2.2, 2.5, 3.0, 3.5}
 * 适配 7 权重: 去掉最外侧一对 (传感器数减少), 保持内侧权重分布
 */
float adc_quan[SENSOR_HALF] = {0.3, 1.0, 1.7, 2.1, 2.4, 2.8, 3.5};

/* ---- 传感器数据 ---- */
float adc_test[SENSOR_COUNT] = {0};
float adc_sum = 0;
float adc_error_average[10] = {0};
float adc_error = 0, adc_error_l = 0;
int   if_bai[SENSOR_COUNT] = {0};
int   jiaozhun = 0;

/* ---- 调试用: SPI 原始值, 供 Live Watch 观测 ---- */
uint16_t adc_raw[SENSOR_COUNT] = {0};

/* ============================ SPI 底层 (来自参考工程) ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 ir_sensor_Internal 读取 14 路 ADC 数据
// 备注信息     发送命令 2000, 等待 slave 准备, 然后接收 14 × uint16_t (28 bytes)
//              CS = PD8
//              不可使用 HAL_Delay (在中断中调用), 用 NOP 循环代替
//-------------------------------------------------------------------------------------------------------------------
static void ir_sensor_read(uint16_t *out)
{
    uint16_t cmd = 2000;
    ADC_CS_LOW();
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&cmd, 1, 10);
    /* 等待 slave 准备好 TX (~3 µs @ 168 MHz) */
    for (volatile int d = 0; d < 500; d++);
    HAL_SPI_Receive(&hspi2, (uint8_t *)out, 14, 10);
    ADC_CS_HIGH();
}

/* ============================ 应用层接口 ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     传感器板初始化
//-------------------------------------------------------------------------------------------------------------------
void app_adc_init(void)
{
    ADC_CS_HIGH();

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        adc_beijing[i] = 60000.0f;
        adc_xian[i]    = 0.0f;
    }

    /* 一次空读, 丢弃首次数据 */
    uint16_t dummy[SENSOR_COUNT];
    ir_sensor_read(dummy);
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

    /* ---- 读取 14 路传感器原始值 ---- */
    uint16_t raw[SENSOR_COUNT];
    ir_sensor_read(raw);
    memcpy(adc_raw, raw, sizeof(adc_raw));   /* 保存原始值供 Live Watch 观测 */

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        adc_test[i] = (float)raw[i];
        adc_sum += adc_test[i];
    }

    /* ---- 校准控制 (按键2 开始, 按键3 结束) ---- */
    if (anniu_zhuangtai[1][0] == 0 && anniu_zhuangtai[1][1] == 1 && jiaozhun == 0)
    {
        jiaozhun = 1;
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            adc_xian[i]    = 0.0f;
            adc_beijing[i] = 60000.0f;
        }
    }
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
    else if (adc_beijing[3] != 0)
    {
        /* 归一化到 0~10 */
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            float range = adc_xian[i] - adc_beijing[i];
            if (range > 1.0f)
                adc_test[i] = (adc_test[i] - adc_beijing[i]) / range * 10.0f;
            else
                adc_test[i] = 0.0f;
        }

        /* 滑动平均缓冲区移位 */
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

        /* 归一化 */
        adc_error_average[9] /= 14.0f;

        /* 计算 10 次滑动平均 */
        for (int i = 0; i < 10; i++)
        {
            adc_error += adc_error_average[i] / 10.0f;
        }
    }
}