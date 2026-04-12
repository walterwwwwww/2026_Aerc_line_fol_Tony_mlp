#include "app_common.h"

/*
 * app_encoder.c
 * 编码器模块 — 基于用户提供的代码
 *
 * 移植说明:
 *   原始代码 (nyy_encoder.c) 使用逐飞库的 encoder_dir_init() 和 encoder_get_count()
 *   STM32 使用 TIM 外部时钟计数模式 (TI1 Edge Detector):
 *     TIM2 (PA0 脉冲, PA1 方向GPIO) → 左编码器   对应原始 TC_CH19_ENCODER (P08_0, P08_1)
 *     TIM4 (PD12 脉冲, PD13 方向GPIO) → 右编码器  对应原始 TC_CH07_ENCODER (P02_0, P02_1)
 *
 *   ⚠️ 重要: CubeMX 中 TIM2/TIM4 的 Prescaler 必须设为 0!
 *     Period 保持 65535 (16位计数器最大值)
 *
 *   ⚠️ 注意: 原始 nyy_encoder_get() 每 1ms 直接输出 speedl/speedr
 *     本版本改为每 10ms 输出一次平均值, 这会影响 PID 响应
 *     如需恢复原始行为, 将 enc_cnt 阈值改为 1 并去掉除法
 */

int speedl = 0;     // 左轮速度 (脉冲数/ms, 正=前进)
int speedr = 0;     // 右轮速度 (脉冲数/ms, 正=前进)

static int16_t enc_accum_l = 0;   // 左轮脉冲累积
static int16_t enc_accum_r = 0;   // 右轮脉冲累积
static int      enc_cnt     = 0;   // 累积计数 (每10ms输出一次)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
// 参数说明     无
// 返回参数     无
// 使用示例     app_encoder_init();
// 备注信息     启动TIM2和TIM4的外部时钟计数模式
//              清零计数器，确保首次读取从0开始
//-------------------------------------------------------------------------------------------------------------------
void app_encoder_init(void)
{
    // 启动外部时钟计数模式 (TI1 Edge Detector)
    HAL_TIM_Base_Start(&htim2);    // 左编码器
    HAL_TIM_Base_Start(&htim4);    // 右编码器

    // 清零计数器
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取一次编码器数值
// 参数说明     无
// 返回参数     无
// 使用示例     app_encoder_get();
// 备注信息     每1ms在TIM7中断中调用一次
//              读取16位计数值, 通过方向GPIO判断正反转
//              每10ms输出一次平均速度
//
//              方向GPIO: PA1=左Dir, PD13=右Dir
//              ⚠️ 如果实际方向不对, 在此调整取反逻辑或交换 L/R
//-------------------------------------------------------------------------------------------------------------------
void app_encoder_get(void)
{
    // 读取脉冲数 (计数器只计 TI1 边沿, 永远为正)
    uint16_t raw_l = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    uint16_t raw_r = __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // 读取方向 GPIO: PA1=左Dir, PD13=右Dir
    // HIGH=正转, LOW=反转 (如方向反则调换符号)
    int dir_l = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
    int dir_r = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);

    // 注意: 左编码器计数 → 左速度, 右编码器计数 → 右速度
    // 右编码器取反与原始代码 speedr = -encoder_get_count(TC_CH07_ENCODER) 对应
    enc_accum_l += dir_l ? (int16_t)raw_l : -(int16_t)raw_l;
    enc_accum_r += dir_r ? -(int16_t)raw_r : (int16_t)raw_r;

    // 每 10ms 输出一次平均速度
    enc_cnt++;
    if (enc_cnt >= 10) {
        speedl = enc_accum_r / 10;
        speedr = enc_accum_l / 10;
        enc_accum_l = 0;
        enc_accum_r = 0;
        enc_cnt     = 0;
    }
}