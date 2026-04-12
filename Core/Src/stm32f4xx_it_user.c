/*
 * stm32f4xx_it_user.c
 *
 * ====== 本文件不是独立源文件! ======
 * 本文件包含需要插入到 CubeMX 自动生成的 stm32f4xx_it.c 中的用户代码
 *
 * ====== 方法一: 在 stm32f4xx_it.c 中添加回调 (推荐) ======
 * 在 stm32f4xx_it.c 的 USER CODE BEGIN Includes 中添加:
 *   #include "app_common.h"
 *
 * 然后在 stm32f4xx_it.c 的 USER CODE BEGIN 1 中添加下面的回调函数
 *
 * ====== 方法二: 在任意 .c 文件中实现回调 (更灵活) ======
 * HAL_TIM_PeriodElapsedCallback 是弱符号函数,
 * 可以在任意 .c 文件中实现, 只要整个工程中只有一个实现即可
 * 直接将下面的函数放到 main.c 的 USER CODE BEGIN 4 区域也可以
 */

#include "app_common.h"

/*
 * TIM7 周期中断回调 — 1ms 控制循环
 *
 * 对应原始工程中 cm4_isr.c 的 pit0_ch1_isr()
 * 原始 PIT_CH1 配置: pit_init(PIT_CH1, 1000) → 1000us = 1ms
 * STM32 TIM7 配置: Prescaler=83, Period=999 → 84MHz/84/1000 = 1kHz = 1ms
 *
 * 调用顺序与原始完全一致:
 *   1) 编码器读取
 *   2) 陀螺仪读取
 *   3) ADC 传感器读取
 *   4) 按键读取
 *   5) 主状态机
 *   6) VOFA+ 数据发送 (DMA, 不阻塞)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        /* ---- 与原始 pit0_ch1_isr() 完全对应 ---- */

        app_encoder_get();      // 编码器 (原 nyy_encoder_get)
        app_gyro_get();         // 陀螺仪 (原 nyy_gyro_get)
        app_adc_get();          // ADC 传感器 (原 nyy_adc_get)
        app_get_button();       // 按键 (原 nyy_get_anniu)

        app_START(0);           // 主状态机 (原 START(0))

        app_vofa_send();        // VOFA+ DMA 发送 (内部自动分频, 不阻塞)
    }
}