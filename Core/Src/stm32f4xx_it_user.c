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
 *   6) VOFA+ 字节发送 (直接写 USART2->DR, 每 tick 1 字节)
 *
 * ⚠️ 相比上一版的变动:
 *   移除了 app_pidtest_tick() / app_pidtest_is_active() 调用
 *   直接调 app_START(0) 进入巡线状态机
 *
 *   PID 参数调整现在通过 VOFA+ 命令面板实现:
 *     - USART2 IDLE 中断 (stm32f4xx_it.c::USART2_IRQHandler) 收 ASCII 命令
 *     - 主循环 (main.c::while(1)) 的 app_vofa_cmd_process() 解析并修改参数
 *   本中断回调里完全不用管 VOFA 命令, 只专注 1ms 控制循环
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        /* ---- 传感器读取 ---- */
        app_encoder_get();      // 编码器 → speedl, speedr
        app_gyro_get();         // 陀螺仪 → jiaosudu, jiaoduz
        app_adc_get();          // ADC 传感器 → adc_error
        app_get_button();       // 按键 → anniu_zhuangtai

        /* ---- 主控制: 正常巡线 (BTN1 启停) ---- */
        app_START(0);

        /* ---- 数据上传: ISR 驱动字节 FIFO, 每 tick 发一字节, 无需 USART2 中断 ---- */
        /* Thread mode 因 ISR > 周期被尾链饿死, 必须在此处驱动 VOFA 发送          */
        app_vofa_tick();
    }
}