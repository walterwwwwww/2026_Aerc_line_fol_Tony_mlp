/*
 * stm32f4xx_it_user.c
 *
 * ====== 速度环 PID 测试专用版本 ======
 *
 * 替换原版 (跑 app_START 巡线状态机) 的 TIM7 ISR,
 * 改成只跑速度闭环, 配合 main.c 的速度环测试代码使用
 *
 * 1ms 中断里干这几件事:
 *   1) app_encoder_get()  - 读编码器, 更新 speedl/speedr
 *   2) duty = 0           - 强制关闭转向 (不调 angle_ring)
 *   3) speed_test_active 为 1 时:
 *        speed_ = base_speed              (目标速度, 可被 VOFA 实时修改)
 *        app_motor_get(1)                  (速度环 PID, 算出 pwm_)
 *        app_motor_out(1)                  (输出到 H 桥)
 *      为 0 时:
 *        speed_ = 0
 *        app_motor_get(0)                  (复位 pwm_ = 0)
 *        app_motor_out(0)                  (强停)
 *   4) app_vofa_send()    - 把数据 DMA 给 VOFA+
 *
 * 不调用:
 *   - app_gyro_get        (没初始化, 调了会卡死在 SPI 等待)
 *   - app_adc_get         (没初始化)
 *   - app_get_button      (BTN1/BTN3 在主循环直接 HAL_GPIO_ReadPin)
 *   - app_START           (整个巡线状态机不要)
 */

#include "app_common.h"

/* main.c 里定义 */
extern volatile int speed_test_active;

/* app_control.c 里定义, 用作目标速度 (VOFA 可在线改 base_speed=xx) */
extern int base_speed;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        /* ---- 1) 读编码器 ---- */
        app_encoder_get();

        /* ---- 2) 强制无转向 (两轮命令值相同) ---- */
        duty = 0;

        /* ---- 3) 速度环 ---- */
        if (speed_test_active)
        {
            speed_ = base_speed;        // 目标速度, VOFA 可改
            app_motor_get(1);           // 速度 PID
            app_motor_out(1);           // 输出
        }
        else
        {
            speed_ = 0;
            app_motor_get(0);           // 复位 pwm_ = 0
            app_motor_out(0);           // 全停
        }

        /* ---- 4) 上传 VOFA ---- */
        app_vofa_send();
    }
}