#include "app_common.h"

/*
 * app_vofa.c
 * VOFA+ FireWater 协议数据发送模块
 *
 * 架构: ISR 驱动的字节 FIFO, 完全在 TIM7 ISR 内运行, 不依赖 Thread mode.
 *
 * 背景: TIM7 ISR ≈1.1ms > 1ms 周期, ARM Cortex-M4 尾链机制导致 Thread mode
 * 永远得不到 CPU. 任何依赖主循环的 TX 方案 (IT/DMA/Transmit) 均会卡死.
 *
 * 解法:
 *   app_vofa_tick() — 在 TIM7 ISR 末尾调用:
 *     1) 每 vofa_interval 次调用执行一次 snprintf, 把新帧写入 TX FIFO
 *     2) 每次调用发一字节 (直接写 USART2->DR, 无需 USART2 中断)
 *
 * 吞吐: ISR ≈909Hz → ~909字节/s; 180字节/帧 → ~5Hz 刷新率, PID 调参足够.
 * ISR 额外开销: 帧内 ~180ns/tick; 帧加载时 snprintf 28通道 ≈70μs.
 *
 * ========= 通道定义 (28 通道, 全部整数) =========
 *
 * 外环 (偏差 → 角速度):
 *   CH0  adc_error×100      传感器偏差
 *   CH1  jiaosudutobe×100   期望角速度
 *
 * 中环 (角速度环):
 *   CH2  jiaosudu×100       实际角速度
 *   CH3  jiaosuderror×100   角速度误差
 *   CH4  duty               差速输出
 *
 * 内环 (速度环):
 *   CH5  speed_             目标速度
 *   CH6  speedl             左轮实际速度
 *   CH7  speedr             右轮实际速度
 *   CH8  pwm_               速度环基础输出
 *   CH9  pwml               左轮最终 PWM
 *   CH10 pwmr               右轮最终 PWM
 *
 * 状态:
 *   CH11 run                运行标志
 *   CH12 jiaoduz×10         累积 Z 角度
 *   CH13 turn_count         第几弯
 *   CH14 jiaozhun           校准模式标志
 *
 * ADC 传感器阵列 (归一化 0~10, ×10 后整数):
 *   CH15..CH28  adc_test[0..13]×10
 */

/* ---- TX FIFO ---- */
#define VOFA_BUF_SIZE  256
static char     vofa_tx_buf[VOFA_BUF_SIZE];
static uint16_t vofa_tx_len = 0;
static uint16_t vofa_tx_pos = 0;

/* ---- 速率分频 ---- */
static int vofa_interval = VOFA_DEFAULT_INTERVAL;
static int vofa_cnt      = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA 模块初始化
// 备注信息     在 main.c USER CODE BEGIN 2 调用, 必须在 MX_USART2_UART_Init() 之后
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_init(void)
{
    vofa_cnt    = 0;
    vofa_tx_len = 0;
    vofa_tx_pos = 0;
    vofa_interval = VOFA_DEFAULT_INTERVAL;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ISR 驱动帧发送 — 在 TIM7 ISR 末尾每 tick 调用一次
// 备注信息     每 vofa_interval tick 执行一次 snprintf (28通道 ≈70μs)
//              每 tick 发 1 字节; 180字节帧约 200ms 发完 (~5Hz)
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_tick(void)
{
    /* ---- 速率分频: 每 vofa_interval 次调用且 FIFO 空时准备新帧 ---- */
    vofa_cnt++;
    if (vofa_cnt >= vofa_interval && vofa_tx_pos >= vofa_tx_len)
    {
        vofa_cnt = 0;
        int len = snprintf(vofa_tx_buf, VOFA_BUF_SIZE,
            /* 外环 */
            "%d,%d,"
            /* 中环 */
            "%d,%d,%d,"
            /* 内环 */
            "%d,%d,%d,%d,%d,%d,"
            /* 状态 */
            "%d,%d,%d,%d,"
            /* ADC 14通道 */
            "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",

            /* CH0-1  外环 */
            (int)(adc_error    * 100),
            (int)(jiaosudutobe * 100),

            /* CH2-4  中环 */
            (int)(jiaosudu     * 100),
            (int)(jiaosuderror * 100),
            (int)(duty),

            /* CH5-10 内环 */
            speed_,
            speedl,
            speedr,
            pwm_,
            pwml,
            pwmr,

            /* CH11-14 状态 */
            run,
            (int)(jiaoduz * 10),
            turn_count,
            jiaozhun,

            /* CH14-27 ADC 传感器阵列 */
            (int)(adc_test[0]  * 10),
            (int)(adc_test[1]  * 10),
            (int)(adc_test[2]  * 10),
            (int)(adc_test[3]  * 10),
            (int)(adc_test[4]  * 10),
            (int)(adc_test[5]  * 10),
            (int)(adc_test[6]  * 10),
            (int)(adc_test[7]  * 10),
            (int)(adc_test[8]  * 10),
            (int)(adc_test[9]  * 10),
            (int)(adc_test[10] * 10),
            (int)(adc_test[11] * 10),
            (int)(adc_test[12] * 10),
            (int)(adc_test[13] * 10)
        );
        if (len > 0 && len < VOFA_BUF_SIZE)
        {
            vofa_tx_len = (uint16_t)len;
            vofa_tx_pos = 0;
        }
    }

    /* ---- 发一字节: 直接查 TXE 寄存器, 无需中断 ---- */
    if (vofa_tx_pos < vofa_tx_len)
    {
        if ((USART2->SR & USART_SR_TXE) && (huart2.gState == HAL_UART_STATE_READY))
        {
            USART2->DR = (uint8_t)vofa_tx_buf[vofa_tx_pos++];
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     运行时修改发送间隔
// 参数说明     interval_ms: 发送间隔 ms (最小 1, 建议 5~50)
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_set_interval(int interval_ms)
{
    if (interval_ms < 1)    interval_ms = 100;
    if (interval_ms > 1000) interval_ms = 1000;
    vofa_interval = interval_ms;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环占位接口 (逻辑已全部移入 app_vofa_tick)
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_send(void)
{
    /* no-op */
}
