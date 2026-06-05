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
 * 吞吐: 1 字节/ms = 1kB/s; 80字节/帧 → 80ms/帧 ≈ 12Hz, 调试足够.
 * ISR 额外开销: 帧内 ~180ns/tick, 帧加载时 snprintf ~40μs/vofa_interval.
 */

/* ---- TX FIFO ---- */
#define VOFA_BUF_SIZE  128
static char     vofa_tx_buf[VOFA_BUF_SIZE];
static uint16_t vofa_tx_len = 0;
static uint16_t vofa_tx_pos = 0;

/* ---- 速率分频 ---- */
static int vofa_interval = VOFA_DEFAULT_INTERVAL;
static int vofa_cnt      = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA 模块初始化
// 备注信息     在 main.c 的 USER CODE BEGIN 2 中调用, 必须在 MX_USART2_UART_Init() 之后
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
// 备注信息     不依赖任何中断, 直接轮询 TXE 寄存器标志
//              vofa_interval tick 周期准备新帧 (snprintf ~40μs, 很少执行)
//              每 tick 发 1 字节; 80字节帧约 80ms 发完 (~12Hz)
//
//              通道定义 (全部整数):
//                CH0: adc_error ×100       偏差环输入
//                CH1: jiaosudutobe ×100    期望角速度
//                CH2: jiaosudu ×100        实际角速度
//                CH3: jiaosuderror ×100    角速度误差
//                CH4: duty                 差速 PWM
//                CH5: speed_               目标速度
//                CH6: speedl               左轮速度
//                CH7: speedr               右轮速度
//                CH8: pwm_                 速度环输出
//                CH9: run                  运行状态
//                CH10: turn_count          第几弯
//                CH11: turn_forced_flag    强制转向中
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_tick(void)
{
    /* ---- 速率分频: 每 vofa_interval 次调用准备一帧 ---- */
    vofa_cnt++;
    if (vofa_cnt >= vofa_interval && vofa_tx_pos >= vofa_tx_len)
    {
        vofa_cnt = 0;
        int len = snprintf(vofa_tx_buf, VOFA_BUF_SIZE,
            "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            (int)(adc_error      * 100),  /* CH0  */
            (int)(jiaosudutobe   * 100),  /* CH1  */
            (int)(jiaosudu       * 100),  /* CH2  */
            (int)(jiaosuderror   * 100),  /* CH3  */
            (int)(duty),                  /* CH4  */
            speed_,                       /* CH5  */
            speedl,                       /* CH6  */
            speedr,                       /* CH7  */
            pwm_,                         /* CH8  */
            run,                          /* CH9  */
            turn_count,                   /* CH10 */
            turn_forced_flag              /* CH11 */
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
        /* 只在 UART 不被主循环命令回显占用时写入 */
        if ((USART2->SR & USART_SR_TXE) && (huart2.gState == HAL_UART_STATE_READY))
        {
            USART2->DR = (uint8_t)vofa_tx_buf[vofa_tx_pos++];
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     运行时修改发送间隔
// 参数说明     interval_ms: 发送间隔, 单位 ms (最小 1, 建议 5~50)
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_set_interval(int interval_ms)
{
    if (interval_ms < 1)    interval_ms = 100;
    if (interval_ms > 1000) interval_ms = 1000;
    vofa_interval = interval_ms;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环占位接口 (现在逻辑在 app_vofa_tick 里)
// 备注信息     Thread mode 因 TIM7 ISR > 周期而基本不运行, 此函数可保留调用
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_send(void)
{
    /* no-op: VOFA transmission is now handled entirely by app_vofa_tick() in TIM7 ISR */
}
