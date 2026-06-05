#include "app_common.h"

/*
 * app_vofa.c
 * VOFA+ FireWater 协议数据发送模块 (中断模式, 参照 Main_board/vofa.c)
 *
 * 通过 USART2 (PD5=TX, 115200 baud) + UART TXE/TC 中断发送
 * 不依赖 DMA, 不会出现 DMA 中断优先级饿死问题
 *
 * 参考: /Users/chuntingli/Aerc_line_fol/Main_board/Core/Src/vofa.c
 *   关键: HAL_UART_Transmit_IT (中断模式), 不是 DMA
 *
 * 传输带宽估算:
 *   12 通道整数, 每帧约 60~80 字节
 *   115200 baud ≈ 11520 字节/s → 最快约 150 帧/s
 *   默认 5ms 间隔 = 200Hz 调用, 实际发送取决于 UART 忙闲
 */

/* ---- TX 缓冲区 ---- */
#define VOFA_BUF_SIZE  128
static char vofa_tx_buf[VOFA_BUF_SIZE];

/* ---- 状态变量 ---- */
static volatile uint8_t vofa_busy = 0;
static int vofa_interval = VOFA_DEFAULT_INTERVAL;
static int vofa_cnt = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA 模块初始化
// 备注信息     在 main.c 的 USER CODE BEGIN 2 中调用
//              必须在 MX_USART2_UART_Init() 之后
//              不需要 DMA, 直接用 UART 中断
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_init(void)
{
    vofa_cnt = 0;
    vofa_busy = 0;
    vofa_interval = VOFA_DEFAULT_INTERVAL;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     中断模式发送一帧 FireWater 数据到 VOFA+
// 备注信息     在主循环中调用, 内部按 vofa_interval 分频
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
void app_vofa_send(void)
{
    vofa_cnt++;
    if (vofa_cnt < vofa_interval)
        return;
    vofa_cnt = 0;

    /* 上一帧还在发, 跳过本帧 (非阻塞, 参照 vofa.c::vofa_send_frame) */
    if (huart2.gState != HAL_UART_STATE_READY)
        return;

    /* 写入 TX 缓冲区 */
    int len = snprintf(vofa_tx_buf, VOFA_BUF_SIZE,
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        (int)(adc_error      * 100),  /* CH0:  偏差 ×100 */
        (int)(jiaosudutobe   * 100),  /* CH1:  期望角速度 ×100 */
        (int)(jiaosudu       * 100),  /* CH2:  实际角速度 ×100 */
        (int)(jiaosuderror   * 100),  /* CH3:  角速度误差 ×100 */
        (int)(duty),                  /* CH4:  差速 */
        speed_,                       /* CH5:  目标速度 */
        speedl,                       /* CH6:  左轮速度 */
        speedr,                       /* CH7:  右轮速度 */
        pwm_,                         /* CH8:  PWM 输出 */
        run,                          /* CH9:  运行状态 */
        turn_count,                   /* CH10: 第几弯 */
        turn_forced_flag              /* CH11: 强制转向中 */
    );

    if (len > 0 && len < VOFA_BUF_SIZE)
    {
        /*
         * 中断模式发送 (参照 Main_board/vofa.c):
         *   HAL_UART_Transmit_IT 使能 UART TXE 中断, 每个中断发一字节,
         *   全部发完后触发 TC 中断 → HAL_UART_TxCpltCallback 被调用.
         *   不依赖 DMA, 没有 DMA 中断优先级被 TIM7 饿死的问题.
         */
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)vofa_tx_buf, len);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     运行时修改发送间隔
// 参数说明     interval_ms: 发送间隔, 单位 ms (最小 1, 建议 5~20)
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_set_interval(int interval_ms)
{
    if (interval_ms < 1) interval_ms = 100;
    if (interval_ms > 1000) interval_ms = 1000;
    vofa_interval = interval_ms;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART TX 完成回调 (由 HAL 在 TC 中断里调用)
// 备注信息     同时被 VOFA cmd 模块和本模块使用
//-------------------------------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        vofa_busy = 0;
    }
}