#include "app_common.h"

/*
 * app_vofa.c
 * VOFA+ FireWater 协议数据发送模块 (DMA 版本)
 *
 * 通过 USART2 (PD5=TX, 115200 baud) + DMA1_Stream6_Channel4 发送
 * DMA 发送完全不占用 CPU, TIM7 中断中只做 snprintf + 启动 DMA
 *
 * ⚠️ DMA 在 app_vofa_init() 中手动初始化, 不需要修改 CubeMX
 *
 * 传输带宽估算:
 *   10 通道整数, 每帧约 60~80 字节
 *   115200 baud ≈ 11520 字节/s → 最快约 150 帧/s
 *   默认 5ms = 200Hz ≈ 14000 字节/s, 占带宽 ~60%
 */

/* ---- DMA 句柄 ---- */
static DMA_HandleTypeDef hdma_usart2_tx;

/* ---- 双缓冲区 (乒乓) ---- */
/*
 * 为什么用双缓冲:
 *   DMA 正在发送 buf[0] 时, 中断中新数据写入 buf[1]
 *   下一帧发送 buf[1], 新数据写入 buf[0]
 *   这样 snprintf 和 DMA 永远不会操作同一块内存
 */
#define VOFA_BUF_SIZE  128
static char vofa_buf[2][VOFA_BUF_SIZE];
static volatile uint8_t vofa_buf_idx = 0;   /* 当前写入的缓冲区索引 (0 或 1) */

/* ---- 状态变量 ---- */
static volatile uint8_t vofa_dma_busy = 0;  /* DMA 正在发送标志 */
static int vofa_interval = VOFA_DEFAULT_INTERVAL;
static int vofa_cnt = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     手动初始化 USART2 TX DMA (DMA1_Stream6_Channel4)
// 备注信息     不需要修改 CubeMX, DMA1 时钟已在 MX_DMA_Init() 中使能
//              中断优先级设为 7 (低于 TIM7=5, 不影响控制)
//-------------------------------------------------------------------------------------------------------------------
static void vofa_dma_init(void)
{
    /* 配置 DMA1 Stream6 Channel4 → USART2_TX */
    hdma_usart2_tx.Instance                 = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_usart2_tx);

    /* 将 DMA 句柄链接到 USART2 */
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

    /* 使能 DMA1_Stream6 中断 (发送完成回调需要) */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA 模块初始化
// 备注信息     在 main.c 的 USER CODE BEGIN 2 中调用
//              必须在 MX_USART2_UART_Init() 和 MX_DMA_Init() 之后
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_init(void)
{
    vofa_cnt = 0;
    vofa_dma_busy = 0;
    vofa_buf_idx = 0;
    vofa_interval = VOFA_DEFAULT_INTERVAL;

    vofa_dma_init();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA 发送一帧 FireWater 数据到 VOFA+
// 备注信息     在 TIM7 中断尾部调用, 内部按 vofa_interval 分频
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
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_send(void)
{
    vofa_cnt++;
    if (vofa_cnt < vofa_interval)
        return;
    vofa_cnt = 0;

    /* 上一帧 DMA 还没发完, 跳过本帧 (不阻塞) */
    if (vofa_dma_busy)
        return;

    /* 写入当前缓冲区 */
    int len = snprintf(vofa_buf[vofa_buf_idx], VOFA_BUF_SIZE,
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
        vofa_dma_busy = 1;

        /* 启动 DMA 发送 (非阻塞, 立即返回) */
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)vofa_buf[vofa_buf_idx], len);

        /* 切换到另一个缓冲区, 下次写入不会和 DMA 冲突 */
        vofa_buf_idx ^= 1;
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

/* ============================ 中断处理 ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 Stream6 中断处理 (USART2 TX 发送完成)
// 备注信息     DMA1_Stream6 没有被 CubeMX 使用, 这里直接定义 IRQ Handler
//              HAL 库的弱符号机制保证不会和 CubeMX 生成的代码冲突
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART DMA 发送完成回调
// 备注信息     由 HAL 库在 DMA 传输完成后自动调用
//              清除 busy 标志, 允许下一帧发送
//              ⚠️ 这是弱符号函数, 如果其他文件也定义了会冲突
//                 如遇冲突, 将 vofa_dma_busy = 0 移到已有的回调中
//-------------------------------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        vofa_dma_busy = 0;
    }
}