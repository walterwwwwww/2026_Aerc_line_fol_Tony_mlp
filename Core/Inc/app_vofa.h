#ifndef APP_VOFA_H
#define APP_VOFA_H

/*
 * app_vofa.h
 * VOFA+ FireWater 协议数据发送模块 (DMA 版本)
 *
 * 功能: 通过 USART2 (PD5=TX, 115200) 以 FireWater 格式发送 PID 调试数据
 *       使用 DMA1_Stream6_Channel4 发送, 不占用 TIM7 中断时间
 *
 * FireWater 协议:
 *   每帧格式: "数据1,数据2,数据3,...,数据N\n"
 *   纯文本逗号分隔, 换行符结尾, 全部整数
 *
 * VOFA+ 通道分配 (共 10 通道, 全部整数):
 *   CH0: adc_error ×100    — 传感器偏差 (外环输入)
 *   CH1: jiaosudutobe ×100 — 期望角速度 (外环输出 / 中环目标)
 *   CH2: jiaosudu ×100     — 实际角速度 (中环反馈)
 *   CH3: jiaosuderror ×100 — 角速度误差 (中环误差)
 *   CH4: duty              — 差速输出 (中环输出, 不缩放)
 *   CH5: speed_            — 目标速度 (内环目标)
 *   CH6: speedl            — 左轮实际速度 (内环反馈)
 *   CH7: speedr            — 右轮实际速度 (内环反馈)
 *   CH8: pwm_              — 速度环 PWM 输出 (内环输出)
 *   CH9: run               — 运行状态标志
 *
 * 使用方法:
 *   1) main.c 初始化中调用 app_vofa_init()
 *   2) TIM7 中断中调用 app_vofa_send()
 *   3) VOFA+ 端: 选 FireWater 协议, 串口 115200 8N1
 */

#include "main.h"

/* ---- 发送间隔 (ms) ---- */
#define VOFA_DEFAULT_INTERVAL   5   /* 默认每 5ms 发一帧 (200Hz) */

/* ---- 函数声明 ---- */
void app_vofa_init(void);                       /* 初始化 DMA + 清零计数器 */
void app_vofa_send(void);                       /* 在 TIM7 中断中调用, 内部自动分频 */
void app_vofa_set_interval(int interval_ms);    /* 运行时修改发送间隔 */

#endif /* APP_VOFA_H */