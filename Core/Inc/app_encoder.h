#ifndef APP_ENCODER_H
#define APP_ENCODER_H

/*
 * app_encoder.h
 * 编码器模块头文件
 *
 * 硬件映射:
 *   TIM2 (PA0 脉冲, PA1 方向GPIO) → 左编码器  ← 原 TC_CH19_ENCODER (P08_0, P08_1)
 *   TIM4 (PD12 脉冲, PD13 方向GPIO) → 右编码器 ← 原 TC_CH07_ENCODER (P02_0, P02_1)
 *
 * 调用频率: app_encoder_get() 每 1ms 由 TIM7 中断调用
 * 输出更新: speedl / speedr 每 10ms 更新一次 (10 次采样取平均)
 */

#include "main.h"

extern int speedl;   // 左轮速度 (脉冲数/ms 的 10ms 平均, 正=前进)
extern int speedr;   // 右轮速度

void app_encoder_init(void);
void app_encoder_get(void);

#endif /* APP_ENCODER_H */
