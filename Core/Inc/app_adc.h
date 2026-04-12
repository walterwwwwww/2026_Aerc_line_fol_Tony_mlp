#ifndef APP_ADC_H
#define APP_ADC_H

/*
 * app_adc.h
 * 14 路 SensorBoard ADC 模块头文件
 *
 * 硬件映射:
 *   SPI2 (PB13/PB14/PB15)
 *     CS1 = PD8 → 传感器芯片 1 (通道 0~6,  物理左侧 7 路)
 *     CS2 = PD9 → 传感器芯片 2 (通道 7~13, 物理右侧 7 路)
 *
 * 与原始代码差异:
 *   原始: 16 路 (8+8, 通过 SPI_1 读取外部 ADC)
 *   现在: 14 路 (7+7, 通过 SPI2 + 双 CS)
 *   上层接口保持不变: adc_error 输出相同含义的偏差值
 *
 * 传感器物理布局 (俯视, 行进方向 ↑):
 *   [0] [1] [2] [3] [4] [5] [6] | [7] [8] [9] [10] [11] [12] [13]
 *   ←——————— 左侧 ———————→   ←——————— 右侧 ————————→
 *   对称配对: (0,13) (1,12) (2,11) (3,10) (4,9) (5,8) (6,7)
 */

#include "main.h"

/* ---- 传感器通道数 ---- */
#define SENSOR_COUNT      14
#define SENSOR_HALF       7

/* ---- 导出变量 (保持与原始接口一致) ---- */
extern float adc_test[SENSOR_COUNT];      // 归一化后传感器值 (0~10)
extern float adc_error;                   // 巡线偏差 (正=偏右, 负=偏左)
extern float adc_error_l;                 // 上一次偏差 (用于微分)
extern float adc_error_average[10];       // 偏差滑动平均缓冲区
extern int   if_bai[SENSOR_COUNT];        // 二值化标志 (1=白/无线, 0=黑/有线)
extern int   jiaozhun;                    // 校准模式标志

/* ---- 函数声明 ---- */
void  app_adc_init(void);                // SPI2 传感器板初始化
void  app_adc_get(void);                 // 读取 + 归一化 + 偏差计算 (1ms 调用)

#endif /* APP_ADC_H */
