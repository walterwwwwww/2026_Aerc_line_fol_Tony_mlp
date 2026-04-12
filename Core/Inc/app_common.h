#ifndef APP_COMMON_H
#define APP_COMMON_H

/*
 * app_common.h
 * 公共头文件 — 统一包含 HAL 与各模块头文件
 *
 * 移植说明:
 *   对应原始工程中 zf_common_headfile.h 的作用
 *   所有 app_*.c 文件通过 #include "app_common.h" 获取全局声明
 */

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* -------- 外设句柄 (CubeMX 自动生成, 声明在 main.c) -------- */
extern TIM_HandleTypeDef  htim1;   // 电机 PWM (PA8/PA9/PA10/PA11)
extern TIM_HandleTypeDef  htim2;   // 左编码器 外部计数 (PA0)
extern TIM_HandleTypeDef  htim3;   // 蜂鸣器(PB0) + 风扇(PB1)
extern TIM_HandleTypeDef  htim4;   // 右编码器 外部计数 (PD12)
extern TIM_HandleTypeDef  htim7;   // 1ms 定时中断
extern SPI_HandleTypeDef  hspi1;   // IMU (PB3/PB4/PB5, CS=PD7)
extern SPI_HandleTypeDef  hspi2;   // 14路传感器板 (PB13/PB14/PB15, CS1=PD8, CS2=PD9)
extern UART_HandleTypeDef huart2;  // 调试串口 (PD5/PD6)
extern UART_HandleTypeDef huart3;  // 超声波串口 (PB10/PB11)

/* -------- 各模块头文件 -------- */
#include "app_encoder.h"
#include "app_adc.h"
#include "app_motor.h"
#include "app_gyro.h"
#include "app_button.h"
#include "app_control.h"

#endif /* APP_COMMON_H */
