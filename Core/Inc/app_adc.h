#ifndef APP_ADC_H
#define APP_ADC_H

/*
 * app_adc.h
 * 14 路 SensorBoard ADC 模块头文件
 *
 * 硬件映射:
 *   SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI)
 *   CS = PD8 (软件控制)
 *
 * 通信协议:
 *   Master 发送 uint16_t 2000 → Slave 回传 14 × uint16_t
 */

#include "main.h"

/* ---- 传感器通道数 ---- */
#define SENSOR_COUNT      14
#define SENSOR_HALF       7

/* ---- CS 控制宏 ---- */
#define ADC_CS_LOW()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET)
#define ADC_CS_HIGH()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET)

/* ---- 导出变量 ---- */
extern uint16_t adc_raw[SENSOR_COUNT];      /* SPI 原始值, 调试用 */
extern float adc_test[SENSOR_COUNT];
extern float adc_beijing[SENSOR_COUNT];     /* 校准背景值 (min), 调试用 */
extern float adc_xian[SENSOR_COUNT];        /* 校准线上值 (max), 调试用 */
extern float adc_error;
extern float adc_error_l;
extern float adc_error_average[10];
extern int   if_bai[SENSOR_COUNT];
extern int   jiaozhun;

/* ---- 函数声明 ---- */
void  app_adc_init(void);
void  app_adc_get(void);

#endif /* APP_ADC_H */