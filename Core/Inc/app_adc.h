#ifndef APP_ADC_H
#define APP_ADC_H

/*
 * app_adc.h
 * 14 路 SensorBoard ADC 模块头文件 + 转弯计数
 */

#include "main.h"

/* ---- 传感器通道数 ---- */
#define SENSOR_COUNT      14
#define SENSOR_HALF       7

/* ---- CS 控制宏 ---- */
#define ADC_CS_LOW()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET)
#define ADC_CS_HIGH()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET)

/* ---- 导出变量 ---- */
extern uint16_t adc_raw[SENSOR_COUNT];
extern float adc_test[SENSOR_COUNT];
extern float adc_beijing[SENSOR_COUNT];
extern float adc_xian[SENSOR_COUNT];
extern float adc_error;
extern float adc_error_l;
extern float adc_error_average[10];
extern int   if_bai[SENSOR_COUNT];
extern int   jiaozhun;
extern int   calibrated;       /* 1 = 已完成至少一次完整校准 */

/* ---- 转弯计数导出变量 (可接 VOFA+ 观测) ---- */
extern int   turn_count;           /* 当前第几弯 */
extern int   turn_forced_flag;     /* 1 = 正在强制转向中 */

/* ---- 函数声明 ---- */
void  app_adc_init(void);
void  app_adc_get(void);

#endif /* APP_ADC_H */