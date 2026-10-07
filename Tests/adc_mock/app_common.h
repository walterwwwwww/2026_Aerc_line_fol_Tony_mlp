/* Host-only peripheral doubles for exercising the actual app_adc.c state machine. */
#ifndef ADC_TEST_COMMON_H
#define ADC_TEST_COMMON_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "app_sensor_weights.h"
#define SENSOR_COUNT 14
#define SENSOR_HALF 7
#define TURN_LOOKUP_SLOT_COUNT 5
#define ADC_CS_HIGH() ((void)0)
#define ADC_CS_LOW() ((void)0)
#define __NOP() ((void)0)
#define __DMB() ((void)0)
extern uint32_t mock_irq_mask;
static inline uint32_t __get_PRIMASK(void) {return mock_irq_mask;}
static inline void __disable_irq(void) {mock_irq_mask=1;}
static inline void __set_PRIMASK(uint32_t value) {mock_irq_mask=value;}
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
extern int hspi2;
static inline int HAL_SPI_Abort(void *p) {(void)p;return 0;}
static inline int HAL_SPI_DeInit(void *p) {(void)p;return 0;}
static inline int HAL_SPI_Transmit(void *p,uint8_t *data,uint16_t n,uint32_t timeout)
{(void)p;(void)data;(void)n;(void)timeout;return 1;}
static inline int HAL_SPI_Receive(void *p,uint8_t *data,uint16_t n,uint32_t timeout)
{(void)p;(void)data;(void)n;(void)timeout;return 1;}
static inline void app_LED_set(int pin,int value) {(void)pin;(void)value;}
extern int run;
extern int anniu_zhuangtai[3][2],duty,chusaidaojuli,chusaidao;
extern float jiaoduz,jiaosuderror,jiaosuderrorl,jiaosudutobe;
#endif
