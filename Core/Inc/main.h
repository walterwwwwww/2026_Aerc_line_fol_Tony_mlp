/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM2_CH1_EC_Pin GPIO_PIN_0
#define TIM2_CH1_EC_GPIO_Port GPIOA
#define TIM2_DIR_L_Pin GPIO_PIN_1
#define TIM2_DIR_L_GPIO_Port GPIOA
#define TIM3_CH3_Buzzer_Pin GPIO_PIN_0
#define TIM3_CH3_Buzzer_GPIO_Port GPIOB
#define TIM3_CH4_FAN_Pin GPIO_PIN_1
#define TIM3_CH4_FAN_GPIO_Port GPIOB
#define USART3_TX_ULTRA_Pin GPIO_PIN_10
#define USART3_TX_ULTRA_GPIO_Port GPIOB
#define USART3_RX_ULTRA_Pin GPIO_PIN_11
#define USART3_RX_ULTRA_GPIO_Port GPIOB
#define SPI2_SCK_SENSOR_Pin GPIO_PIN_13
#define SPI2_SCK_SENSOR_GPIO_Port GPIOB
#define SPI2_MISO_SENSOR_Pin GPIO_PIN_14
#define SPI2_MISO_SENSOR_GPIO_Port GPIOB
#define SPI2_MOSI_SENSOR_Pin GPIO_PIN_15
#define SPI2_MOSI_SENSOR_GPIO_Port GPIOB
#define SPI2_CS1_SENSOR_Pin GPIO_PIN_8
#define SPI2_CS1_SENSOR_GPIO_Port GPIOD
#define SPI2_CS2_SENSOR_Pin GPIO_PIN_9
#define SPI2_CS2_SENSOR_GPIO_Port GPIOD
#define TIM4_CH1_EC_Pin GPIO_PIN_12
#define TIM4_CH1_EC_GPIO_Port GPIOD
#define TIM4_DIR_R_Pin GPIO_PIN_13
#define TIM4_DIR_R_GPIO_Port GPIOD
#define TIM1_CH1_PWM1_Pin GPIO_PIN_8
#define TIM1_CH1_PWM1_GPIO_Port GPIOA
#define TIM1_CH2_PWM2_Pin GPIO_PIN_9
#define TIM1_CH2_PWM2_GPIO_Port GPIOA
#define TIM1_CH3_PWM3_Pin GPIO_PIN_10
#define TIM1_CH3_PWM3_GPIO_Port GPIOA
#define TIM1_CH4_PWM4_Pin GPIO_PIN_11
#define TIM1_CH4_PWM4_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOC
#define BTN3_Pin GPIO_PIN_1
#define BTN3_GPIO_Port GPIOD
#define BTN2_Pin GPIO_PIN_2
#define BTN2_GPIO_Port GPIOD
#define BTN1_Pin GPIO_PIN_3
#define BTN1_GPIO_Port GPIOD
#define SPI1_CS_IMU_Pin GPIO_PIN_7
#define SPI1_CS_IMU_GPIO_Port GPIOD
#define SPI1_SCK_IMU_Pin GPIO_PIN_3
#define SPI1_SCK_IMU_GPIO_Port GPIOB
#define SPI1_MISO_IMU_Pin GPIO_PIN_4
#define SPI1_MISO_IMU_GPIO_Port GPIOB
#define SPI1_MOSI_IMU_Pin GPIO_PIN_5
#define SPI1_MOSI_IMU_GPIO_Port GPIOB
#define GPIO_EXTI_IMU_INT_Pin GPIO_PIN_6
#define GPIO_EXTI_IMU_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
