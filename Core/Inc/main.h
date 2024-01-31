/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define CHG_QON_Pin GPIO_PIN_13
#define CHG_QON_GPIO_Port GPIOC
#define CHG_IIN_Pin GPIO_PIN_0
#define CHG_IIN_GPIO_Port GPIOA
#define NTC_V_Pin GPIO_PIN_2
#define NTC_V_GPIO_Port GPIOA
#define HEAT_CUR_Pin GPIO_PIN_3
#define HEAT_CUR_GPIO_Port GPIOA
#define PWR_SENSE_Pin GPIO_PIN_4
#define PWR_SENSE_GPIO_Port GPIOA
#define WS2812B_Pin GPIO_PIN_6
#define WS2812B_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOB
#define HEAT_CTRL_Pin GPIO_PIN_1
#define HEAT_CTRL_GPIO_Port GPIOB
#define SW_CNT_Pin GPIO_PIN_2
#define SW_CNT_GPIO_Port GPIOB
#define SW_CNT_EXTI_IRQn EXTI2_3_IRQn
#define EYE_SCL_Pin GPIO_PIN_10
#define EYE_SCL_GPIO_Port GPIOB
#define EYE_SDA_Pin GPIO_PIN_11
#define EYE_SDA_GPIO_Port GPIOB
#define HX711_SCK_Pin GPIO_PIN_13
#define HX711_SCK_GPIO_Port GPIOB
#define HX711_DOUT_Pin GPIO_PIN_14
#define HX711_DOUT_GPIO_Port GPIOB
#define EE_SCL_Pin GPIO_PIN_15
#define EE_SCL_GPIO_Port GPIOB
#define EE_SDA_Pin GPIO_PIN_8
#define EE_SDA_GPIO_Port GPIOA
#define EE_WP_Pin GPIO_PIN_9
#define EE_WP_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOC
#define SCREEN_Pin GPIO_PIN_10
#define SCREEN_GPIO_Port GPIOA
#define TMC_SDO_Pin GPIO_PIN_11
#define TMC_SDO_GPIO_Port GPIOA
#define TMC_SDI_Pin GPIO_PIN_12
#define TMC_SDI_GPIO_Port GPIOA
#define TMC_CSN_Pin GPIO_PIN_15
#define TMC_CSN_GPIO_Port GPIOA
#define TMC_ENN_Pin GPIO_PIN_0
#define TMC_ENN_GPIO_Port GPIOD
#define CHG_DSEL_Pin GPIO_PIN_1
#define CHG_DSEL_GPIO_Port GPIOD
#define CHG_DSEL_EXTI_IRQn EXTI0_1_IRQn
#define CHG_GPOUT_Pin GPIO_PIN_2
#define CHG_GPOUT_GPIO_Port GPIOD
#define CHG_CE_Pin GPIO_PIN_3
#define CHG_CE_GPIO_Port GPIOD
#define TMC_SCK_Pin GPIO_PIN_3
#define TMC_SCK_GPIO_Port GPIOB
#define CHG_STAT_Pin GPIO_PIN_4
#define CHG_STAT_GPIO_Port GPIOB
#define CHG_STAT_EXTI_IRQn EXTI4_15_IRQn
#define CHG_INT_Pin GPIO_PIN_5
#define CHG_INT_GPIO_Port GPIOB
#define CHG_INT_EXTI_IRQn EXTI4_15_IRQn
#define CHG_SCL_Pin GPIO_PIN_8
#define CHG_SCL_GPIO_Port GPIOB
#define CHG_SDA_Pin GPIO_PIN_9
#define CHG_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tmc5130.h"
#include "queue.h"
#include "Screen.h"
#include "stdint.h"
#include "usart.h"
#include <stdio.h>
#include "event_groups.h"
#include "stm32g0xx_hal.h"
#include "pid.h"
#include "tmp114.h"
#include "ucs1903.h"
#include "hx711.h"
#include "bq27441.h"
#include "bq25895.h"
#include "24cxx.h"

#define Heat_BIT_0 (1 << 0)
#define SW_BIT_1 (1 << 1)
#define Motor_BIT_2 (1 <<2)
#define Auto_BIT_3 (1 <<3)
#define Reset_Motor_BIT_4 (1 <<4)

#define TMP114Address		0x92//xin
//#define TMP114Address		0x90//lao
//#include "retarget.h"


#define BQ25895Address	0xd4
#define BQ27441Address	0xaa








typedef struct {
    uint8_t buffer[20];   // æŒ‡å�‘æ�?�°æ�®çš„æŒ�?�ï�???
    uint16_t length;  // æ•°æ�®é•¿åº�?
} uart_data;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
