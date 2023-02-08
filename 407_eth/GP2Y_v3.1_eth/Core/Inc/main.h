/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	Optic,
	Ultrasound,
	NoInit,
}SensorType;

typedef struct
{
	uint32_t callTimeMin; 				//  = 0xFFFFFFFF минимальное время прохождения ребра измеренное при каллибровке
	uint32_t callTimeMax;  				//  = 0 максимальное время прохождения ребра измеренное при каллибровке
	uint32_t timOutFalling; 			// = 10
}callTime_t;

typedef struct
{
	uint32_t offsetTime;
	uint16_t callDistanceMin;			//  = 0 зона работы датчика
	uint16_t callDistanceMax;			//  = 4096 зона работы датчика
	callTime_t timeParametrs[4];		// массив с разными каллибровками времени для разных скоростей
	uint16_t chanelCallTime;			//  = 0 текущий канал каллибровки
	uint16_t triger;					//  = 100 смещение от ленты
	uint32_t timeCall;					//  = 5000 время выполнение калибровки
	uint16_t modePwr;					// режим работы питания
	float k_H;							// = 0.1 коэфицент фильтра для быстрых изменений ( начальное 0.9)
	float k_L;							// = 0.03 коэфицент фильтра для медленных изменений
	SensorType sensorType;				// = NoInit
}sensorSett_t;

typedef struct
{
	uint8_t	MAC_end;
	sensorSett_t sensorSett[2];
}settings_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ID_STRING " GP2Y_v3.1_eth_09.01.23"
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define R_Pin GPIO_PIN_13
#define R_GPIO_Port GPIOC
#define G_Pin GPIO_PIN_14
#define G_GPIO_Port GPIOC
#define B_Pin GPIO_PIN_15
#define B_GPIO_Port GPIOC
#define eth_NRST_Pin GPIO_PIN_0
#define eth_NRST_GPIO_Port GPIOA
#define TRIG2_Pin GPIO_PIN_12
#define TRIG2_GPIO_Port GPIOD
#define ECHO2_Pin GPIO_PIN_13
#define ECHO2_GPIO_Port GPIOD
#define TRIG1_Pin GPIO_PIN_6
#define TRIG1_GPIO_Port GPIOC
#define ECHO1_Pin GPIO_PIN_7
#define ECHO1_GPIO_Port GPIOC
#define pwr1_Pin GPIO_PIN_11
#define pwr1_GPIO_Port GPIOA
#define pwr2_Pin GPIO_PIN_12
#define pwr2_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define WP_Pin GPIO_PIN_0
#define WP_GPIO_Port GPIOD
#define HOLD_Pin GPIO_PIN_1
#define HOLD_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
