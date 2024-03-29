/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_spi.h"
#include "Delay_us_DWT.h"
#include "LED.h"
#include "sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
settings_t settings = {0, 0x0E};
//HCSR04Driver hcsr04Driver;
sensor Sensor1;
sensor Sensor2;
SensorType sensorType = NoInit;

uint32_t count_tic = 0; //для замеров времени выполнения кода
extern TIM_HandleTypeDef htim3;

led LED_IPadr;
led LED_error;
led LED_OSstart;

bool resetSettings = false;

// for SPI Flash
extern SPI_HandleTypeDef hspi3;
pins_spi_t ChipSelect = {SPI3_CS_GPIO_Port, SPI3_CS_Pin};
pins_spi_t WriteProtect = {WP_GPIO_Port, WP_Pin};
pins_spi_t Hold = {HOLD_GPIO_Port, HOLD_Pin};
flash mem_spi;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_SPI3_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(eth_NRST_GPIO_Port, eth_NRST_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

	mem_spi.Init(&hspi3, 0, ChipSelect, WriteProtect, Hold);

	mem_spi.Read(&settings);

	if ((settings.MAC_end == 0) | (settings.MAC_end == 0xFFFFFFFF) | resetSettings)
	{
		mem_spi.W25qxx_EraseSector(0);
		uint32_t s = 1;
		// for sensor 1
		settings.sensorSett[s].callDistanceMax = 4096;
		settings.sensorSett[s].callDistanceMin = 0;

		settings.sensorSett[s].timeParametrs[1].callTimeMax = 0xFFFFFFFF;
		settings.sensorSett[s].timeParametrs[1].callTimeMin = 0;
		settings.sensorSett[s].timeParametrs[1].timOutFalling = 10;

		settings.sensorSett[s].timeParametrs[2].callTimeMax = 0xFFFFFFFF;
		settings.sensorSett[s].timeParametrs[2].callTimeMin = 0;
		settings.sensorSett[s].timeParametrs[2].timOutFalling = 10;

		settings.sensorSett[s].timeParametrs[3].callTimeMax = 0xFFFFFFFF;
		settings.sensorSett[s].timeParametrs[3].callTimeMin = 0;
		settings.sensorSett[s].timeParametrs[3].timOutFalling = 10;

		settings.sensorSett[s].chanelCallTime = 1;
		settings.sensorSett[s].k_H = 0.1;
		settings.sensorSett[s].k_L = 0.03;
		settings.sensorSett[s].modePwr = 2;
		settings.sensorSett[s].offsetTime = 0;
		settings.sensorSett[s].sensorType = Optic;
		settings.sensorSett[s].timeCall = 5000;
		settings.sensorSett[s].triger = 100;

		// for sensor 1
		s = 2;
		settings.sensorSett[s].callDistanceMax = 4096;
		settings.sensorSett[s].callDistanceMin = 0;

		settings.sensorSett[s].timeParametrs[1].callTimeMax = 0xFFFFFFFF;
		settings.sensorSett[s].timeParametrs[1].callTimeMin = 0;
		settings.sensorSett[s].timeParametrs[1].timOutFalling = 10;

		settings.sensorSett[s].timeParametrs[2].callTimeMax = 0xFFFFFFFF;
		settings.sensorSett[s].timeParametrs[2].callTimeMin = 0;
		settings.sensorSett[s].timeParametrs[2].timOutFalling = 10;

		settings.sensorSett[s].timeParametrs[3].callTimeMax = 0xFFFFFFFF;
		settings.sensorSett[s].timeParametrs[3].callTimeMin = 0;
		settings.sensorSett[s].timeParametrs[3].timOutFalling = 10;

		settings.sensorSett[s].chanelCallTime = 1;
		settings.sensorSett[s].k_H = 0.1;
		settings.sensorSett[s].k_L = 0.03;
		settings.sensorSett[s].modePwr = 2;
		settings.sensorSett[s].offsetTime = 0;
		settings.sensorSett[s].sensorType = Optic;
		settings.sensorSett[s].timeCall = 5000;
		settings.sensorSett[s].triger = 100;

		settings.MAC_end = 0x05;

		mem_spi.Write(settings);

		mem_spi.Read(&settings);
	}
	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 6;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM7) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM4) {
		Sensor2._acknowledgeTimerUpdate();
	}
	if (htim->Instance == TIM3) {
		Sensor1._acknowledgeTimerUpdate();
	}
	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
