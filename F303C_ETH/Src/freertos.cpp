/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sensor.h"
#include "flash_spi.h"
#include "LED.h"
using namespace std;
#include <string>
#include <iostream>
#include <vector>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct mesage_t{
	uint32_t cmd;
	uint32_t addres_var;
	uint32_t data_in;
	bool need_resp;
	bool data_in_is;
	uint32_t data_out;
	string err; // сообщение клиенту об ошибке в сообщении
	bool f_bool; // наличие ошибки в сообшении
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc1;

extern settings_t settings;
uint16_t sensBuff[8] = {0};
uint8_t sensState = 255; // битовое поле

uint32_t freqSens = HAL_RCC_GetHCLKFreq()/30000u;
uint32_t pwmSens;

uint16_t adc_buffer[1024] = {0};
sensor Sensor1;
sensor Sensor2;
sensor Sensor3;
uint16_t call = 0;

extern led LED_IPadr;
extern led LED_error;
extern led LED_OSstart;

// for SPI Flash
extern SPI_HandleTypeDef hspi2;
pins_spi_t ChipSelect = {CS_GPIO_Port, CS_Pin};
pins_spi_t WriteProtect = {WP_GPIO_Port, WP_Pin};
pins_spi_t Hold = {HOLD_GPIO_Port, HOLD_Pin};

flash mem_spi;

bool resetSettings = false;
uint32_t Start = 0;
/* USER CODE END Variables */
osThreadId MainTaskHandle;
osThreadId LEDHandle;
osSemaphoreId ADC_endHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void mainTask(void const * argument);
void led(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
extern "C" void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ADC_end */
  osSemaphoreDef(ADC_end);
  ADC_endHandle = osSemaphoreCreate(osSemaphore(ADC_end), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MainTask */
  osThreadDef(MainTask, mainTask, osPriorityNormal, 0, 256);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, led, osPriorityNormal, 0, 256);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_mainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainTask */
void mainTask(void const * argument)
{
  /* USER CODE BEGIN mainTask */
	mem_spi.Init(&hspi2, 0, ChipSelect, WriteProtect, Hold);

	mem_spi.Read(&settings);
	if((settings.BaudRate == 0) | (settings.BaudRate == 0xFFFFFFFF) | resetSettings)
	{
	      settings.BaudRate = 115200;
	      settings.SlaveAddress = 0x02;
	      settings.offsetMax = 0;
	      settings.offsetMin = 0;
	      settings.timeCall = 3000;
		mem_spi.Write(settings);
	}

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buffer, 1);
	HAL_TIM_Base_Start_IT(&htim3);
	Sensor1.setTimeCall(settings.timeCall);

	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  for(;;)
  {
      osSemaphoreWait(ADC_endHandle, osWaitForever);
      Sensor1.Filter_SMA(adc_buffer[0]);
      //Sensor2.Filter_SMA(adc_buffer[1]);
      //Sensor3.Filter_SMA(adc_buffer[2]);
      //printf("CH1: %d\r\n",Sensor1.Get_Result());
      //printf("CH2: %d\r\n",Sensor2.Get_Result());
      //printf("CH3: %d\r\n",Sensor3.Get_Result());

      if(call){
         call = 0;
         HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
         Sensor1.Call(&adc_buffer[0]);
         //Flash_Write(settings, StartSettingsAddres);
         HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
      }
      if(Sensor1.detectPoll()){
         HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET);
      }else{
         HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
      }
      //taskYIELD();
  }
  /* USER CODE END mainTask */
}

/* USER CODE BEGIN Header_led */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led */
void led(void const * argument)
{
  /* USER CODE BEGIN led */
  /* Infinite loop */

	LED_IPadr.Init(G_GPIO_Port, G_Pin);
	LED_error.Init(R_GPIO_Port, R_Pin);
	LED_OSstart.Init(B_GPIO_Port, B_Pin);
	LED_OSstart.setParameters(mode::BLINK, 2000, 100);
	LED_OSstart.LEDon();

	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for(;;)
	{
		LED_IPadr.poll();
		LED_error.poll();
		LED_OSstart.poll();

		if(Start == 1){
			Start = 0;
			//pMotor->stop();
		}
		if(Start == 2){
			Start = 0;
			//pMotor->deceleration();

		}
		if(Start == 3){
			Start = 0;
			//pMotor->start();

		}
		if(Start == 4){
			Start = 0;
			//pMotor->SetDirection(dir::CW);
		}
		if(Start == 5){
			Start = 0;
			//pMotor->SetDirection(dir::CCW);
		}

		osDelay(1);
		//taskYIELD();
		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
	}
  /* USER CODE END led */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  /* This is called after the conversion is completed */

   osSemaphoreRelease(ADC_endHandle);

}
/* USER CODE END Application */

