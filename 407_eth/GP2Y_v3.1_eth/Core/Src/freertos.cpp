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
#include "lwip.h"
using namespace std;
#include <string>
#include "api.h"
#include <iostream>
#include <vector>
#include "hcsr04_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct mesage_t{
	uint32_t cmd;
	uint32_t addres_var;
	uint32_t data_in;
	bool need_resp = false;
	bool data_in_is;
	uint32_t data_out;
	string err; // сообщение клиенту об ошибке в сообщении
	bool f_bool = false; // наличие ошибки в сообшении
};

struct debugSensor{
	uint32_t time;
	uint16_t dada[16];
	bool detect = false;
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
extern TIM_HandleTypeDef htim4;

extern settings_t settings;
uint16_t sensBuff[8] = {0};
uint8_t sensState = 255; // битовое поле

uint32_t freqSens = HAL_RCC_GetHCLKFreq()/30000u;
uint32_t pwmSens;

uint16_t adc_buffer[24] = {0};
uint16_t adc_buffer2[24] = {0};
extern sensor Sensor1;
extern sensor Sensor2;
//sensor Sensor3;
//extern HCSR04Driver hcsr04Driver;
//bool ultrasens2 = false;
float distance_ul_S2 = 0.0, distance_ul_S1 = 0.0;
uint16_t call1 = 0, call2 = 0;

extern led LED_IPadr;
extern led LED_error;
extern led LED_OSstart;

//переменные для отладки
vector<debugSensor> debugBuf;
uint32_t debug_I = 0;
bool debug_send = false;

//структуры для netcon
extern struct netif gnetif;

//TCP_IP
string strIP;
string in_str;

//переменные для обшей работы
uint32_t Start = 0;

extern flash mem_spi;

/* USER CODE END Variables */
osThreadId MainTaskHandle;
osThreadId LEDHandle;
osThreadId ethTasHandle;
osThreadId MainTask2Handle;
osThreadId debug_udpHandle;
osMutexId s2DistanceMutexHandle;
osMutexId mutexADCHandle;
osMutexId s1DistanceMutexHandle;
osSemaphoreId ADC_endHandle;
osSemaphoreId ADC_end2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void mainTask(void const * argument);
void led(void const * argument);
void eth_Task(void const * argument);
void mainTask2(void const * argument);
void Debug_udp(void const * argument);

extern void MX_LWIP_Init(void);
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
  /* Create the mutex(es) */
  /* definition and creation of s2DistanceMutex */
  osMutexDef(s2DistanceMutex);
  s2DistanceMutexHandle = osMutexCreate(osMutex(s2DistanceMutex));

  /* definition and creation of mutexADC */
  osMutexDef(mutexADC);
  mutexADCHandle = osMutexCreate(osMutex(mutexADC));

  /* definition and creation of s1DistanceMutex */
  osMutexDef(s1DistanceMutex);
  s1DistanceMutexHandle = osMutexCreate(osMutex(s1DistanceMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ADC_end */
  osSemaphoreDef(ADC_end);
  ADC_endHandle = osSemaphoreCreate(osSemaphore(ADC_end), 1);

  /* definition and creation of ADC_end2 */
  osSemaphoreDef(ADC_end2);
  ADC_end2Handle = osSemaphoreCreate(osSemaphore(ADC_end2), 1);

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
  osThreadDef(LED, led, osPriorityNormal, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of ethTas */
  osThreadDef(ethTas, eth_Task, osPriorityNormal, 0, 512);
  ethTasHandle = osThreadCreate(osThread(ethTas), NULL);

  /* definition and creation of MainTask2 */
  osThreadDef(MainTask2, mainTask2, osPriorityNormal, 0, 256);
  MainTask2Handle = osThreadCreate(osThread(MainTask2), NULL);

  /* definition and creation of debug_udp */
  osThreadDef(debug_udp, Debug_udp, osPriorityNormal, 0, 512);
  debug_udpHandle = osThreadCreate(osThread(debug_udp), NULL);

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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN mainTask */

	float temp_distance_ul = 0.0;

	switch (settings.sensorType1) {
	case 1: // оптика
		Sensor1.Init(&ADC_endHandle, &hadc1, adc_buffer, pwr1_GPIO_Port, pwr1_Pin, 1);
		Sensor1.SetTimeCall(settings.timeCall1);
		break;
	case 2: // ултразвук
		Sensor1.Init(TIM3,TIM_CHANNEL_1 ,TIM_CHANNEL_2, pwr1_GPIO_Port, pwr1_Pin, 1);

		break;
	default:
		//Error_Handler();
		break;
	}

	//переменные для сбора данных
	static uint32_t prevTime = HAL_GetTick();
	static uint32_t GTime = 0;
	debugSensor tempUnit;

	/* Infinite loop */
	for(;;)
	{
		switch (settings.sensorType1) {
		case 1: // оптика
			switch (call1) {
			case 1: // каллибровка дистанции
				call1 = 0;
				//взять мютекс
				osMutexWait(mutexADCHandle, osWaitForever);

				//HAL_GPIO_WritePin(pwr1_GPIO_Port, pwr1_Pin, GPIO_PIN_SET); // питение
				Sensor1.PwrSet(2); // pwr on
				LED_error.LEDon();
				osDelay(300);
				Sensor1.CallDistance();
				LED_error.LEDoff();
				Sensor1.PwrSet(3); // pwr off
				//HAL_GPIO_WritePin(pwr1_GPIO_Port, pwr1_Pin, GPIO_PIN_RESET); // питение

				//вернуть мютекс
				osMutexRelease(mutexADCHandle);
				break;
			case 2:// каллибровка времени
				call1 = 0;
				//взять мютекс
				osMutexWait(mutexADCHandle, osWaitForever);

				//HAL_GPIO_WritePin(pwr1_GPIO_Port, pwr1_Pin, GPIO_PIN_SET); // питение
				Sensor1.PwrSet(2); // pwr on
				LED_error.LEDon();
				osDelay(300);
				if(Sensor1.CallTime()){
					LED_error.LEDoff();
					LED_error.LEDon(10);
				}
				LED_error.LEDoff();
				Sensor1.PwrSet(3); // pwr off
				//HAL_GPIO_WritePin(pwr1_GPIO_Port, pwr1_Pin, GPIO_PIN_RESET); // питение

				//вернуть мютекс
				osMutexRelease(mutexADCHandle);
				break;
			default:
				//взять мютекс
				osMutexWait(mutexADCHandle, osWaitForever);
				//запустить ацп
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buffer, Sensor1.Depth);
				//подождать симафор от АЦП
				osSemaphoreWait(ADC_endHandle, osWaitForever);
				//вернуть мютекс
				osMutexRelease(mutexADCHandle);
				// обработать данные
				Sensor1.DataProcessing(adc_buffer);
				//start debug
				if(debug_I <= 100){
					debug_I++;
					tempUnit.time = GTime += ((HAL_GetTick()) - prevTime);
					prevTime = HAL_GetTick();
					tempUnit.dada[0] = Sensor1.GetResult();
					tempUnit.detect = Sensor1.Getdetect();
					debugBuf.push_back(tempUnit);
				}else{
					if(debug_send){
						debug_send = false;
						debugBuf.clear();
						debug_I = 0;
					}
				}
				//end debug

				if(Sensor1.DetectPoll()){
					LED_error.LEDon();
				}else{
					LED_error.LEDoff();
				}
				break;
			}

			break;
			case 2: // ултразвук
				//HAL_GPIO_WritePin(pwr2_GPIO_Port, pwr2_Pin, GPIO_PIN_SET);
				xSemaphoreTake(s1DistanceMutexHandle, 100);
				temp_distance_ul = Sensor1.GetDistance();
				if(temp_distance_ul < 0) {
					distance_ul_S2 = 0;
				}else{
					distance_ul_S1 = temp_distance_ul;
				}
				xSemaphoreGive(s1DistanceMutexHandle);

				osDelay(100);

				break;
			default:
				//Error_Handler();
				break;
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
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);

	LED_IPadr.Init(G_GPIO_Port, G_Pin);
	LED_error.Init(R_GPIO_Port, R_Pin);
	LED_OSstart.Init(B_GPIO_Port, B_Pin);

	LED_IPadr.setParameters(mode::ON_OFF);
	LED_error.setParameters(mode::ON_OFF);
	LED_OSstart.setParameters(mode::BLINK, 2000, 100);
	LED_OSstart.LEDon();

	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for(;;)
	{
		LED_IPadr.poll();
		LED_error.poll();
		LED_OSstart.poll();

		if(Start == 1){
			Start = 0;
			Sensor1.PwrSet(2);
		}
		if(Start == 2){
			Start = 0;
			Sensor2.PwrSet(2);

		}
		if(Start == 3){
			Start = 0;
			Sensor1.PwrSet(3);

		}
		if(Start == 4){
			Start = 0;
			Sensor2.PwrSet(3);
		}
		if(Start == 5){
			Start = 0;
			//pMotor->SetDirection(dir::CCW);
			call1 = 1;
		}
		if(Start == 6){
			Start = 0;
			//pMotor->SetDirection(dir::CCW);
			call2 = 1;
		}
		if(Start == 7){
			Start = 0;
			//pMotor->SetDirection(dir::CCW);
			call1 = 1;
			call2 = 1;
		}
		osDelay(1);
		//taskYIELD();
		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
	}
  /* USER CODE END led */
}

/* USER CODE BEGIN Header_eth_Task */
/**
 * @brief Function implementing the ethTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_eth_Task */
void eth_Task(void const * argument)
{
  /* USER CODE BEGIN eth_Task */


	while(gnetif.ip_addr.addr == 0){osDelay(1);}	//ждем получение адреса
	LED_IPadr.LEDon();
	osDelay(1000);
	LED_IPadr.LEDoff();
	strIP = ip4addr_ntoa(&gnetif.ip_addr);

	//структуры для netcon
	struct netconn *conn;
	struct netconn *newconn;
	struct netbuf *netbuf;
	volatile err_t err, accept_err;
	//ip_addr_t local_ip;
	//ip_addr_t remote_ip;
	void 		*in_data = NULL;
	uint16_t 		data_size = 0;

	//Флаги для разбора сообщения
	string f_cmd("C");
	string f_addr("A");
	string f_datd("D");
	string delim("x");

	/* Infinite loop */
	for(;;)
	{

		conn = netconn_new(NETCONN_TCP);
		if (conn!=NULL)
		{
			err = netconn_bind(conn,NULL,81);//assign port number to connection
			if (err==ERR_OK)
			{
				netconn_listen(conn);//set port to listening mode
				while(1)
				{
					accept_err=netconn_accept(conn,&newconn);//suspend until new connection
					if (accept_err==ERR_OK)
					{
						//LED_IPadr.LEDon();
						while ((accept_err=netconn_recv(newconn,&netbuf))==ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
						{

							do
							{
								netbuf_data(netbuf,&in_data,&data_size);//get pointer and data size of the buffer
								in_str.assign((char*)in_data, data_size);//copy in string
								/*-----------------------------------------------------------------------------------------------------------------------------*/
								// Парсинг
								vector<string> arr_msg;
								vector<mesage_t> arr_cmd;
								size_t prev = 0;
								size_t next;
								size_t delta = delim.length();

								//разбить на сообщения
								while( ( next = in_str.find( delim, prev ) ) != string::npos ){
									arr_msg.push_back( in_str.substr( prev, (next +1)-prev ) );
									prev = next + delta;
								}
								//arr_msg.push_back( in_str.substr( prev ) );

								//занести сообщения в структуру
								int count_msg = arr_msg.size();
								for (int i = 0; i < count_msg; ++i) {
									prev = 0;
									next = 0;
									size_t posC = 0;
									//size_t posA = 0;
									size_t posD = 0;
									size_t posx = 0;
									mesage_t temp_msg;

									// выделение комманды
									delta = f_cmd.length();
									next = arr_msg[i].find(f_cmd);
									posC = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in C flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;

									}
									prev = next + delta;
									/*
									// выделение адреса
									delta = f_addr.length();
									next = arr_msg[i].find(f_addr, prev);
									posA = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in A flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;
									}
									prev = next + delta;
									 */
									// выделение данных
									delta = f_datd.length();
									next = arr_msg[i].find(f_datd, prev);
									posD = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in D flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;
									}
									prev = next + delta;

									// выделение данных
									delta = delim.length();
									next = arr_msg[i].find(delim, prev);
									posx = next;
									if(next == string::npos){
										//Ошибка
										temp_msg.err = "wrong format in x flag";
										temp_msg.f_bool = true;
										arr_cmd.push_back(temp_msg);
										continue;
									}

									temp_msg.cmd = (uint32_t)stoi(arr_msg[i].substr(posC +1, (posD -1) - posC));
									//temp_msg.addres_var = (uint32_t)stoi(arr_msg[i].substr(posA +1, (posD -1) - posA));
									temp_msg.data_in = (uint32_t)stoi(arr_msg[i].substr(posD +1, (posx -1) - posD));
									arr_cmd.push_back(temp_msg);
								}
								// Закончили парсинг
								/*-----------------------------------------------------------------------------------------------------------------------------*/
								//Выполнение комманд
								int count_cmd = arr_cmd.size();
								for (int i = 0; i < count_cmd; ++i) {

									switch (arr_cmd[i].cmd) {
									case 1: //
										arr_cmd[i].data_out = (uint32_t)Sensor1.Getdetect();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										/*arr_cmd[i].data_out = (uint32_t)pMotor->getStatusDirect();
										arr_cmd[i].need_resp = true;*/
										break;
									case 2: //
										arr_cmd[i].data_out = (uint32_t)Sensor2.Getdetect();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 3:
										Sensor1.PwrSet(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 4:
										Sensor2.PwrSet(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 5: //
										arr_cmd[i].data_out = (uint32_t)Sensor1.GetResult();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 6://
										arr_cmd[i].data_out = (uint32_t)Sensor2.GetResult();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 7://
										/*										arr_cmd[i].data_out = (uint32_t)Sensor1.Depth;
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";*/
										arr_cmd[i].err = "no_CMD";
										break;
									case 8:
										/*										arr_cmd[i].data_out = (uint32_t)Sensor2.Depth;
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";*/
										arr_cmd[i].err = "no_CMD";
										break;
									case 9:
										/*										Sensor1.change_settings = true; // включение режима настроек
										Sensor1.Depth = arr_cmd[i].data_in;
										Sensor1.change_settings = false; // выключение режима настроек
										arr_cmd[i].err = "OK";*/
										arr_cmd[i].err = "no_CMD";
										break;
									case 10:
										/*										Sensor2.change_settings = true; // включение режима настроек
										Sensor2.Depth = arr_cmd[i].data_in;
										Sensor2.change_settings = false; // выключение режима настроек
										arr_cmd[i].err = "OK";*/
										arr_cmd[i].err = "no_CMD";
										break;
									case 11:
										arr_cmd[i].data_out = (uint32_t)Sensor1.GetTimeoutRasing();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 12:
										arr_cmd[i].data_out = (uint32_t)Sensor2.GetTimeoutRasing();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 13:
										Sensor1.SetTimeoutRasing(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 14:
										Sensor2.SetTimeoutRasing(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 15:
										Sensor1.SetTimeCall(arr_cmd[i].data_in);
										settings.timeCall1 = arr_cmd[i].data_in;
										arr_cmd[i].err = "OK";
										break;
									case 16:
										Sensor2.SetTimeCall(arr_cmd[i].data_in);
										settings.timeCall2 = arr_cmd[i].data_in;
										arr_cmd[i].err = "OK";
										break;
									case 17:
										//call1 = 1;
										//call2 = 1;
										//arr_cmd[i].err = "OK";
										arr_cmd[i].err = "no_CMD";
										break;
									case 18://включение в качестве первого сенсора ултразвук
										if(arr_cmd[i].data_in == 1){
											settings.sensorType1 = 1;
											arr_cmd[i].err = "OK";
										}else if (arr_cmd[i].data_in == 2){
											settings.sensorType1 = 2;
											arr_cmd[i].err = "OK";
										}else{
											arr_cmd[i].err = "errData";
										}
										break;
									case 19://данные от ултразвука
										arr_cmd[i].need_resp = true;
										xSemaphoreTake(s1DistanceMutexHandle, 100);
										arr_cmd[i].data_out = (uint32_t)(distance_ul_S1); //переводим в сантиметры и сохранияем
										xSemaphoreGive(s1DistanceMutexHandle);
										arr_cmd[i].err = "OK";
										break;
									case 20://включение в качестве второго сенсора ултразвук
										if(arr_cmd[i].data_in == 1){
											settings.sensorType2 = 1;
											arr_cmd[i].err = "OK";
										}else if (arr_cmd[i].data_in == 2){
											settings.sensorType2 = 2;
											arr_cmd[i].err = "OK";
										}else{
											arr_cmd[i].err = "errData";
										}

										break;
									case 21://данные от ултразвука 2
										arr_cmd[i].need_resp = true;
										xSemaphoreTake(s2DistanceMutexHandle, 100);
										arr_cmd[i].data_out = (uint32_t)(distance_ul_S2); //переводим в сантиметры и сохранияем
										xSemaphoreGive(s2DistanceMutexHandle);
										arr_cmd[i].err = "OK";
										break;
									case 22: // коллибровка S1
										if(arr_cmd[i].data_in == 1){
											call1  = 1;
											arr_cmd[i].err = "OK";
										}else if (arr_cmd[i].data_in == 2){
											call1  = 2;
											arr_cmd[i].err = "OK";
										}else{
											arr_cmd[i].err = "err_CMD";
										}
										break;
									case 23:// коллибровка S2
										if(arr_cmd[i].data_in == 1){
											call2  = 1;
											arr_cmd[i].err = "OK";
										}else if (arr_cmd[i].data_in == 2){
											call2  = 2;
											arr_cmd[i].err = "OK";
										}else{
											arr_cmd[i].err = "err_CMD";
										}
										break;
									case 24:
										Sensor1.SetTrigger(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 25:
										Sensor2.SetTrigger(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 26:
										arr_cmd[i].data_out = (uint32_t)Sensor1.GetTrigger();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 27:
										arr_cmd[i].data_out = (uint32_t)Sensor2.GetTrigger();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 28:
										arr_cmd[i].data_out = (uint32_t)Sensor1.StatusCalibration();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 29:
										arr_cmd[i].data_out = (uint32_t)Sensor2.StatusCalibration();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 30:
										Sensor1.SetOffsetTime(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 31:
										Sensor2.SetOffsetTime(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 32:
										arr_cmd[i].data_out = (uint32_t)Sensor1.GetOffsetTime();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 33:
										arr_cmd[i].data_out = (uint32_t)Sensor2.GetOffsetTime();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 34:
										Sensor1.SetCallChanel(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 35:
										Sensor2.SetCallChanel(arr_cmd[i].data_in);
										arr_cmd[i].err = "OK";
										break;
									case 36:
										arr_cmd[i].data_out = (uint32_t)Sensor1.GetCallChanel();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 37:
										arr_cmd[i].data_out = (uint32_t)Sensor2.GetCallChanel();
										arr_cmd[i].need_resp = true;
										arr_cmd[i].err = "OK";
										break;
									case 38:
										arr_cmd[i].err = "no_CMD";
										break;
									case 39:
										arr_cmd[i].err = "no_CMD";
										break;
									case 40:
										mem_spi.W25qxx_EraseSector(0);
										mem_spi.Write(settings);
										arr_cmd[i].err = "OK";
										break;

									default:
										arr_cmd[i].err = "err_CMD";
										break;
									}
								}
								/*-----------------------------------------------------------------------------------------------------------------------------*/
								//Формируем ответ
								string resp;
								for (int i = 0; i < count_cmd; ++i) {
									resp.append(f_cmd + to_string(arr_cmd[i].cmd));
									if(arr_cmd[i].need_resp){
										resp.append(f_datd + to_string(arr_cmd[i].data_out));
									}else{
										resp.append(f_datd + arr_cmd[i].err);
									}
									resp.append(delim);
								}
								netconn_write(newconn, resp.c_str(), resp.size(), NETCONN_COPY);

							} while (netbuf_next(netbuf) >= 0);
							netbuf_delete(netbuf);

						}
						netconn_close(newconn);
						netconn_delete(newconn);
						//LED_IPadr.LEDoff();
					} else netconn_delete(newconn);
					osDelay(20);
				}
			}
		}
		osDelay(1);
	}
  /* USER CODE END eth_Task */
}

/* USER CODE BEGIN Header_mainTask2 */
/**
 * @brief Function implementing the MainTask2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_mainTask2 */
void mainTask2(void const * argument)
{
  /* USER CODE BEGIN mainTask2 */

	float temp_distance_ul = 0.0;

	switch (settings.sensorType2) {
	case 1: // оптика
		Sensor2.Init(&ADC_end2Handle, &hadc2, adc_buffer2, pwr2_GPIO_Port, pwr2_Pin, 2);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_buffer2, Sensor2.Depth);
		Sensor2.SetTimeCall(settings.timeCall2);
		break;
	case 2: // ултразвук
		Sensor2.Init(TIM4,TIM_CHANNEL_1 ,TIM_CHANNEL_2, pwr2_GPIO_Port, pwr2_Pin, 2);

		break;
	default:
		//Error_Handler();
		break;
	}

	/* Infinite loop */
	for(;;)
	{
		switch (settings.sensorType2) {
		case 1: // оптика
			switch (call2) {
			case 1: // каллибровка дистанции
				call2 = 0;
				//взять мютекс
				osMutexWait(mutexADCHandle, osWaitForever);

				//HAL_GPIO_WritePin(pwr2_GPIO_Port, pwr2_Pin, GPIO_PIN_SET); // питание
				Sensor2.PwrSet(2); // pwr on
				LED_IPadr.LEDon();
				osDelay(300);
				Sensor2.CallDistance();
				LED_IPadr.LEDoff();
				Sensor2.PwrSet(3); // pwr off
				//HAL_GPIO_WritePin(pwr2_GPIO_Port, pwr2_Pin, GPIO_PIN_RESET); // питание

				//вернуть мютекс
				osMutexRelease(mutexADCHandle);
				break;
			case 2: // каллибровка времени
				call2 = 0;
				//взять мютекс
				osMutexWait(mutexADCHandle, osWaitForever);

				//HAL_GPIO_WritePin(pwr1_GPIO_Port, pwr1_Pin, GPIO_PIN_SET); // питение
				Sensor2.PwrSet(2); // pwr on
				LED_IPadr.LEDon();
				osDelay(300);
				if(Sensor2.CallTime()){
					LED_IPadr.LEDoff();
					LED_IPadr.LEDon(10);
				}
				LED_IPadr.LEDoff();
				Sensor2.PwrSet(3); // pwr off
				//HAL_GPIO_WritePin(pwr1_GPIO_Port, pwr1_Pin, GPIO_PIN_RESET); // питение

				//вернуть мютекс
				osMutexRelease(mutexADCHandle);
				break;
			default:
				//взять мютекс
				osMutexWait(mutexADCHandle, osWaitForever);
				//запустить ацп
				HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_buffer2, Sensor2.Depth);
				//подождать симафор от АЦП
				osSemaphoreWait(ADC_end2Handle, osWaitForever);
				//вернуть мютекс
				osMutexRelease(mutexADCHandle);
				// обработать данные
				Sensor2.DataProcessing(adc_buffer2);
				//HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_buffer2, Sensor2.Depth);

				if(Sensor2.DetectPoll()){
					LED_IPadr.LEDon();
				}else{
					LED_IPadr.LEDoff();
				}
				break;
			}

			break;
			case 2: // ултразвук
				//HAL_GPIO_WritePin(pwr2_GPIO_Port, pwr2_Pin, GPIO_PIN_SET);
				xSemaphoreTake(s2DistanceMutexHandle, 100);
				temp_distance_ul = Sensor2.GetDistance();
				if(temp_distance_ul < 0) {
					distance_ul_S2 = 0;
				}else{
					distance_ul_S2 = temp_distance_ul;
				}
				xSemaphoreGive(s2DistanceMutexHandle);
				osDelay(100);

				break;
			default:
				//Error_Handler();
				break;
		}

	}
  /* USER CODE END mainTask2 */
}

/* USER CODE BEGIN Header_Debug_udp */
/**
 * @brief Function implementing the debug_udp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Debug_udp */
void Debug_udp(void const * argument)
{
  /* USER CODE BEGIN Debug_udp */

	while(gnetif.ip_addr.addr == 0){osDelay(1);}	//ждем получение адреса

	strIP = ip4addr_ntoa(&gnetif.ip_addr);

	//структуры для netcon
	struct netconn *conn;
	struct netconn *newconn;
	struct netbuf *netbuf;
	volatile err_t accept_err;
	//ip_addr_t local_ip;
	//ip_addr_t remote_ip;
	void 		*in_data = NULL;
	uint16_t 		data_size = 0;

	/* Infinite loop */
	for(;;)
	{

		conn = netconn_new(NETCONN_TCP);
		if (conn!=NULL)
		{
			netconn_listen(conn);//set port to listening mode
			while(1)
			{
				accept_err=netconn_accept(conn,&newconn);//suspend until new connection
				if (accept_err==ERR_OK)
				{
					//LED_IPadr.LEDon();
					while ((accept_err=netconn_recv(newconn,&netbuf))==ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
					{

						do
						{
							netbuf_data(netbuf,&in_data,&data_size);//get pointer and data size of the buffer
							in_str.assign((char*)in_data, data_size);//copy in string


							//Формируем ответ
							string resp;

							for (int i = 0; i < 100; ++i) {
								resp.append(to_string(debugBuf[i].time) + ";");
								resp.append(to_string(debugBuf[i].detect) + ";");
								resp.append(to_string(debugBuf[i].dada[0]) + "\n");

							}
							netconn_write(newconn, resp.c_str(), resp.size(), NETCONN_NOCOPY);
							debug_send = true;

						} while (netbuf_next(netbuf) >= 0);
						netbuf_delete(netbuf);

					}
					netconn_close(newconn);
					netconn_delete(newconn);
					//LED_IPadr.LEDoff();
				} else netconn_delete(newconn);
				osDelay(20);
			}
		}

		osDelay(1);
	}
  /* USER CODE END Debug_udp */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	/* This is called after the conversion is completed */
	if(hadc->Instance == ADC1)
	{
		HAL_ADC_Stop_DMA(&hadc1);
		osSemaphoreRelease(ADC_endHandle);
	}
	if(hadc->Instance == ADC2)
	{
		HAL_ADC_Stop_DMA(&hadc2);
		osSemaphoreRelease(ADC_end2Handle);
	}
}

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		Sensor2._acknowledgeChannelCapture();
	}
	if (htim->Instance == TIM3) {
		Sensor1._acknowledgeChannelCapture();
	}
}

/* USER CODE END Application */
