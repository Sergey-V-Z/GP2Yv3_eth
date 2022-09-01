#include "sensor.h"
//extern osSemaphoreId ADC_endHandle;
//extern ADC_HandleTypeDef hadc2;
//extern ADC_HandleTypeDef hadc1;
//extern uint16_t adc_buffer[1024];

sensor :: sensor(){
   
}

sensor :: ~sensor(){
   
}
void sensor :: Init(osSemaphoreId *ADC_endHandle, ADC_HandleTypeDef *hadc, uint16_t *adc_buffer, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr){
	sensor :: ADC_endHandle = ADC_endHandle;
	sensor :: hadc = hadc;
	sensor :: adc_buffer = adc_buffer;
	sensor :: GPIO_pwr = GPIO_pwr;
	sensor :: Pin_pwr = Pin_pwr;
}

// обробатываем накопленные данные
void sensor :: data_processing(uint16_t *data){

	for (int var = 0; var < 16; ++var) {
		Filter_SMA(*data); // обробатываем
		*data = 0; // обнуляем
		data++; // идем дальше
	}

}

bool sensor :: detectPoll(){
  
  //if((Result > offsetMin) && (Result < offsetMax)){
  if(Result > offsetMax){
    if(oldTime == 0){
      oldTime = HAL_GetTick();
    }
    
    time = HAL_GetTick() - oldTime; 
    
    if(time >= timOut){
      detect = true;
      oldTime = 0;
    }
  }
  else{
    detect = false;
    oldTime = 0;
  }
  return detect;
}

void sensor :: Call(){
  peak = 0;
  gorge = 0;

  uint32_t old_time = HAL_GetTick();

  while(!(timeCall <= (HAL_GetTick() - old_time)))
  {
	 osSemaphoreWait(*ADC_endHandle, osWaitForever);
	 uint16_t *data = adc_buffer;
	 for (int var = 0; var < 16; ++var) {
		 Filter_SMA(*data); // обробатываем
		 *data = 0; // обнуляем
		 data++; // идем дальше
	 }
	 HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buffer, 16);
  }
  //uint32_t test_time = HAL_GetTick() - old_time;
   offsetMax = peak;
   offsetMin = gorge;
}

bool sensor :: getdetect(){
	if (mode == 1) {
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
	}
   return detect;
}

void sensor :: pwr_set(uint16_t r){

	switch (r) {
		case 1:
			mode = 1; // режим при котором питание отключается при чтрении
			HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_SET);
			break;
		case 2:
			mode = 2; // режим при котором питание отключается только принудительно
			HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
			break;
		default:
			HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
			break;
	}
}

uint16_t sensor :: Get_Result(){
	if (mode == 1) {
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
	}
   return Result;
}
void sensor :: setOffsetMin(uint16_t offset){
   offsetMin = offset;
}

void sensor :: setOffsetMax(uint16_t offset){
   offsetMax = offset;
}

void sensor :: setTimeCall(uint32_t time){
   timeCall = time;
}

uint16_t sensor :: getOffsetMin(){
   return offsetMin;
}

uint16_t sensor :: getOffsetMax(){
   return offsetMax;
}
