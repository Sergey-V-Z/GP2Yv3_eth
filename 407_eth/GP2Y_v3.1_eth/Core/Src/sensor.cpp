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

	if(!change_settings){
		float Output = 0;
		/* Sum */
		for (int var = 0; var < Depth; ++var) {
			//Output += *data;
			if((*data) >= 400){ //на входе фильтра отсекаем маленькие значенияя
				Output = expRunningAvgAdaptive(*data); // фильтруем
				*data = 0; // обнуляем входные данные
			}
			data++; // идем дальше
		}

		/*  */
		Result = (uint16_t) (Output);
	}
}

// бегущее среднее с адаптивным коэффициентом
float sensor :: expRunningAvgAdaptive(float newVal) {
  static float filVal = 0;

  // резкость фильтра зависит от модуля разности значений
  if (abs(newVal - filVal) > 1.5) filVal += (newVal - filVal) *  k_H;
  else filVal += (newVal - filVal) *  k_L;

  //filVal += (newVal - filVal) * k;
  return filVal;
}

bool sensor :: detectPoll(){
  
  //if((Result > offsetMin) && (Result < offsetMax)){
  if(Result > (offsetMin + triger)){
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
  gorge = 10000;
  //добавить защиту при переходе времени через 0

  uint32_t old_time = HAL_GetTick();

  do
  {
	  osSemaphoreWait(*ADC_endHandle, osWaitForever);
	  data_processing(adc_buffer); // оброботка данных
	  HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buffer, Depth);

	  if((HAL_GetTick() - old_time) >= 100) // ждем стабилизации и начинаем писать данные
	  {
		  if(Result > 500){ //отсекаем маленькие значения
			  if(Result > peak){peak = Result;}
			  if(Result < gorge){gorge = Result;}
		  }
	  }
  }
  while(!(timeCall <= (HAL_GetTick() - old_time)));

  offsetMax = peak;
  offsetMin = gorge;


}

bool sensor :: getdetect(){
	if (modePwr == 1) {
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
	}
   return detect;
}

void sensor :: pwr_set(uint16_t r){

	switch (r) {
		case 1:
			modePwr = 1; // режим при котором питание отключается при чтении
			HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_SET);
			break;
		case 2:
			modePwr = 2; // режим при котором питание отключается только принудительно
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
	//перенести выключение в pool  сделать его через флаг
	if (modePwr == 1) {
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
	}

	//защитить результат мютексом
   return Result;
}
void sensor :: setOffsetMin(uint16_t offset){
   offsetMin = offset;
}

void sensor :: setTrigger(uint16_t offset){
	sensor :: triger = offset;
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
