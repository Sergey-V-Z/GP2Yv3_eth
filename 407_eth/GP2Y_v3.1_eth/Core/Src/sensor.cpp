#include "sensor.h"
#include "utils.h"
//extern osSemaphoreId ADC_endHandle;
//extern ADC_HandleTypeDef hadc2;
//extern ADC_HandleTypeDef hadc1;
//extern uint16_t adc_buffer[1024];

void sensor :: Init(osSemaphoreId *ADC_endHandle, ADC_HandleTypeDef *hadc, uint16_t *adc_buffer, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr)
{
	if(sensorType == NoInit){
		sensorType = Optic;

		this->ADC_endHandle = ADC_endHandle;
		this->hadc = hadc;
		this->adc_buffer = adc_buffer;
		this->GPIO_pwr = GPIO_pwr;
		this->Pin_pwr = Pin_pwr;
	}else{
		Error_Handler();
	}
}

void sensor :: Init(TIM_TypeDef* tim, uint32_t triggerChannel, uint32_t echoChannel, float soundSpeed)
{
	if(sensorType == NoInit){
		sensorType = Ultrasound;

		// default driver state
		echoPulseStart = 0;
		echoPulseEnd = 0;
		echoPulseState = NOT_STARTED;
		echoDelay = -1;

		// save driver parameters
		this->soundSpeed = soundSpeed;

		// save timer parameters
		hTim.Instance = tim;
		this->triggerChannel = triggerChannel;
		this->echoChannel = echoChannel;

		// configure and initizialize timer
		hTim.State = HAL_TIM_STATE_RESET;

		// calculate timer prescaler to make timer frequncy equals COUNTER_FREQUNCY
		hTim.Init.Prescaler = (HAL_RCC_GetPCLK1Freq()*2) / COUNTER_FREQUNCY - 1;

		// calculate counter period
		hTim.Init.Period = COUNTER_FREQUNCY / PROBING_FREQUNCY - 1;
		hTim.Init.CounterMode = TIM_COUNTERMODE_UP;
		hTim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		hTim.Init.RepetitionCounter = 0;
		hTim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

		// configure timer
		enableTimerClock(hTim.Instance);
		HAL_TIM_Base_Init(&hTim);

		// configure and initizialize trigger channel
		TIM_OC_InitTypeDef sConfigTrigger;
		sConfigTrigger.OCMode = TIM_OCMODE_PWM1;
		sConfigTrigger.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigTrigger.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigTrigger.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigTrigger.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		sConfigTrigger.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigTrigger.Pulse = (TRIGGER_PULSE_LEN * COUNTER_FREQUNCY) / 1000000;

		// configure trigger channel
		HAL_TIM_PWM_ConfigChannel(&hTim, &sConfigTrigger, triggerChannel);

		// configure capture channel
		TIM_IC_InitTypeDef sConfigCapture;

		// detect start and end of the echo pulse
		sConfigCapture.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
		sConfigCapture.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigCapture.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigCapture.ICFilter = 0x00; // magic value between 0x00 and 0x0F
		HAL_TIM_IC_ConfigChannel(&hTim, &sConfigCapture, echoChannel);

		// configure timer interruptions in the NVIC

		IRQn_Type triggerIRQ = getIRQCode(hTim.Instance, TIM_IT_UPDATE);
		IRQn_Type captureIRQ = getIRQCode(hTim.Instance, getTimInterruptTypeCode(echoChannel));
		HAL_NVIC_SetPriority(triggerIRQ, INTERRUPT_PRIOIRY, INTERRUPT_SUBPRIOIRY);
		HAL_NVIC_EnableIRQ(triggerIRQ);
		HAL_NVIC_SetPriority(captureIRQ, INTERRUPT_PRIOIRY, INTERRUPT_SUBPRIOIRY);
		HAL_NVIC_EnableIRQ(captureIRQ);


		// start timer
		// enable update interrupt at the timer side
		__HAL_TIM_ENABLE_IT(&hTim, TIM_IT_UPDATE);

		// enable echo capture/compare interrupt at the timer side
		__HAL_TIM_ENABLE_IT(&hTim, getTimInterruptTypeCode(echoChannel));

		// enable trigger channel
		TIM_CCxChannelCmd(hTim.Instance, triggerChannel, TIM_CCx_ENABLE);
		if (IS_TIM_BREAK_INSTANCE(hTim.Instance) != RESET) {
			__HAL_TIM_MOE_ENABLE(&hTim);
		}

		// enable echo channel
		TIM_CCxChannelCmd(hTim.Instance, echoChannel, TIM_CCx_ENABLE);

		// start timer
		__HAL_TIM_ENABLE(&hTim);
	}else{
		Error_Handler();
	}

}
void sensor :: Init(osSemaphoreId *ADC_endHandle, ADC_HandleTypeDef *hadc, uint16_t *adc_buffer, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr, int ID){
	sensor :: ADC_endHandle = ADC_endHandle;
	sensor :: hadc = hadc;
	sensor :: adc_buffer = adc_buffer;
	sensor :: GPIO_pwr = GPIO_pwr;
	sensor :: Pin_pwr = Pin_pwr;
	id = ID;
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

		/**/
		Result = (uint16_t) (Output);
	}else{
		Error_Handler();
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
	  if(this->id == 1){
		  __NOP();
	  }
    if(oldTime == 0){
      oldTime = HAL_GetTick();
    }
    
    time = HAL_GetTick() - oldTime; 
    
    if(time >= timOut){
  	  if(this->id == 1){
  		  __NOP();
  	  }
      detect = true;
      oldTime = 0;

    }
  }
  else{
	  if(this->id == 1){
		  __NOP();
	  }
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

		if((HAL_GetTick() - old_time) >= 10) // ждем стабилизации и начинаем писать данные
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
	uint16_t Ret = 0;
	switch (sensorType) {
	case Optic: // оптика
		//перенести выключение в pool  сделать его через флаг
		if (modePwr == 1) {
			HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
		}

		Ret = Result;
		break;
	case Ultrasound: // ултразвук

		break;
	default:

		break;
	}

	//защитить результат мютексом
	return Ret;

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

//for hcsr04

void sensor :: _acknowledgeTimerUpdate()
{
	if (echoPulseState == ENDED) {
		echoDelay = echoPulseEnd - echoPulseStart;
	} else {
		// there no echo sigma or echo signal isn't ended
		echoDelay = -1;
	}
	echoPulseState = NOT_STARTED;
}

void sensor :: _acknowledgeChannelCapture()
{
	switch (echoPulseState) {
	case NOT_STARTED:
		echoPulseStart = HAL_TIM_ReadCapturedValue(&hTim, echoChannel);
		echoPulseState = STARTED;
		break;
	case STARTED:
		echoPulseEnd = HAL_TIM_ReadCapturedValue(&hTim, echoChannel);
		echoPulseState = ENDED;
		break;
	default:
		// do nothing
		break;
	}
}

float sensor :: getDistanceInSeconds()
{
	if (echoDelay < 0) {
		return -1.0f;
	} else {
		return (float)echoDelay / (COUNTER_FREQUNCY * 2.0f);
	}
}

float sensor :: getDistance()
{
	float delay = getDistanceInSeconds();
	if (delay < 0) {
		return delay;
	} else {
		return delay * soundSpeed;
	}
}
