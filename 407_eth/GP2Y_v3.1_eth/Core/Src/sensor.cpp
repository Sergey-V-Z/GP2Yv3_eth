#include "sensor.h"
#include "utils.h"
//extern osSemaphoreId ADC_endHandle;
//extern ADC_HandleTypeDef hadc2;
//extern ADC_HandleTypeDef hadc1;
//extern uint16_t adc_buffer[1024];
extern osSemaphoreId setMutexHandle;

void sensor :: Init(settings_t *set, osSemaphoreId *ADC_endHandle, ADC_HandleTypeDef *hadc, uint16_t *adc_buffer, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr, int ID)
{
	setttings = set;
	id = ID;
	if(setttings->sensorSett[id].sensorType == NoInit){
		setttings->sensorSett[id].sensorType = Optic;

		this->ADC_endHandle = ADC_endHandle;
		this->hadc = hadc;
		this->adc_buffer = adc_buffer;
		this->GPIO_pwr = GPIO_pwr;
		this->Pin_pwr = Pin_pwr;
	}else{
		Error_Handler();
	}
}

void sensor :: Init(settings_t *set, TIM_TypeDef* tim, uint32_t triggerChannel, uint32_t echoChannel, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr, int ID, float soundSpeed)
{
	setttings = set;
	id = ID;
	if(setttings->sensorSett[id].sensorType == NoInit){
		setttings->sensorSett[id].sensorType = Ultrasound;

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
		this->GPIO_pwr = GPIO_pwr;
		this->Pin_pwr = Pin_pwr;

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

// обробатываем накопленные данные
uint32_t sensor :: DataProcessing(uint16_t *data){

	if(!change_settings){
		float Output = 0;
		/* Sum */
		for (int var = 0; var < Depth; ++var) {
			//Output += *data;
			if((*data) >= 100){ //на входе фильтра отсекаем маленькие значенияя
				Output = ExpRunningAvgAdaptive(*data); // фильтруем
				*data = 0; // обнуляем входные данные
			}
			data++; // идем дальше
		}

		/**/
		result = (uint16_t) (Output);

	}else{
		//Error_Handler();
	}
	return result;
}

// бегущее среднее с адаптивным коэффициентом
float sensor :: ExpRunningAvgAdaptive(float newVal) {
	static float filVal = 0;
	float k_H, k_L;

	xSemaphoreTake(setMutexHandle, 100);
	k_H = setttings->sensorSett[id].k_H;
	k_L = setttings->sensorSett[id].k_L;
	xSemaphoreGive(setMutexHandle);
	// резкость фильтра зависит от модуля разности значений
	if (abs(newVal - filVal) > 1.5) filVal += (newVal - filVal) *  k_H;
	else filVal += (newVal - filVal) *  k_L;

	//filVal += (newVal - filVal) * k;
	return filVal;
}

bool sensor :: DetectPoll(uint32_t tRising, uint32_t tFalling){

	uint32_t tempTimeOutRising, tempTimeOutFalling;

	// если таймауты не переданы в функцию или они нулевые выставляем из откалиброванных данных
	if(tRising == 0 && tFalling == 0){

		if(setttings->sensorSett[id].chanelCallTime == 0){
			setttings->sensorSett[id].chanelCallTime = 1;
		}

		xSemaphoreTake(setMutexHandle, 100);
		tempTimeOutRising = setttings->sensorSett[id].timeParametrs[setttings->sensorSett[id].chanelCallTime].callTimeMax + setttings->sensorSett[id].offsetTime;
		tempTimeOutFalling = setttings->sensorSett[id].timeParametrs[setttings->sensorSett[id].chanelCallTime].timOutFalling;
		xSemaphoreGive(setMutexHandle);
	}else{
		tempTimeOutRising = tRising;
		tempTimeOutFalling = tFalling;
	}

	xSemaphoreTake(setMutexHandle, 100);
	uint16_t temp = (setttings->sensorSett[id].callDistanceMin + setttings->sensorSett[id].triger);
	xSemaphoreGive(setMutexHandle);

	// если у нас сработка по входным данным и низкий уровень по выходу
	if(result > temp){

		if(detect == false){ // если сработки нету то проходим процедуру
			if(oldTimeRising == 0){
				oldTimeRising = HAL_GetTick();
			}

			timeRising = HAL_GetTick() - oldTimeRising; // обновляем время на таймере переднего фронта

			// если время вышло значит переводим в 1 и сбрасываем таймера
			if(timeRising >= tempTimeOutRising){
				detect = true;
				oldTimeFalling = 0;
				oldTimeRising = 0;
				timeRising = 0;
				timeFalling = 0;
			}
			if(oldTimeFalling !=0) {oldTimeFalling = 0;} // сбрасываем таймер заднего фронта
		}
		else{ // иначе сработка есть и процедуру не проходим а только сбрасываем таймера
			oldTimeFalling = 0;
			oldTimeRising = 0;
			timeRising = 0;
			timeFalling = 0;
		}
	}
	else{
		if(oldTimeRising != 0){ // если таймер переднего запущен то работаем с задним фронтом

			if(oldTimeFalling == 0){// если таймер заднего фронта в нуле значит нужно запустить таймер заднего
				oldTimeFalling = HAL_GetTick();
			}
			timeFalling = HAL_GetTick() - oldTimeFalling; // обновляем время на таймере заднего фронта
			timeRising = HAL_GetTick() - oldTimeRising;	// обновляем время на таймере переднего фронта

			// если время вышло значит переводим в 0 и сбрасываем таймера
			if((timeFalling >= tempTimeOutFalling) || (timeRising >= tempTimeOutRising)){
				detect = false;
				oldTimeFalling = 0;
				oldTimeRising = 0;
				timeRising = 0;
				timeFalling = 0;
			}
		}
		else{

			detect = false;
			oldTimeFalling = 0;
			oldTimeRising = 0;
			timeRising = 0;
			timeFalling = 0;
		}

	}
	//g_Detect = detect;
	return detect;
}

void sensor :: CallDistance(){
	calibrationInProgress = true;
	float peak = 0;
	float gorge = 10000;

	//добавить защиту при переходе времени через 0

	uint32_t old_time = HAL_GetTick();
	xSemaphoreTake(setMutexHandle, 100);
	uint32_t timeF1 = setttings->sensorSett[id].timeCall;
	xSemaphoreGive(setMutexHandle);

	//uint32_t timeF2 = timeCall - timeF1;

	//uint32_t timePulse = 0;
	//uint32_t timePulseOld = 0;
	//uint32_t timePulseMin = 0;
	//uint32_t timePulseMax = 0;

	//bool detect = false;
	//bool detectoOld = false;

	do
	{
		HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buffer, Depth);
		osSemaphoreWait(*ADC_endHandle, osWaitForever);
		DataProcessing(adc_buffer); // оброботка данных

		if((HAL_GetTick() - old_time) >= 10) // ждем стабилизации и начинаем писать данные
		{
			if(result > 500){ //отсекаем маленькие значения
				if(result > peak){peak = result;}
				if(result < gorge){gorge = result;}
			}
		}
	}
	while(!(timeF1 <= (HAL_GetTick() - old_time)));

	//пишем настройки
	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].callDistanceMax = peak;
	setttings->sensorSett[id].callDistanceMin = gorge;
	xSemaphoreGive(setMutexHandle);

	calibrationInProgress = false;
}

int sensor :: CallTime(){

	calibrationInProgress = true;

	//если канал не установлен ставим канал 1
	xSemaphoreTake(setMutexHandle, 100);
	if(setttings->sensorSett[id].chanelCallTime == 0){
		setttings->sensorSett[id].chanelCallTime = 1;
	}
	uint32_t timeF2 = setttings->sensorSett[id].timeCall;
	xSemaphoreGive(setMutexHandle);

	//добавить защиту при переходе времени через 0

	uint32_t old_time = HAL_GetTick();
	uint32_t timePulse = 0;
	uint32_t timePulseOld = 0;
	uint32_t timePulseMin = 0;
	uint32_t timePulseMax = 0;

	bool detect = false;
	static bool detectOld = false;
	timePulseOld = HAL_GetTick();

	do
	{
		HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buffer, Depth);
		osSemaphoreWait(*ADC_endHandle, osWaitForever);
		DataProcessing(adc_buffer); // оброботка данных

		detect = DetectPoll(1,1); // детектируем с минимальным временем

		if(detect != detectOld){
			if(detect){
				timePulseOld = HAL_GetTick();
			}else{
				timePulse = (HAL_GetTick() - timePulseOld);
				if(timePulse > timePulseMax){timePulseMax = timePulse;}
				if(timePulse < timePulseMin){timePulseMin = timePulse;}
				//timePulseOld = HAL_GetTick();
			}

		}

		detectOld = detect;
		osDelay(1);


	}
	while(!(timeF2 <= (HAL_GetTick() - old_time)));

	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].timeParametrs[setttings->sensorSett[id].chanelCallTime].callTimeMax = timePulseMax;
	setttings->sensorSett[id].timeParametrs[setttings->sensorSett[id].chanelCallTime].callTimeMin = timePulseMin;
	xSemaphoreGive(setMutexHandle);

	calibrationInProgress = false;
	return 1;
}

bool sensor :: Getdetect(){
	//mutex heare
	if (setttings->sensorSett[id].modePwr == 1) {
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
	}
	return detect;
}

void sensor :: PwrSet(uint16_t r){

	switch (r) {
	case 1:
		xSemaphoreTake(setMutexHandle, 100);
		setttings->sensorSett[id].modePwr = 1; // режим при котором питание отключается при чтении
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_SET);
		xSemaphoreGive(setMutexHandle);
		break;
	case 2:
		xSemaphoreTake(setMutexHandle, 100);
		setttings->sensorSett[id].modePwr = 2; // режим при котором питание отключается только принудительно
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_SET);
		xSemaphoreGive(setMutexHandle);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
		break;
	default:
		HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
		break;
	}
}

uint16_t sensor :: GetResult(){
	uint16_t Ret = 0;
	switch (setttings->sensorSett[id].sensorType) {
	case Optic: // оптика
		//перенести выключение в pool сделать его через флаг (Нужно ли?)
		xSemaphoreTake(setMutexHandle, 100);
		if (setttings->sensorSett[id].modePwr == 1) {
			HAL_GPIO_WritePin(GPIO_pwr, Pin_pwr, GPIO_PIN_RESET);
		}
		xSemaphoreGive(setMutexHandle);
		Ret = result;
		break;
	case Ultrasound: // ултразвук

		break;
	default:

		break;
	}

	//защитить результат мютексом
	return Ret;

}

void sensor :: SetOffsetMin(uint16_t offset){
	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].callDistanceMin = offset;
	xSemaphoreGive(setMutexHandle);
}

void sensor :: SetTrigger(uint16_t offset){
	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].triger = offset;
	xSemaphoreGive(setMutexHandle);
}

void sensor :: SetOffsetMax(uint16_t offset){
	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].callDistanceMax = offset;
	xSemaphoreGive(setMutexHandle);
}

void sensor :: SetTimeCall(uint32_t time){
	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].timeCall = time;
	xSemaphoreGive(setMutexHandle);
}

uint16_t sensor :: GetOffsetMin(){
	xSemaphoreTake(setMutexHandle, 100);
	uint16_t ret = setttings->sensorSett[id].callDistanceMin;
	xSemaphoreGive(setMutexHandle);
	return ret;
}

uint16_t sensor :: GetOffsetMax(){
	xSemaphoreTake(setMutexHandle, 100);
	uint16_t ret = setttings->sensorSett[id].callDistanceMax;
	xSemaphoreGive(setMutexHandle);
	return ret;
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

float sensor :: GetDistanceInSeconds()
{
	if (echoDelay < 0) {
		return -1.0f;
	} else {
		return (float)echoDelay / (COUNTER_FREQUNCY * 2.0f);
	}
}

float sensor :: GetDistance()
{
	float delay = GetDistanceInSeconds();
	if (delay < 0) {
		return delay;
	} else {
		return delay * soundSpeed;
	}
}

void sensor::SetCallChanel(uint16_t ch) {
	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].chanelCallTime = ch;
	xSemaphoreGive(setMutexHandle);
}

uint16_t sensor::GetCallChanel() {
	xSemaphoreTake(setMutexHandle, 100);
	uint16_t ret = setttings->sensorSett[id].chanelCallTime;
	xSemaphoreGive(setMutexHandle);
	return ret;
}

uint16_t sensor::GetTrigger(){
	xSemaphoreTake(setMutexHandle, 100);
	uint16_t ret = setttings->sensorSett[id].triger;
	xSemaphoreGive(setMutexHandle);
	return ret;
}

bool sensor::StatusCalibration() {
	return calibrationInProgress;
}

void sensor::SetOffsetTime(uint32_t time) {
	setttings->sensorSett[id].offsetTime = time;
}

uint32_t sensor::GetOffsetTime() {
	return setttings->sensorSett[id].offsetTime;
}

uint32_t sensor::GetTimeoutRasing() {
	uint16_t ret = 0;
	xSemaphoreTake(setMutexHandle, 100);
	if(setttings->sensorSett[id].chanelCallTime == 0){
		setttings->sensorSett[id].chanelCallTime = 1;
	}
	ret = setttings->sensorSett[id].timeParametrs[setttings->sensorSett[id].chanelCallTime].callTimeMax;
	xSemaphoreGive(setMutexHandle);

	return ret;
}

void sensor::SetTimeoutRasing(uint32_t time) {

	xSemaphoreTake(setMutexHandle, 100);
	if(setttings->sensorSett[id].chanelCallTime == 0){
		setttings->sensorSett[id].chanelCallTime = 1;
	}
	setttings->sensorSett[id].timeParametrs[setttings->sensorSett[id].chanelCallTime].callTimeMax = time;
	xSemaphoreGive(setMutexHandle);
}

void sensor::SetSensorType(SensorType type) {
	xSemaphoreTake(setMutexHandle, 100);
	setttings->sensorSett[id].sensorType = type;
	xSemaphoreGive(setMutexHandle);
}

SensorType sensor::GetSensorType() {
	xSemaphoreTake(setMutexHandle, 100);
	SensorType type = setttings->sensorSett[id].sensorType;
	xSemaphoreGive(setMutexHandle);
	return type;
}
