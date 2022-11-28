#include "main.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "main.h"

//******************
// CLASS: sensor
//
// DESCRIPTION:
//  distance optical sensor class
//
// CREATED: 03.03.2021, by Sergey
//
// FILE: sensor.h
//
class sensor{
  public:
   sensor() = default;
   ~sensor() = default;
   void setOffsetMin(uint16_t offset);
   void setTrigger(uint16_t offset);
   void setOffsetMax(uint16_t offset);
   void setTimeCall(uint32_t time);
   void data_processing(uint16_t *data);
   uint16_t getOffsetMin();
   uint16_t getOffsetMax();
   bool detectPoll();
   uint16_t Get_Result();
   bool getdetect();
   void Call(); // калибрует датчик, принимает указатель на переменную в которую поступаю свежие данные
   void pwr_set(uint16_t r);
   void Init(TIM_TypeDef* tim, uint32_t triggerChannel, uint32_t echoChannel, int ID, float soundSpeed = 343.0f);
   void Init(osSemaphoreId *ADC_endHandle, ADC_HandleTypeDef *hadc, uint16_t *adc_buffer, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr, int ID);
   
   uint32_t timOut = 200;
   bool change_settings = false;
   uint16_t Depth = 10;

   //for hcsr04

   /*Note: pins should be configured separatly.*/
   void init(TIM_TypeDef* tim, uint32_t triggerChannel, uint32_t echoChannel, float soundSpeed = 343.0f);

    /*This method should be invoked when timer update event occurs*/
   void _acknowledgeTimerUpdate();

   /*This method should be invoked when channel input capture event occurs*/
   void _acknowledgeChannelCapture();

   /*Negative values will be returned if there is no object before sensor. */
   float getDistance();

   /*Negative values will be returned if there is no object before sensor. */
    float getDistanceInSeconds();



  private:
    enum SensorType {
    	Optic,
		Ultrasound,
		NoInit,
    };

   float expRunningAvgAdaptive(float newVal);

   uint16_t offsetMin = 0;              // зона работы датчика
   uint16_t offsetMax = 4096;           // зона работы датчика
   uint16_t triger = 100;           	// смещение от ленты
   uint32_t timeCall = 5000;            // время выполнение калибровки
   bool detect;                         // в зоне сенсора что то есть
   uint16_t modePwr; 						// режим работы питания
   float k_H = 0.1;						// коэфицент для быстрых изменений // начальное 0.9
   float k_L= 0.03;						// коэфицент для медленных изменений
   uint16_t Result = 0;
   float gorge = 0;                  // минимальное значение
   float peak = 0;                   // пиковое значение
   int id = 0;

   
   osSemaphoreId *ADC_endHandle;
   ADC_HandleTypeDef *hadc;
   uint16_t *adc_buffer;
   GPIO_TypeDef* GPIO_pwr;
   uint16_t Pin_pwr;

   uint32_t oldTimeRising = 0;
   uint32_t timeRising = 0;
   uint32_t oldTimeFalling = 0;
   uint32_t timeFalling = 0;
   SensorType sensorType = NoInit;

   //for hcsr04
   enum EchoPulseState {
        NOT_STARTED,
        STARTED,
        ENDED,
    };

    float soundSpeed;

    // base timer structure and channel codes
    TIM_HandleTypeDef hTim;
    uint32_t triggerChannel;
    uint32_t echoChannel;

    // start and end of echo pulse
    uint32_t echoPulseStart;
    uint32_t echoPulseEnd;

    // echo pulse state
    EchoPulseState echoPulseState;

    // calculated echo delay
    int32_t echoDelay;

     // control parameters
    static const uint32_t PROBING_FREQUNCY = 20; // 20 Hz
    static const uint32_t COUNTER_FREQUNCY = 100000; // 100 kHz
    static const uint32_t TRIGGER_PULSE_LEN = 10; // 10 us
    static const uint32_t INTERRUPT_PRIOIRY = 8;
    static const uint32_t INTERRUPT_SUBPRIOIRY = 8;
       
};
