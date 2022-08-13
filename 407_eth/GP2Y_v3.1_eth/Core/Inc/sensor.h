#include "filter_sma.h"
#include "cmsis_os.h"

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
class sensor: public filter{
  public:
   sensor();
   ~sensor();
   void setOffsetMin(uint16_t offset);
   void setOffsetMax(uint16_t offset);
   void setTimeCall(uint32_t time);
   void data_processing(uint16_t *data);
   uint16_t getOffsetMin();
   uint16_t getOffsetMax();
   bool detectPoll();
   uint16_t Get_Result();
   bool getdetect();
   void Call(); // калибрует датчик, принимает указатель на переменную в которую поступаю свежие данные
   
   uint32_t timOut = 10;
   
  private:
   uint16_t offsetMin = 0;              // зона работы датчика
   uint16_t offsetMax = 4096;           // зона работы датчика
   uint32_t timeCall = 3000;            // время выполнение калибровки
   bool detect;                         // в зоне сенсора что то есть
   
   uint32_t oldTime = 0;
   uint32_t time = 0;
       
};
