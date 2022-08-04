#include "main.h"
/* Choose filter order */
#define FILTER_SMA_ORDER 100

//******************
// CLASS: filter
//
// DESCRIPTION:
//  --
//
// CREATED: 03.03.2021, by Sergey
//
// FILE: filter_sma.c
//
class filter{
  public:
   filter();
   ~filter();
   void Filter_SMA(uint16_t For_Filtered);
   uint16_t Get_Result();
   uint16_t Depth = 10;
  private:
   uint16_t Filter_Buffer[FILTER_SMA_ORDER] = {0,};
   uint16_t Result = 0;
   uint16_t gorge = 0;                  // минимальное значение
   uint16_t peak = 0;                   // пиковое значение
   
   friend class sensor;
   
};