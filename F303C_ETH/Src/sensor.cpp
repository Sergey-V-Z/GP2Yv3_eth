#include "sensor.h"

sensor :: sensor(){
   
}

sensor :: ~sensor(){
   
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

void sensor :: Call(uint16_t *data){
  peak = 0;
  gorge = 0;
  for(int i = 0; i < timeCall; ++i)
  {
     Filter_SMA(*data);
     osDelay(1);
  }
   offsetMax = peak;
   offsetMin = gorge;
      
}

bool sensor :: getdetect(){
   return detect;
}

uint16_t sensor :: Get_Result(){
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
