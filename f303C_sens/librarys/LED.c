/*
  Обстракция для управления RGB светодиодом
*/

#include "LED.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"

// переменная хронящая настройки состояния диодов
static led_t R = {0};
static led_t G = {0};
static led_t B = {0};

// масив указателей на диоды. порядок соответствут ledName_t
led_t *p_led[3] = {&R, &G, &B};

// инициализация в потоке до цикла
void LEDsInit()
{  
  // настраиваем переменные
    // индикатор загрузки
    R.mode = on_off;
    R.status = OFF;
    R.status_last = OFF;
    R.timeOff = 0;
    R.timeOn = 0;
    R.ms_counter = 0;
    R.puls_counter = 0;
    R.port = LED_S1_GPIO_Port;
    R.pin = LED_S1_Pin;
    
    G.mode = on_off;
    G.status = OFF;
    G.status_last = OFF;
    G.timeOff = 0;
    G.timeOn = 0;
    G.ms_counter = 0;
    G.puls_counter = 0;
    G.port = LED_S2_GPIO_Port;
    G.pin = LED_S2_Pin;

    B.mode = on_off;
    B.status = OFF;
    B.status_last = OFF;
    B.timeOff = 0;
    B.timeOn = 0;
    B.ms_counter = 0;
    B.puls_counter = 0;
    B.port = LED_S3_GPIO_Port;
    B.pin = LED_S3_Pin;
    
}


// функция управления диодами вызывать из потока в бесконечном цикле
void handlerLED()
{
  for(int i = 0;i < 3;i++)
  {
    switch (p_led[i]->mode)
    {
      case on_off:
      {	
        switch (p_led[i]->status)
        {
          case ON:
          {	
            if(p_led[i]->status_last != p_led[i]->status){
              HAL_GPIO_WritePin(p_led[i]->port, p_led[i]->pin, GPIO_PIN_RESET);
            }
            break;
          }
          case OFF:
          {	
            if(p_led[i]->status_last != p_led[i]->status){
              HAL_GPIO_WritePin(p_led[i]->port, p_led[i]->pin, GPIO_PIN_SET);
            }
            break;
          }
          default:
          {	
            break;
          }
        }
        break;
      }
      case blinks:
      {	
        // добовляем счетчик
        p_led[i]->ms_counter += 10;
        // если выключен
        if (p_led[i]->status == ON)
        {        
          if (p_led[i]->ms_counter >= p_led[i]->timeOn)
          {
            p_led[i]->status = OFF;
            p_led[i]->ms_counter = 0;
            HAL_GPIO_WritePin(p_led[i]->port, p_led[i]->pin, GPIO_PIN_SET);
          }
        }
        // если включен
        else if (p_led[i]->status == OFF)
        {
          if (p_led[i]->ms_counter >= p_led[i]->timeOff)
          {
            p_led[i]->status = ON;
            p_led[i]->ms_counter = 0;
            HAL_GPIO_WritePin(p_led[i]->port, p_led[i]->pin, GPIO_PIN_RESET);
          }
        }
        break;
      }
      case puls:
      {	

        break;
      }
      case noinit:
      {	
        p_led[i]->mode = on_off;
        p_led[i]->status = OFF;
        p_led[i]->timeOff = 0;
        p_led[i]->timeOn = 0;
        p_led[i]->ms_counter = 0;
        break;
      }
    }
  }
}


// функция включения выключения диода
void on_off_LED (ledName_t LED, ledState_t status)
{
  p_led[LED]->mode = on_off;
  p_led[LED]->status = status;
}

void blinks_LED (ledName_t LED, ledState_t status, uint32_t timeOn, uint32_t timeOff)
{
  p_led[LED]->mode = blinks;
  p_led[LED]->timeOn = timeOn;
  p_led[LED]->timeOff = timeOff;
  p_led[LED]->status = status;

}

// переключение диодов
void Settings_LED (ledMode_t Mode, ledName_t LED, ledState_t status, uint32_t timeOn, uint32_t timeOff)
{
//  if ((Mode == NULL) | (LED == NULL) | (status == NULL))
//  {return;}
  
  switch (LED)
  {
    case LedR:
    {	
      p_led[LedR]->mode = Mode;
      if((timeOn != NULL) | (timeOff != NULL)){
        p_led[LedR]->timeOn = timeOn;
        p_led[LedR]->timeOff = timeOff;}
      p_led[LedR]->status = status;

      p_led[LedG]->mode = on_off;
      p_led[LedG]->status = OFF;

      p_led[LedB]->mode = on_off;
      p_led[LedB]->status = OFF;

      break;
    }
    case LedG:
    {	
      p_led[LedR]->mode = on_off;
      p_led[LedR]->status = OFF;

      p_led[LedG]->mode = Mode;
      if((timeOn != NULL) | (timeOff != NULL)){
        p_led[LedG]->timeOn = timeOn;
        p_led[LedG]->timeOff = timeOff;}
      p_led[LedG]->status = status;

      p_led[LedB]->mode = on_off;
      p_led[LedB]->status = OFF;

      break;
    }
    case LedB:
    {	
      p_led[LedR]->mode = on_off;
      p_led[LedR]->status = OFF;

      p_led[LedG]->mode = on_off;
      p_led[LedG]->status = OFF;

      p_led[LedB]->mode = Mode;
      if((timeOn != NULL) | (timeOff != NULL)){
        p_led[LedB]->timeOn = timeOn;
        p_led[LedB]->timeOff = timeOff;}
      p_led[LedB]->status = status;

      break;
    }
  }

}
