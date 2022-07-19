#include "main.h"
#include "stdbool.h"

typedef enum 
{
  noinit,
	on_off,
	blinks,
  puls
}ledMode_t;

typedef enum 
{
  LedR = 0,
	LedG = 1,
	LedB = 2
}ledName_t;

typedef enum 
{
	ON,
	OFF
}ledState_t;

typedef struct
{
  uint32_t      timeOn;
  uint32_t      timeOff;
  uint32_t      ms_counter;
  uint32_t      puls_counter;
  ledMode_t     mode;
  ledState_t    status;
  ledState_t    status_last;
  GPIO_TypeDef  *port;
  uint16_t      pin;
} led_t;


void LEDsInit(void);
void handlerLED(void);
//void on_off_LED (ledName_t LED, ledState_t status);
//void blinks_LED (ledName_t LED, ledState_t status, uint32_t timeOn, uint32_t timeOff);
void Settings_LED (ledMode_t Mode, ledName_t LED, ledState_t status, uint32_t timeOn, uint32_t timeOff);
