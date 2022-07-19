#include "EEPROM.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdint.h>
//#include "LL_I2C.h"

extern I2C_HandleTypeDef hi2c1;
extern osMutexId MutexMemHandle;
static const uint16_t Start_EEPR_Addr = 0xA0;//, Max_EEPR_Addr = 2;

EEPR_State Write_EEPROM(settings_t settings)
{
  // переменные для проверки
  HAL_StatusTypeDef HAL_StatusW, HAL_StatusR;
  EEPR_State E_Stat = EEPR_OK;
  settings_t settingsTest;
 
  // указатели на данные и на память для тестов
  uint8_t *charPtr_settings = (uint8_t*)&settings;
  uint8_t *ptr_test = (uint8_t*)&settingsTest;
  
  uint32_t Size = sizeof(settings);
  
  //osMutexWait(MutexMemHandle, osWaitForever);
  
  //Прверка на присутствие устройства
  if(HAL_I2C_IsDeviceReady(&hi2c1,Start_EEPR_Addr,2,500))
  {return DEV_NOT_FAUND;}

//  HAL_StatusW = HAL_I2C_Mem_Write(&hi2c1, Start_EEPR_Addr, addressOfSettings, I2C_MEMADD_SIZE_8BIT, charPtr_settings, Size, 1000);
//  HAL_Delay(30);
//  HAL_StatusR = HAL_I2C_Mem_Read(&hi2c1, Start_EEPR_Addr, addressOfSettings, I2C_MEMADD_SIZE_8BIT, ptr_test, Size, 1000);
  if(Size > 16)
  {
    uint32_t c = Size /16, pos = 0, ost = Size%16;
    for(uint32_t i = 0;i <= c;i++)
    {
      HAL_StatusW = HAL_I2C_Mem_Write(&hi2c1, Start_EEPR_Addr, addressOfSettings + pos, I2C_MEMADD_SIZE_8BIT, charPtr_settings + pos, 16, 1000);
      pos +=16;
    }
    if(ost)
    {
      HAL_StatusW = HAL_I2C_Mem_Write(&hi2c1, Start_EEPR_Addr, addressOfSettings + pos, I2C_MEMADD_SIZE_8BIT, charPtr_settings + pos, Size - pos, 1000);
    }
  }else 
  {
    HAL_StatusW = HAL_I2C_Mem_Write(&hi2c1, Start_EEPR_Addr, addressOfSettings, I2C_MEMADD_SIZE_8BIT, charPtr_settings, Size, 1000);
  }
  //HAL_Delay(1);
  
  for(int i = 0;((HAL_I2C_IsDeviceReady(&hi2c1,Start_EEPR_Addr,5,500)) | (i >= 100));i++){ HAL_Delay(10);}
    
  HAL_StatusR = HAL_I2C_Mem_Read(&hi2c1, Start_EEPR_Addr, addressOfSettings, I2C_MEMADD_SIZE_8BIT, ptr_test, Size, 1000);
  
//  if(ll_IsDeviceReady(I2C1,Start_EEPR_Addr,2,500))
//  {return DEV_NOT_FAUND;}

//  if(Size > 16)
//  {
//    uint32_t c = Size /16, pos = 0, ost = Size%16;
//    for(uint32_t i = 0;i <= c;i++)
//    {
//      HAL_StatusW = ll_Mem_WriteBytes(I2C1, Start_EEPR_Addr, addressOfSettings + pos, charPtr_settings + pos, 16, 1000);
//      pos +=16;
//    }
//    if(ost)
//    {
//      HAL_StatusW = ll_Mem_WriteBytes(I2C1, Start_EEPR_Addr, addressOfSettings + pos, charPtr_settings + pos, Size - pos, 1000);
//    }
//  }else 
//  {
//     HAL_StatusW = ll_Mem_WriteBytes(I2C1, Start_EEPR_Addr, addressOfSettings, charPtr_settings, Size, 1000);
//  }
//  HAL_Delay(1);
//  HAL_StatusR = ll_Mem_ReadBytes(I2C1, Start_EEPR_Addr, addressOfSettings, ptr_test, Size, 1000);

  //osMutexRelease(MutexMemHandle);
  
  if(HAL_StatusW != HAL_OK){
    E_Stat = EEPR_ERR_W;
  }
  
  if(HAL_StatusR != HAL_OK){
    E_Stat = EEPR_ERR_R;
  }

  for(int i=0;i<Size;i++)
  {
    if(*ptr_test != *charPtr_settings)
      { E_Stat = EEPR_ERR ;}
      ptr_test++;
      charPtr_settings++;
  }
  return E_Stat;
}


EEPR_State Read_EEPROM(settings_t *settings)
{
  uint32_t Size = sizeof(*settings);
  uint8_t *charPtr_settings = (uint8_t*)settings;
  int a = 0, i = 0;
    
  //osMutexWait(MutexMemHandle, osWaitForever);
  
  //Прверка на присутствие устройства
  if(HAL_I2C_IsDeviceReady(&hi2c1,Start_EEPR_Addr,2,500))
  {return DEV_NOT_FAUND;}

  if ( HAL_OK != HAL_I2C_Mem_Read(&hi2c1, Start_EEPR_Addr, addressOfSettings, I2C_MEMADD_SIZE_8BIT, (uint8_t *)settings, Size, 1000))
	{
		return EEPR_ERR_R ;
	}

//	if(ll_IsDeviceReady(I2C1, Start_EEPR_Addr, 2, 500))
//	{return DEV_NOT_FAUND;}

//	if ( ll_I2C_OK != ll_Mem_ReadBytes(I2C1, Start_EEPR_Addr, 0, (uint8_t *)settings, Size, 1000))
//	{
//		return EEPR_ERR_R ;
//	}
  //osMutexRelease(MutexMemHandle);
  
  // проверка на пустую память
  for(i=0;i<Size;i++)
  {
    if(0xFF == *charPtr_settings){ a++;}
    else if(0x00 == *charPtr_settings){ a++;}
    charPtr_settings++;
  }

  if(a >= i){return EEPR_Empty;}

return EEPR_OK;
}
	
