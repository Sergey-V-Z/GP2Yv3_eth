#include "main.h"


typedef enum
{
	EEPR_OK,
	EEPR_ERR,
	EEPR_ERR_W,
	EEPR_ERR_R,
	DEV_NOT_FAUND,
	EEPR_Empty
}EEPR_State;

// Распределение данных настроек в EEPROM
#define addressOfSettings      0x01

// структура с настройками
typedef struct
{
  ULONG BaudRate_M;
  ULONG BaudRate;
  UCHAR Role;
  UCHAR SlaveAddress;
  eMBMode mode;
} settings_t;

EEPR_State Write_EEPROM(settings_t settings);
EEPR_State Read_EEPROM(settings_t *settings);
