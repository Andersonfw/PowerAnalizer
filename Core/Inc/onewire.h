//***************** (C) COPYRIGHT 2021 Ânderson F.W *****************
/// @brief
/// @file 		ds18b20.h
/// @author		Ânderson F. Weschenfelder
/// @version	V1.00
/// @date		02/08/21
// *****************************************************************************

// Prevençâo contra inclusão recursiva -----------------------------------------
#ifndef ONEWIRE_H
#define ONEWIRE_H 

// C++ detection
#ifdef __cplusplus
extern "C" {
#endif

// Includes --------------------------------------------------------------------
#include "main.h"
#include "ds18b20Config.h"
#if (_DS18B20_USE_FREERTOS==1)
#include "cmsis_os.h"
#define	OneWireDelay(x)			osDelay(x)
#else
#define	OneWireDelay(x)			HAL_Delay(x)
#endif

// Exported constants ----------------------------------------------------------
/* OneWire commands */
#define ONEWIRE_CMD_RSCRATCHPAD			0xBE
#define ONEWIRE_CMD_WSCRATCHPAD			0x4E
#define ONEWIRE_CMD_CPYSCRATCHPAD		0x48
#define ONEWIRE_CMD_RECEEPROM			0xB8
#define ONEWIRE_CMD_RPWRSUPPLY			0xB4
#define ONEWIRE_CMD_SEARCHROM			0xF0
#define ONEWIRE_CMD_READROM				0x33
#define ONEWIRE_CMD_MATCHROM			0x55
#define ONEWIRE_CMD_SKIPROM				0xCC

// Exported types ----------------------------------------------------------
typedef struct {
	GPIO_TypeDef* GPIOx;           /*!< GPIOx port to be used for I/O functions */
	uint16_t GPIO_Pin;             /*!< GPIO Pin to be used for I/O functions */
	uint8_t LastDiscrepancy;       /*!< Search private */
	uint8_t LastFamilyDiscrepancy; /*!< Search private */
	uint8_t LastDeviceFlag;        /*!< Search private */
	uint8_t ROM_NO[8];             /*!< 8-bytes address of last search device */
} OneWire_t;

// Exported functions ----------------------------------------------------------
void ONEWIRE_DELAY(uint16_t time_us);		// OneWire delay

/* Pin settings */
void ONEWIRE_LOW(OneWire_t *gp);			
void ONEWIRE_HIGH(OneWire_t *gp);		
void ONEWIRE_INPUT(OneWire_t *gp);		
void ONEWIRE_OUTPUT(OneWire_t *gp);		
void OneWire_Init(OneWire_t* OneWireStruct, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t OneWire_Reset(OneWire_t* OneWireStruct);
uint8_t OneWire_ReadByte(OneWire_t* OneWireStruct);
void OneWire_WriteByte(OneWire_t* OneWireStruct, uint8_t byte);
void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit);
uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct);
uint8_t OneWire_Search(OneWire_t* OneWireStruct, uint8_t command);
void OneWire_ResetSearch(OneWire_t* OneWireStruct);
uint8_t OneWire_First(OneWire_t* OneWireStruct);
uint8_t OneWire_Next(OneWire_t* OneWireStruct);
void OneWire_GetFullROM(OneWire_t* OneWireStruct, uint8_t *firstIndex);
void OneWire_Select(OneWire_t* OneWireStruct, uint8_t* addr);
void OneWire_SelectWithPointer(OneWire_t* OneWireStruct, uint8_t* ROM);
uint8_t OneWire_CRC8(uint8_t* addr, uint8_t len);
 
// C++ detection
#ifdef __cplusplus
}
#endif

#endif

//***************** (C) COPYRIGHT 2021 Ânderson F.W *****END OF FILE*********
