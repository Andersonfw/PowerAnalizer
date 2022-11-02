//***************** (C) COPYRIGHT 2022 Anderson F.W *****************
/// @brief		Header de inicialização do sistema
/// @file 		config.c
/// @author		Anderson F.W
/// @version	V1.00
/// @date		28/10/2022
// *****************************************************************************

// Prevenção contra inclusão recursiva -----------------------------------------
#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

// Includes --------------------------------------------------------------------
#include "stm32l4xx_hal.h"
#include "user_5110.h"
#include <stdlib.h>
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "BMP180.h"
#include <string.h>


// Exported types --------------------------------------------------------------
enum
{
	EVENT= 1,
	COMAND = 2
};

typedef struct
{
	float fFilterOutput;
} SAD_FILTER;

enum
{
	VIN,
	VOUT,
	HALL,
	SHUNT,
	TEMP,
	VREF,
	V_DAC,
	NUM_AD_CHANNELS
};

// Exported constants ----------------------------------------------------------
#define UART_BUFFER_SIZE 	 120
#define REFRESH_OUTPUT 		 500
#define REFRESH_DISPLAY 	 500
#define UART_DATA_TIME		 500
#define NAME_BL		"POWERANALYZER\r\n"
#define UART_BL		"9600,0,0\r\n"
#define PSW_BL		"1234\r\n"
#define GET_TEMPBMP	"GETEMPBMP"
#define GET_VIN		"GETVIN"
#define GET_IS		"GETIS"
#define GET_IH		"GETIH"
#define GET_PS		"GETPS"
#define GET_PH		"GETPH"
#define FW_VERSION	"1.2"


// Porta A
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

// Porta B
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

// Porta C
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_0
#define LCD_RST_GPIO_Port GPIOC
#define LCD_CE_Pin GPIO_PIN_1
#define LCD_CE_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOC
#define BLE_CMD_Pin GPIO_PIN_6
#define BLE_CMD_GPIO_Port GPIOC
#define BLE_PWR_Pin GPIO_PIN_8
#define BLE_PWR_GPIO_Port GPIOC



#define BUTTON_EXTI_IRQn EXTI15_10_IRQn


// Exported macro --------------------------------------------------------------
#define BluCmd(_a)		HAL_GPIO_WritePin(BLE_CMD_GPIO_Port, BLE_CMD_Pin, (GPIO_PinState)_a)
#define BluPwr(_a)		HAL_GPIO_WritePin(BLE_PWR_GPIO_Port, BLE_PWR_Pin, (GPIO_PinState)_a)

// Exported functions ----------------------------------------------------------
void ConfigRCC(void);
void ConfigPeriph(void);
void ConfigIO(void);
void ConfigUSART2(void);
void ConfigDMA(void);
void ConfigUART4(void);
void ConfigI2C(void);
void ConfigTIM(void);
void ConfigADC1(void);
void ConfigDAC1(void);
void ConfigSPI2(void);
void ConfigADC2(void);
uint8_t Config_Bt(void);
void Error_Handler(void);

uint8_t* IsUartNewData(void);
void Uart2Write(char *ps8String, uint8_t u8Len);
void CheckCharMatch(UART_HandleTypeDef *hUart);
uint32_t TimeDiff(uint32_t u32OldTime);
uint16_t ReadAnalogIn(uint8_t u8Channel);
uint16_t GetUartRxDataSize(void);
uint16_t UartGetData(uint16_t u16Size, uint8_t *u8BufGetdata);
void SetDAC(uint16_t u32value);
void Uart4Receive(char *ps8String, uint8_t u8Len);
void Uart4Send(char *ps8String, uint8_t u8Len);


#endif

//********* (C) COPYRIGHT 2022 Ânderson F.W *****END OF FILE*********
