/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_5110.h"
#include "onewire.h"
#include "ds18b20.h"
#include <stdlib.h>
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include <stdint.h>
#include "BMP180.h"
#include <string.h>
#include "crc.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Extern variables*/
extern OneWire_t OneWire;


/* Private define ------------------------------------------------------------*/
#define UART_BUFFER_SIZE 	 30
#define REFRESH_OUTPUT 		 500
#define REFRESH_DISPLAY 	 500
#define UART_DATA_TIME		 500
#define NAME_BL		"PT-100\r\n"
#define UART_BL		"9600,0,0\r\n"
#define PSW_BL		"1234\r\n"
#define SET_TEMP	"SETEMP"
#define GET_TEMPIN	"GETEMPIN"
#define GET_TEMPOUT	"GETEMPOUT"
#define GET_PRES	"GETPRES"
#define GET_VOLTAGE_10	"GETVOLTAGE"
#define GET_VOLTAGE_AD	"GETAD"
#define NACK_RESPONSE		'0'				// resposta a um comando invalido
#define ACK_RESPONSE		'1'				// resposta a um c
#define FW_VERSION			"1.2"

#define VREFCAL_ADDRESS		0x1FFF75AA	// endereço da calibração de tensão da referência interna
#define T30					30.00f			// Valor da temperatura utilizada na calibração 1
#define T110				110.0f			// Valor da temperatura utilizada na calibração 2
#define TS30				((uint16_t*)((uint32_t)0x1FFF75A8))	// endereço da calibração de temperatura 1 da referência interna
#define TS110				((uint16_t*)((uint32_t)0x1FFF75CA))	// endereço da calibração de temperatura 2 da referência interna
#define TEMP_COMP			30.0f		// Valor de temperatura default
#define AD_MAX				4095.0f		// valor máximo do conversor AD (12 bits)
#define VDDA_REF			3000.0f		// tensão de calibração da referência
#define VAD_FILTER_SIZE		100			// número de termos dos filtros do conversor AD
#define VAD_SAMPLE_TIME		10000		// intervalo de leitura do conversor AD (ms)
#define VMAX_PT100			10000.0f
#define REFRESH_AD			500
#define REFRESH_TEMP		2000
#define R1					66950
#define R2					30050
//#define CORRECTION			0.913119f
#define CORRECTION			1

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
	INPUT,
	VREF,
	TEMP,
	NUM_AD_CHANNELS
};

/* Private macro -------------------------------------------------------------*/
uint16_t u16BufIndex;						//Index do Buffer RX DMA
uint8_t u8BufRxUartData[UART_BUFFER_SIZE];	//Bufeer RX DMA
static uint32_t u32TimeBuff;				//Atualizção de dados no RX DMA
uint8_t u8BufUart4Data[30];					//Buffer da UART BLE
uint8_t u8BufUart2Data[30];					//Buffer da UART USB
char Buff_Display[120];						//Buffer para escrita display
char bufferUart2Tx[120];					//Buffer de transmissão Bluetooth
char bufferUart2Rx[120];					//Buffer de Recepção Bluetooth
bool BreadTemp = false;						//Flag de leitura da temperatura ONEWIRE
float fSet_Temp = 25;						//Valor de temperatura de controle
uint32_t u32RefreschDisplay;				//Atualização do display
uint32_t u32TimerOutput;					//tempo de controle da sáida
float ftempIn;								//Temperatura BMP-180
float press;								//Pressão BM-180
uint8_t u8IndexUart2 = 0;
uint32_t u32KeyTime;
bool bNewData;
uint8_t u8ProtocolI2c[4] = {0};
uint8_t u8RxI2C[100] = {0};
uint8_t u8TxI2C[100] = {0};
uint8_t u8SizeTx = 0;
uint8_t u8SizeRx = 0;
uint8_t u8BuffBluPy [100];
uint8_t u8SizeBluPy;
bool bFlagSize = false;
bool bFlagRxDataBLE = false;
uint8_t sizeBLETransmit = 0;
uint8_t u8BuffBLEWriteI2C[50] ={0};
uint8_t u8DataReceiveBLEPy[5];
uint8_t u8IndexBLEWrite=0;
uint8_t u8DebugBuff [10] = {0XFF};

uint16_t au16ADRawValue[NUM_AD_CHANNELS];
float fConvert_volt = 0;
float fConvert_temp = 0;
float fNew_temp = 0;
uint32_t u32timeTemp = 0;
float  as16Vad[NUM_AD_CHANNELS];
int16_t as16Filter[NUM_AD_CHANNELS];
SAD_FILTER adc[NUM_AD_CHANNELS];
uint8_t u8count = 0;
uint32_t u32time_means = 0;
float fmeans;
float fcurrent = 0;

void ReadTemp(void);
uint8_t Config_Bt(void);
uint8_t Manage_Data_Blue(uint8_t *u8Data);
uint16_t GetUartRxDataSize(void);
uint16_t UartGetData(uint16_t u16Size, uint8_t *u8BufGetdata);
uint32_t TimeDiff(uint32_t u32OldTime);
void PinControl(void);
void TaskDisplay(void);
void UartData (void);
void TaskKey (void);
void SendSPI(uint8_t* u8data, uint8_t u8size);
void ReceiveSPI(uint8_t* u8data, uint8_t u8size);
void FwVersion(void);
void RxProcess(void);
void OutputControl(uint8_t u8outNumber, bool bAction);
void GetIn(uint8_t u8INNumber);
void GetTemp(uint8_t u8NumberTemp);
void CommandResponse(uint8_t u8Response);
void GetPress(void);

uint16_t ReadAnalogIn(uint8_t u8Channel);
void ReadAD(void);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	F4_CS(SET);
	HAL_GPIO_WritePin(BLE_PWR_GPIO_Port, BLE_PWR_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BLE_CMD_GPIO_Port, BLE_CMD_Pin, GPIO_PIN_SET);
	MX_USART2_UART_Init();
	MX_SPI2_Init();
	MX_UART4_Init();
	BluPwr(true);
	MX_I2C1_Init();
	MX_CRC_Init();
	MX_TIM6_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_Write_String(0,0,"INICIALIZANDO");
	LCD_Write_String(0,1,"SISTEMA...");
	LCD_Write_String(0,4,"AGUARDE.....");
	BMP180_Start();

	/*
    Ds18b20_Init();		//Inicia sensor OneWire
    Ds18b20_ManualConvert();
    DS18B20_Start(&OneWire, ds18b20[0].Address);
    BMP180_Start(); //Inicia sensor BMP-180
*/
	HAL_Delay(3000);
    if(Config_Bt() != HAL_OK)			//Configura nome, uart e PSW do bluetooth
  	  {
  		  BluPwr(false);
  		  HAL_Delay(100);
  		  BluPwr(true);
  		  HAL_Delay(2000);
  		  if(Config_Bt() != HAL_OK)
  			  HAL_NVIC_SystemReset();
  	  }

    HAL_UART_Receive_IT(&huart2, (uint8_t*)bufferUart2Rx, 1);		//Inicia a recepção UART2




	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		ReadTemp();
		TaskDisplay();
		UartData ();
		TaskKey();
		ReadAD();
		if(TimeDiff(u32time_means) > 10)
		{
			fmeans += (as16Vad[0]*(R1+R2))/R2 * CORRECTION;
			u32time_means = HAL_GetTick();
			u8count++;
			if (u8count == 100)
			{
				u8count = 0;
				fConvert_volt = fmeans/100;  //pega a média de 100 valores lidos pelo AD dentro de 1 segundo
				fcurrent = ((fConvert_volt/10)*16.0) + 4000.0 ;
				fmeans = 0;
			}
		}
		fNew_temp = fConvert_volt *100.0 / VMAX_PT100;
		if(fNew_temp != fConvert_temp)
		{
			if(TimeDiff(u32timeTemp) > REFRESH_TEMP) //Aguarda tempo para que a temperatura esteja estavél para então atualizar o valor no display
			{
				u32timeTemp = HAL_GetTick();
				fConvert_temp = fNew_temp;
			}
		}
		else
			u32timeTemp = HAL_GetTick();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

// *****************************************************************************
/// @brief		Lê as entradas analógicas
/// @fn			bool ReadAD(void)
/// @retval		TRUE se há dados disponíveis
// *****************************************************************************

void ReadAD(void)
{
	uint8_t n;
	SAD_FILTER * psAdFilter;
	static uint32_t u32ReadADTimer;
	float fixas;
	float ADvalue;
	float middle;

	if (TimeDiff(u32ReadADTimer) > REFRESH_AD)
	{
		u32ReadADTimer = HAL_GetTick();
		for (n = INPUT; n < NUM_AD_CHANNELS; n++)
		{
			psAdFilter = &adc[n];
			as16Filter[n] = ReadAnalogIn(n);
			psAdFilter->fFilterOutput = psAdFilter->fFilterOutput * 0.05f + as16Filter[n] * 0.95f;
		}

		as16Vad[VREF] =   VDDA_REF * (*(int16_t*)VREFCAL_ADDRESS) / adc[VREF].fFilterOutput;

		as16Vad[INPUT] = (adc[INPUT].fFilterOutput * as16Vad[VREF] / AD_MAX);

		fixas = ((T110 - T30 ) / ((uint32_t)*TS110 - (uint32_t)*TS30));
		ADvalue = (adc[TEMP].fFilterOutput * as16Vad[VREF] / VDDA_REF);
		middle = fixas * (ADvalue - (uint32_t)*TS30);
		as16Vad[TEMP] = middle + TEMP_COMP;

	}
}
// *****************************************************************************
/// @brief		Lê o valor de um canal analógico
/// @fn			uint16_t ReadAnalogIn(uint8_t u8Channel)
/// @param[in]	u8Channel			@brief Canal a ser lido
// *****************************************************************************
uint16_t ReadAnalogIn(uint8_t u8Channel)
{
	return au16ADRawValue[u8Channel];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hAdc)
{
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.GeneratingPolynomial = 79764919;
	hcrc.Init.CRCLength = CRC_POLYLENGTH_32B;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x10909CEC;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10909CEC;
	hi2c2.Init.OwnAddress1 = 128;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 79;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 38400;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	/* DMA2_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin|BLE_CMD_Pin
			|BLE_PWR_Pin|ds18b20_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(F4_CS_GPIO_Port, F4_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RELE1_Pin|RELE2_Pin|RELE3_Pin|RELE4_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RST_Pin LCD_CE_Pin LCD_DC_Pin BLE_CMD_Pin
                           BLE_PWR_Pin */
	GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin|BLE_CMD_Pin
			|BLE_PWR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_GREEN_Pin */
	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : F4_CS_Pin */
	GPIO_InitStruct.Pin = F4_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(F4_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RELE1_Pin RELE2_Pin RELE3_Pin RELE4_Pin */
	GPIO_InitStruct.Pin = RELE1_Pin|RELE2_Pin|RELE3_Pin|RELE4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ds18b20_Pin */
	GPIO_InitStruct.Pin = ds18b20_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ds18b20_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : IN1_Pin IN2_Pin */
	GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV128;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = NUM_AD_CHANNELS;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);


	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)au16ADRawValue, NUM_AD_CHANNELS);
	/* USER CODE END ADC1_Init 2 */

}

/* USER CODE BEGIN 4 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(!TransferDirection)
		HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, u8RxI2C, 3, I2C_FIRST_FRAME);
	else
		HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, u8TxI2C, u8SizeTx, I2C_FIRST_FRAME);
}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}
void GetDataBLE()
{
	u8DebugBuff[u8IndexBLEWrite] = u8DataReceiveBLEPy[0];
	if(u8DataReceiveBLEPy[0]== '\n')
	{
		u8BuffBLEWriteI2C[u8IndexBLEWrite++] = '\n';
		HAL_UART_Transmit(&huart4, (uint8_t *)u8BuffBLEWriteI2C, u8IndexBLEWrite,10);
		u8IndexBLEWrite = 0;

	}
	else
	{
		u8BuffBLEWriteI2C[u8IndexBLEWrite++] = u8DataReceiveBLEPy[0];
	}
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	bNewData = true;
	if(bFlagSize)
	{
		bFlagSize = false;
		GetDataBLE();
	}
	else
		RxProcess();
}
void CommandResponse(uint8_t u8Response)
{
	u8SizeTx = sprintf((char*)u8TxI2C,"%c",u8Response);
}
void StatusOutputs(void)
{
	u8SizeTx = 4;
	u8TxI2C[0] = !HAL_GPIO_ReadPin(RELE1_GPIO_Port, RELE1_Pin);
	u8TxI2C[1] = !HAL_GPIO_ReadPin(RELE1_GPIO_Port, RELE2_Pin);
	u8TxI2C[2] = !HAL_GPIO_ReadPin(RELE1_GPIO_Port, RELE3_Pin);
	u8TxI2C[3] = !HAL_GPIO_ReadPin(RELE1_GPIO_Port, RELE4_Pin);
}
void OutputControl(uint8_t u8outNumber, bool bAction)
{
	switch(u8outNumber)
	{
	case 1:
		//			if(!HAL_GPIO_ReadPin(RELE1_GPIO_Port, RELE1_Pin))
		//				break;
		rele1(bAction);
		break;
	case 2:
		//			if(!HAL_GPIO_ReadPin(RELE2_GPIO_Port, RELE2_Pin))
		//				break;
		rele2(bAction);
		break;
	case 3:
		//			if(!HAL_GPIO_ReadPin(RELE3_GPIO_Port, RELE3_Pin))
		//				break;
		rele3(bAction);
		break;
	case 4:
		//			if(!HAL_GPIO_ReadPin(RELE4_GPIO_Port, RELE4_Pin))
		//				break;
		rele4(bAction);
		break;
	}
	CommandResponse(ACK_RESPONSE);						// informa mensagem recebida
}
void GetIn(uint8_t u8INNumber)
{
	if(u8INNumber == 1)
	{
		u8TxI2C[0] = IN(IN1_Pin);
		//sprintf((char*)u8TxI2C,"%d",IN(IN1_Pin));
	}
	else if(u8INNumber == 2)
	{
		u8TxI2C[0] = IN(IN2_Pin);
		//sprintf((char*)u8TxI2C,"%d",IN(IN2_Pin));
	}

	u8SizeTx = 1;
}
void GetTemp(uint8_t u8NumberTemp)
{
	if(u8NumberTemp == 1)
	{
		//snprintf((char*)u8TxI2C,3,"%d",(uint8_t)ds18b20[0].Temperature);
		snprintf((char*)u8TxI2C,6,"%.2f",ds18b20[0].Temperature);
	}
	else if(u8NumberTemp == 2)
	{
		//snprintf((char*)u8TxI2C,3,"%d",(uint8_t)ftempIn);
		snprintf((char*)u8TxI2C,6,"%.2f",ftempIn);
	}

	u8SizeTx = 6;
}
void GetPress(void)
{

	snprintf((char*)u8TxI2C,7,"%.2f",press);
	u8SizeTx = 6;
}
void FwVersion(void)
{
	u8SizeTx = sprintf((char*)u8TxI2C,"%s",FW_VERSION);
}
/* *****************************************************************************
/// @brief		Controla a recep��o serial
/// @fn			void RxProcess(void)
// *****************************************************************************/
void RxProcess(void)
{
	uint8_t u8Data;

	if (bNewData == true)
	{
		bNewData = false;
		u8Data = u8RxI2C[COMAND] - '0';
		switch (u8RxI2C[EVENT])				// verifica o comando recebido
		{
		case 'V':											// informa a versão de fimware
			FwVersion();
			break;
		case 'L':											// liga o rele
			OutputControl(u8Data, true);
			break;
		case 'D':											// desliga o rele
			OutputControl(u8Data, false);
			break;
		case 'I':											// le entrada
			GetIn(u8Data);
			break;
		case 'T':											// leitura da vari�veis ambientais
			GetTemp(u8Data);
			break;
		case 'P':											// leitura da vari�veis ambientais
			GetPress();
			break;
		case 'S':											// status o rele
			StatusOutputs();
			break;
		case 'B':											// leitura da vari�veis ambientais
			if(u8Data == ('S' - '0'))
			{
				u8SizeTx = 1;
				u8TxI2C[0] = u8SizeBluPy;
			}
			if(u8Data == ('R' - '0'))
			{
				memcpy(u8TxI2C,u8BuffBluPy,u8SizeBluPy);
				u8SizeTx = u8SizeBluPy;
			}
			if(u8Data == ('W' - '0'))
			{
				bFlagSize = true;
				HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, u8DataReceiveBLEPy, 1, I2C_FIRST_FRAME);
			}
			break;
		default:
			__HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_ADDR);
			CommandResponse(ACK_RESPONSE);					// comando invalido
		}
	}
}
void SendSPI(uint8_t* u8data, uint8_t u8size)
{

	//uint8_t u8buff[150];
	//
	//	crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)u8dataSPI, 1);
	//	crc8 = crc8x_fast(u8dataSPI, 4, CRC8Table[10]);
	//	crc16 = CRC16(u8dataSPI, 4, CRC16Table[10]);
	//	sprintf((char*)u8buff,"ENVIANDO DADOS:\r\nCRC:%lu \r\nCRC8=%lu \r\nCRC16=%lu\r\nPAYLOAD:%s\r\n",crc32,crc8,crc16,u8dataSPI);
	//	HAL_UART_Transmit(&huart2, u8buff, strlen((char*)u8buff), 100);
	//
	//	memset(&u8BufTXSPI,'0',sizeof(u8BufTXSPI));
	//	memcpy(&u8BufTXSPI[0],&crc32,4);
	//	memcpy(&u8BufTXSPI[4],&crc16,2);
	//	memcpy(&u8BufTXSPI[6],&crc8,1);
	//	memcpy(&u8BufTXSPI[7],u8data,u8size);
	//
	//	F4_CS(RESET);
	//	HAL_SPI_Transmit(&hspi2, u8BufTXSPI, u8size + 7,100);
	//	F4_CS(SET);
}
void ReceiveSPI(uint8_t* u8data, uint8_t u8size)
{
	//	F4_CS(RESET);
	//	memset(u8data,'0',sizeof(*u8data));
	//	HAL_SPI_Receive_IT(&hspi2, u8data, u8size);

}
void PinControl(void)
{
	if(TimeDiff(u32TimerOutput) > REFRESH_OUTPUT)
	{
		u32TimerOutput = HAL_GetTick();
		if(ds18b20[0].Temperature <= fSet_Temp)
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

		else
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	}
}
void TaskDisplay(void)
{
	if(TimeDiff(u32RefreschDisplay) > REFRESH_DISPLAY )
	{
		u32RefreschDisplay = HAL_GetTick();
		LCD_Clear();
		LCD_Write_String(15,0,"LEITURAS");
		sprintf(Buff_Display,"T-BMP: %.2fC",ftempIn);
		LCD_Write_String(0,1,Buff_Display);
		sprintf(Buff_Display,"PT100:%.1fC",fConvert_temp);
		LCD_Write_String(0,2,Buff_Display);
		sprintf(Buff_Display,"V_AD:%.2fmV",as16Vad[0]);
		LCD_Write_String(0,3,Buff_Display);
		sprintf(Buff_Display,"V_O:%.1fmV",fConvert_volt);
		LCD_Write_String(0,4,Buff_Display);
		sprintf(Buff_Display,"[I]:%.2fuA",fcurrent);
		LCD_Write_String(0,5,Buff_Display);
	}
}
void TaskKey(void)
{

	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
	{
		if(TimeDiff(u32KeyTime) > 500)
		{
			fSet_Temp +=5;
			u32KeyTime = HAL_GetTick();
			if(fSet_Temp > 100)
				fSet_Temp = 25;
		}
	}
	else
		u32KeyTime = HAL_GetTick();
}
void UartData (void)
{
	uint8_t u8size;
	if(TimeDiff(u32TimeBuff)> UART_DATA_TIME)
	{
		u32TimeBuff = HAL_GetTick();
		u8size = GetUartRxDataSize();
		if(u8size)
		{
			UartGetData(u8size, (uint8_t*)u8BufUart4Data);
			Manage_Data_Blue(u8BufUart4Data);
		}
	}
}
uint8_t Config_Bt(void)
{
	uint8_t u8BuffRxConfig[30];
	uint8_t u8size;


	u8size = sprintf((char*)u8BufUart4Data,"AT+UART?\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)u8BufUart4Data, u8size,10);
	HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
	if(strstr((char*)u8BuffRxConfig,(char*)UART_BL) == NULL)
	{
		memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
		u8size = sprintf((char*)u8BufUart4Data,"AT+UART=%s",UART_BL);
		HAL_UART_Transmit(&huart4, (uint8_t *)u8BufUart4Data, u8size,10);
		HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
		if(!strcmp((char*)u8BuffRxConfig,(char*)"OK\r\n"))
		{
			return HAL_ERROR;
		}
	}

	memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
	u8size = sprintf((char*)u8BufUart4Data,"AT+NAME?\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)u8BufUart4Data, u8size,10);
	HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
	if(strstr((char*)u8BuffRxConfig,(char*)NAME_BL) == NULL)
	{
		memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
		u8size = sprintf((char*)u8BufUart4Data,"AT+NAME=%s",NAME_BL);
		HAL_UART_Transmit(&huart4, (uint8_t *)u8BufUart4Data, u8size,10);
		HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
		if(strcmp((char*)u8BuffRxConfig,(char*)"OK\r\n"))
		{
			return HAL_ERROR;
		}
	}

	memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
	u8size = sprintf((char*)u8BufUart4Data,"AT+PSWD?\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)u8BufUart4Data, u8size,10);
	HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
	if(strstr((char*)u8BuffRxConfig,(char*)PSW_BL) == NULL)
	{
		memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
		u8size = sprintf((char*)u8BufUart4Data,"AT+PSWD=%s",PSW_BL);
		HAL_UART_Transmit(&huart4, (uint8_t *)u8BufUart4Data, u8size,10);
		HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
		if(strcmp((char*)u8BuffRxConfig,(char*)"OK\r\n"))
		{
			return HAL_ERROR;
		}
	}
	BluPwr(false);
	BluCmd(false);
	HAL_Delay(100);
	BluPwr(true);
	huart4.Init.BaudRate = 9600;
	HAL_UART_Init(&huart4);
	UART_Start_Receive_DMA(&huart4,u8BufRxUartData,UART_BUFFER_SIZE);

	return HAL_OK;
}
uint8_t Manage_Data_Blue(uint8_t *u8Data)
{
	char identify;
	char *p;
	char cmd[10];
	char parametro[10];

	if(strstr((char*)u8Data,(char*)"AT+") == NULL)
		goto ERROR;

	p = strchr((char*)u8Data,'=');
	if(p == NULL)
		p = strchr((char*)u8Data,'?');
	if(p == NULL)
		goto ERROR;
	identify = p[0];
	p = strtok ((char*)u8Data,"+=?");
	p = strtok (NULL,"+=?");
	if(p != NULL)
		strcpy(cmd,p);
	else
		goto ERROR;
	if(identify == '=')
		p = strtok (NULL,"=");
	else
		p = strtok (NULL,"?");
	if(p != NULL)
		strcpy(parametro,p);
	else
		goto ERROR;

	if(identify == '=')
	{
		if(!strcmp(cmd, (char*)SET_TEMP))
		{
			fSet_Temp = atof(parametro);
			sprintf(bufferUart2Tx,"SETADO PARAMETRO %.2fC\r\n",fSet_Temp);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
		}
	}

	if(identify == '?')
	{
		if(!strcmp(cmd, (char*)GET_TEMPOUT))
		{
			sprintf(bufferUart2Tx,"SOLICITADO TEMP. PT-100 %.2fC\r\n",fConvert_temp);

			u8SizeBluPy=sprintf((char*)u8BuffBluPy,"SOLICITADO TEMPOUT %.2fC\r\n",ds18b20[0].Temperature);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
			HAL_UART_Transmit_IT(&huart4, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
		}
		if(!strcmp(cmd, (char*)GET_VOLTAGE_10))
		{
			sprintf(bufferUart2Tx,"SOLICITADO TENSÃO 0 a 10 -> %.2fmV\r\n",fConvert_volt);

			u8SizeBluPy=sprintf((char*)u8BuffBluPy,"SOLICITADO TEMPOUT %.2fC\r\n",ds18b20[0].Temperature);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
			HAL_UART_Transmit_IT(&huart4, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
		}
		if(!strcmp(cmd, (char*)GET_VOLTAGE_AD))
		{
			sprintf(bufferUart2Tx,"SOLICITADO TENSÃO AD -> %.2fmV\r\n",as16Vad[0]);

			u8SizeBluPy=sprintf((char*)u8BuffBluPy,"SOLICITADO TEMPOUT %.2fC\r\n",ds18b20[0].Temperature);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
			HAL_UART_Transmit_IT(&huart4, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
		}
		if(!strcmp(cmd, (char*)GET_TEMPIN))
		{
			u8SizeBluPy=sprintf((char*)u8BuffBluPy,"SOLICITADO TEM. BMP %.2fC\r\n",ftempIn);
			sprintf(bufferUart2Tx,"SOLICITADO TEM. BMP %.2fC\r\n",ftempIn);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
			HAL_UART_Transmit_IT(&huart4, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
		}
		if(!strcmp(cmd, (char*)GET_PRES))
		{
			u8SizeBluPy=sprintf((char*)u8BuffBluPy,"SOLICITADO PRES BMP %.2fhPa\r\n",press);
			sprintf(bufferUart2Tx,"SOLICITADO PRES %.2fhPa\r\n",press);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
			HAL_UART_Transmit_IT(&huart4, (uint8_t*)bufferUart2Tx,strlen(bufferUart2Tx));
		}

	}
	return 0;

	ERROR:
	return HAL_ERROR;

}
void ReadTemp(void)
{
	//static uint32_t u32TimeTemp;

//	if(!BreadTemp)
//	{
//		DS18B20_Start(&OneWire, ds18b20[0].Address);
//		BreadTemp = true;
		ftempIn = BMP180_GetTemp() / (float)1000; //LENDO TEMPERATURA
		press = BMP180_GetPress (0) / (float)100; //LENDO PRESSAO
//		u32TimeTemp = HAL_GetTick();
//	}
//	if(DS18B20_AllDone(&OneWire))
//	{
//		ds18b20[0].DataIsValid = DS18B20_Read(&OneWire, ds18b20[0].Address, &ds18b20[0].Temperature);
//		BreadTemp = true;
//	}
//	if(TimeDiff(u32TimeTemp) > 200)
//	{
//		BreadTemp = false;
//	}
}
/* USER CODE BEGIN 4 */
// *****************************************************************************
/// @brief		Calcula o número de bytes disponíveis para leitura
/// @fn			uint16_t GetUartRxDataSize(void)
/// @retval		Número de bytes
// *****************************************************************************
uint16_t GetUartRxDataSize(void)
{
	int16_t u16Size, u16UartRxIndex;

	u16UartRxIndex = UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
	u16Size = u16UartRxIndex - u16BufIndex;

	if (u16Size < 0)
		u16Size = UART_BUFFER_SIZE - u16BufIndex + u16UartRxIndex;
	if (u16Size > UART_BUFFER_SIZE)
		u16Size = UART_BUFFER_SIZE;
	return u16Size;
}

// *****************************************************************************
/// @brief		Lê os dados recebidos pela UART
/// @fn			uint16_t UartGetData(uint16_t u16Size, uint8_t *u8BufGetdata)
/// @param[in]	u16Size		@brief Número de bytes
/// @retval		Número de bytes
// *****************************************************************************
uint16_t UartGetData(uint16_t u16Size, uint8_t *u8BufGetdata)
{
	uint16_t n;
	uint16_t u16DataSize = GetUartRxDataSize();


	for (n = 0; n < u16DataSize; n++)
	{
		//au8UartRxRdBuffer[u8Uart][n] = au8UartRxWrBuffer[u8Uart][as16UartRxRdIndex[u8Uart]];
		u8BufGetdata[n] = u8BufRxUartData[u16BufIndex];

		u16BufIndex += 1;

		if (u16BufIndex >= UART_BUFFER_SIZE)
			u16BufIndex = 0;
	}
	return true;
}
// *****************************************************************************
/// @brief		Retorna o período de tempo entre uma variável e o relógio do sistema
/// @fn			uint32_tTimeDiff(void)
/// @retval		u32ElapsedTime @brief Período de tempo decorrido (ms)
// *****************************************************************************
uint32_t TimeDiff(uint32_t u32OldTime)
{
	uint32_t u32Now, u32ElapsedTime;

	u32Now = HAL_GetTick();
	if (u32Now >= u32OldTime)
		u32ElapsedTime = u32Now - u32OldTime;
	else
		u32ElapsedTime = 0xFFFFFFFF - u32OldTime + u32Now;
	return u32ElapsedTime;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART2)
	{
		if(bufferUart2Rx[0] != '\n')
		{
			u8BufUart2Data[u8IndexUart2] = bufferUart2Rx[0];
			u8IndexUart2 ++;
		}
		else
		{
			u8IndexUart2 = 0;
			Manage_Data_Blue((uint8_t*)u8BufUart2Data);

		}
		HAL_UART_Receive_IT(&huart2, (uint8_t*)bufferUart2Rx, 1);
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE)
	{
		HAL_UART_MspDeInit(huart);

		if(huart->Instance == UART4)
			MX_UART4_Init();
		if(huart->Instance == USART2)
			MX_USART2_UART_Init();
	}
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(HAL_I2C_GetError(hi2c) != HAL_SPI_ERROR_NONE)
	{
		HAL_I2C_MspDeInit(hi2c);
		if(hi2c->Instance == I2C1)
			MX_I2C1_Init();
		if(hi2c->Instance == I2C2)
		{
			MX_I2C2_Init();
			HAL_I2C_EnableListen_IT(&hi2c2);
		}
	}
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{

	if(HAL_SPI_GetError(hspi) != HAL_SPI_ERROR_NONE)
	{
		HAL_SPI_MspDeInit(hspi);
		MX_SPI2_Init();
	}

}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI2)
	{
	}

}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI2)
	{
	}
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM7) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
