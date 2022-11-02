//***************** (C) COPYRIGHT 2022 Anderson F.W *****************
/// @brief		Arquivos de inicialização do sistema
/// @file 		config.c
/// @author		Anderson F.W
/// @version	V1.00
/// @date		28/10/2022
// *****************************************************************************

// Includes --------------------------------------------------------------------
#include <string.h>
#include <time.h>
#include "config.h"

// Private typedef -------------------------------------------------------------

// Private define --------------------------------------------------------------

// Private macro ---------------------------------------------------------------

// Private variables -----------------------------------------------------------
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

uint16_t u16BufIndex;						//Index do Buffer RX DMA
uint8_t u8BufBLE_Tx[UART_BUFFER_SIZE];		//Bufeer RX DMA
uint8_t u8BufBLE_Rx[UART_BUFFER_SIZE];					//Buffer da UART BLE

uint8_t vu8Index;							//Index da recepção serial COM
uint8_t u8Uart2Tx[UART_BUFFER_SIZE];		//Buffer de transmissão COM
uint8_t u8Uart2Rx[UART_BUFFER_SIZE];		//Buffer de Recepção COM
uint16_t u16Adc_Dac1;						//Buffer para DMA do ADC2 canal DAC1
uint16_t au16ADRawValue[NUM_AD_CHANNELS];	//Buffer para DMA do ADC1


// Private functions -----------------------------------------------------------

// *****************************************************************************
/// @brief		Configura os vários sistemas de clock
/// @fn			void ConfigRCC(void)
// *****************************************************************************
void ConfigRCC(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
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

// *****************************************************************************
/// @brief		Configura periféricos de clock
/// @fn			void ConfigPeriph(void)
// *****************************************************************************
void ConfigPeriph(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

// *****************************************************************************
/// @brief		Configura o conversor AD1
/// @fn			void ConfigADC1(void)
// *****************************************************************************
void ConfigADC1(void)
{

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV128;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 6;
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

	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_16;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_6;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)au16ADRawValue, NUM_AD_CHANNELS - 1);

}

// *****************************************************************************
/// @brief		Configura o conversor AD2
/// @fn			void ConfigADC2(void)
// *****************************************************************************
void ConfigADC2(void)
{

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV128;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_DAC1CH1_ADC2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	//HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&u16Adc_Dac1, 1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&au16ADRawValue[V_DAC], 1);
}

// *****************************************************************************
/// @brief		Configura o conversor DAC1
/// @fn			void ConfigDAC1(void)
// *****************************************************************************
void ConfigDAC1(void)
{
	DAC_ChannelConfTypeDef sConfig = {0};

	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}

// *****************************************************************************
/// @brief		Configura a portas de comunicação I2C
/// @fn			void ConfigI2C(void)
// *****************************************************************************
void ConfigI2C(void)
{

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

	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

// *****************************************************************************
/// @brief		Inicializa a porta de comunicação SPI
/// @fn			void ConfigSPI2(void)
// *****************************************************************************
void ConfigSPI2(void)
{

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
}

// *****************************************************************************
/// @brief		Configura os temporizadores
/// @fn			void ConfigTIM(void)
// *****************************************************************************
void ConfigTIM(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

// *****************************************************************************
/// @brief		Configura a porta de comunicação serial BLE
/// @fn			void ConfigUART4(void)
// *****************************************************************************
void ConfigUART4(void)
{
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
}

// *****************************************************************************
/// @brief		Configura a porta de comunicação serial COM
/// @fn			void ConfigUSART2(void)
// *****************************************************************************
void ConfigUSART2(void)
{

	UART_WakeUpTypeDef sAddressMatch;

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

	sAddressMatch.WakeUpEvent = UART_WAKEUP_ON_ADDRESS;
	sAddressMatch.AddressLength = UART_ADDRESS_DETECT_7B;
	sAddressMatch.Address = '\n';
	HAL_UARTEx_StopModeWakeUpSourceConfig(&huart2, sAddressMatch);

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_CM);
	HAL_UART_Receive_DMA(&huart2, u8Uart2Rx, UART_BUFFER_SIZE);

	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

// *****************************************************************************
// @brief		Inicialização do DMA
/// @fn			void ConfigDMA(void)
// *****************************************************************************
void ConfigDMA(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();


	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

	HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
}

// *****************************************************************************
/// @brief		Configura o BLE HC-05
/// @fn			void Config_Bt(void)
// *****************************************************************************
uint8_t Config_Bt(void)
{
	uint8_t u8BuffRxConfig[30];
	uint8_t u8size;


	u8size = sprintf((char*)u8BufBLE_Tx,"AT+UART?\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)u8BufBLE_Tx, u8size,10);
	HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
	if(strstr((char*)u8BuffRxConfig,(char*)UART_BL) == NULL)
	{
		memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
		u8size = sprintf((char*)u8BufBLE_Tx,"AT+UART=%s",UART_BL);
		HAL_UART_Transmit(&huart4, (uint8_t *)u8BufBLE_Tx, u8size,10);
		HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
		if(!strcmp((char*)u8BuffRxConfig,(char*)"OK\r\n"))
		{
			return HAL_ERROR;
		}
	}

	memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
	u8size = sprintf((char*)u8BufBLE_Tx,"AT+NAME?\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)u8BufBLE_Tx, u8size,10);
	HAL_UART_Receive(&huart4, u8BuffRxConfig, 30,100);
	if(strstr((char*)u8BuffRxConfig,(char*)NAME_BL) == NULL)
	{
		memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
		u8size = sprintf((char*)u8BufBLE_Tx,"AT+NAME=%s",NAME_BL);
		HAL_UART_Transmit(&huart4, (uint8_t *)u8BufBLE_Tx, u8size,10);
		HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
		if(strcmp((char*)u8BuffRxConfig,(char*)"OK\r\n"))
		{
			return HAL_ERROR;
		}
	}

	memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
	u8size = sprintf((char*)u8BufBLE_Tx,"AT+PSWD?\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)u8BufBLE_Tx, u8size,10);
	HAL_UART_Receive(&huart4, u8BuffRxConfig, 20,100);
	if(strstr((char*)u8BuffRxConfig,(char*)PSW_BL) == NULL)
	{
		memset(u8BuffRxConfig,0,sizeof(u8BuffRxConfig));
		u8size = sprintf((char*)u8BufBLE_Tx,"AT+PSWD=%s",PSW_BL);
		HAL_UART_Transmit(&huart4, (uint8_t *)u8BufBLE_Tx, u8size,10);
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
	UART_Start_Receive_DMA(&huart4,u8BufBLE_Rx,UART_BUFFER_SIZE);

	return HAL_OK;
}
// *****************************************************************************
/// @brief		Erro de inicialização
/// @fn			void Error_Handler(void)
// *****************************************************************************
void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}

// *****************************************************************************
/// @brief		Configura as portas de entrada e saída
/// @fn			void ConfigIO(void)
// *****************************************************************************
void ConfigIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin|BLE_CMD_Pin
			|BLE_PWR_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin|BLE_CMD_Pin
			|BLE_PWR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

// *****************************************************************************
/// @brief		Retorna um ponteiro com o buffer de recep��o da UART
/// @fn			u8* IsUartNewData(void)
/// @retval		NULL se n�o h� dados ou ponteiro para os dados?
// *****************************************************************************
uint8_t* IsUartNewData(void)
{

	if (vu8Index != 0)
	{
		vu8Index = 0;
		return u8Uart2Rx;
	}
	return NULL;
}

// *****************************************************************************
/// @brief		Verifica se recebeu o �ltimo caractere do pacote
/// @fn			void CheckCharMatch(UART_HandleTypeDef *hUart)
/// @param[in]	hUart		@brief Ponteiro para a estrutura da interface UART
// *****************************************************************************
void CheckCharMatch(UART_HandleTypeDef *hUart)
{
	if (__HAL_UART_GET_FLAG(hUart, UART_FLAG_CMF) == true)
	{
		__HAL_UART_CLEAR_FLAG(hUart, UART_CLEAR_CMF);
		vu8Index = UART_BUFFER_SIZE - hUart->hdmarx->Instance->CNDTR;
		u8Uart2Rx[vu8Index -1] = '\0';
		HAL_UART_DMAStop(hUart);									//para a recepção UART por DMA
		HAL_UART_Receive_DMA(hUart, u8Uart2Rx, UART_BUFFER_SIZE);		//Ativa a recepção UART por DMA
	}
}

// *****************************************************************************
/// @brief		Escreve dados na USART2
/// @fn			void Uart1Write(char *ps8String)
// *****************************************************************************
void Uart2Write(char *ps8String, uint8_t u8Len)
{
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ps8String, u8Len);
	//HAL_UART_Transmit_IT(&huart2, (uint8_t*)ps8String, u8Len);
}

// *****************************************************************************
/// @brief		Escreve dados na UART4
/// @fn			void Uart4Send(char *ps8String)
// *****************************************************************************
void Uart4Send(char *ps8String, uint8_t u8Len)
{
	HAL_UART_Transmit(&huart4, (uint8_t *)ps8String, u8Len,10);
}

// *****************************************************************************
/// @brief		Recebe dados na UART4
/// @fn			void Uart4Receive(char *ps8String, uint8_t u8Len)
// *****************************************************************************
void Uart4Receive(char *ps8String, uint8_t u8Len)
{
	HAL_UART_Receive(&huart4, (uint8_t *)ps8String, u8Len,100);
}

// *****************************************************************************
/// @brief		Ajusta o valor de saída do DAC
/// @fn			void SetDAC(uint32_t u32value)
// *****************************************************************************
void SetDAC(uint16_t u16value)
{
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, u16value);
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
		u8BufGetdata[n] = u8BufBLE_Rx[u16BufIndex];

		u16BufIndex += 1;

		if (u16BufIndex >= UART_BUFFER_SIZE)
			u16BufIndex = 0;
	}
	return true;
}

//********* (C) COPYRIGHT 2022 Ânderson F.W *****END OF FILE*********
