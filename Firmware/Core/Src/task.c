//***************** (C) COPYRIGHT 2022 Anderson F.W *****************
/// @brief		 Programa principal do PowerAnanlyzer
/// @file 		task.c
/// @author		Anderson F.W
/// @version	V1.00
/// @date		28/10/2022
// *****************************************************************************

/* Includes ------------------------------------------------------------------*/
#include <config.h>

// Private typedef -------------------------------------------------------------

// Private define --------------------------------------------------------------
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
#define VMAX_VOUT			5000.0f
#define REFRESH_AD			500
#define REFRESH_TEMP		2000
#define R1_OUT				66950
#define R2_OUT				30050
#define R1_IN				100250
#define R2_IN				4647
#define CORRECTION			1
#define MAXCURRENT			5000.0f		//Corrente máxima do HALL
#define HALLRESSOLUTION		10.0f		//100mV=1A -> 1mV=10mA no HALL
#define HALLCURRENTZERO		2550.0f		//Valor de tensão para corrente 0 A no HALL

#define NSAMPLE				100			//numero de amostras para fazer a conversão dos ADS

// Private macro ---------------------------------------------------------------

// Private variables -----------------------------------------------------------
uint32_t u32TimeRefreshBLE;				//Atualizção de dados no RX DMA
char Buff_Display[120];							//Buffer para escrita display
uint32_t u32RefreschDisplay;					//Atualização do display
float ftempBMP;									//Temperatura BMP-180
float fpressBMP;								//Pressão BM-180
uint32_t u32KeyTime;

float f_voltageValue[4] = {0};
float fConvert_temp = 0;
float  as16Vad[NUM_AD_CHANNELS];
int16_t as16Filter[NUM_AD_CHANNELS];
SAD_FILTER adc[NUM_AD_CHANNELS];
uint8_t u8countAdMeans = 0;
uint32_t u32timeADmeans = 0;
uint16_t u16ValueDAC = 4095;
uint32_t u32VoutVoltage = 1000;

float f_Vmean[4];
float fcurrent_4_20 = 0;
float fCurrentShunt = 0;
float fCurrentHall = 0;
float fPowerHall = 0;
float fPowerShunt = 0;

extern UART_HandleTypeDef huart4;


// Private function prototypes -------------------------------------------------
void ReadBMP(void);
uint8_t Manage_Data_Blue(uint8_t *u8Data);
void TaskDisplay(void);
void UartData (void);
void TaskKey (void);
void RxProcess(void);
void ReadAD(void);
void VoutControl(uint32_t u32Voltage);
void RxProcess(void);
char* Print(const char *ps8Fmt, ...);
void PowerSystem(void);


// Private functions -----------------------------------------------------------
// *****************************************************************************
/// @brief		Programa principal
/// @fn			main
// *********************************'********************************************
int main(void)
{

	HAL_Init();
	ConfigRCC();
	ConfigPeriph();
	ConfigIO();
	BluPwr(false);
	BluCmd(true);
	ConfigDMA();
	ConfigUSART2();
	ConfigUART4();
	ConfigI2C();
	BluPwr(true);
	ConfigTIM();
	ConfigADC1();
	ConfigDAC1();
	ConfigSPI2();
	ConfigADC2();

	LCD_Init();
	LCD_Write_String(0,0,"INICIALIZANDO");
	LCD_Write_String(0,1,"SISTEMA...");
	LCD_Write_String(0,4,"AGUARDE.....");
	BMP180_Start();

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

	while (1)
	{

		ReadBMP();
		TaskDisplay();
		UartData ();
		TaskKey();
		ReadAD();
		if(TimeDiff(u32timeADmeans) > 10)
		{
			f_Vmean[VOUT] += (as16Vad[VOUT] *(R1_OUT+R2_OUT) )/ R2_OUT;
			f_Vmean[VIN] += (as16Vad[VIN] * (R1_IN+R2_IN)) / R2_IN;
			f_Vmean[HALL] += as16Vad[HALL];
			f_Vmean[SHUNT] += as16Vad[SHUNT];
			u32timeADmeans = HAL_GetTick();
			u8countAdMeans++;
			if (u8countAdMeans == NSAMPLE)
			{
				u8countAdMeans = 0;
				f_voltageValue[VIN] = f_Vmean[VIN]/NSAMPLE - 900;  //pega a média de 100 valores lidos pelo AD dentro de 1 segundo
				f_voltageValue[VOUT] = f_Vmean[VOUT]/NSAMPLE;
				f_voltageValue[HALL] = f_Vmean[HALL]/NSAMPLE;
				f_voltageValue[SHUNT] = f_Vmean[SHUNT]/NSAMPLE;
				fcurrent_4_20 = (fCurrentShunt * 3.2) + 4000.0 ;
				f_Vmean[VIN] = 0;
				f_Vmean[VOUT] = 0;
				f_Vmean[HALL] = 0;
				f_Vmean[SHUNT] = 0;
			}
		}

		PowerSystem();
		RxProcess();
	}
}
// *****************************************************************************
/// @brief		Leitura de corrente e controle da saída
/// @fn			void PowerSystem(void)
/// @retval
// *****************************************************************************
void PowerSystem(void)
{
	static uint32_t u32timeRefresh;
	if(TimeDiff(u32timeRefresh) > 100)
	{
		u32timeRefresh = HAL_GetTick();

		fCurrentHall = (f_voltageValue[HALL] * 2);	//Multiplicação por 2 dada  curva de resposta na saída do AD620
		fCurrentShunt = (f_voltageValue[SHUNT]/0.6083) - 0.03f * fCurrentShunt; //Multiplicação por 0,6 dada  curva de resposta na saída do AD620 e uma compensação de ajuste fino
		VoutControl((uint32_t)fCurrentShunt);

		if(fCurrentHall < 80)		//limite minimo de 80mA para o HALL
			fCurrentHall = 0;

		if(fCurrentShunt < 0)
			fCurrentShunt = 0;
		fPowerHall = fCurrentHall * f_voltageValue[VIN] /1000000;
		fPowerShunt = fCurrentShunt * f_voltageValue[VIN]/1000000;
	}

}
// *****************************************************************************
/// @brief		Controla a saída DAC
/// @fn			void VoutControl(uint32_t u32Voltage)
/// @retval
// *****************************************************************************
void VoutControl(uint32_t u32Voltage)
{
	uint16_t u16Value = (uint16_t) ((as16Vad[VREF] /AD_MAX) * u32Voltage);

	u16ValueDAC = u16Value;

	if(u16ValueDAC > 4095)
		u16ValueDAC = 4095;

	if(u16ValueDAC <= 0)
		u16ValueDAC = 0;
	SetDAC(u16ValueDAC);
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

	if (TimeDiff(u32ReadADTimer) > REFRESH_AD)
	{
		u32ReadADTimer = HAL_GetTick();
		for (n = 0; n < NUM_AD_CHANNELS; n++)
		{
			psAdFilter = &adc[n];
			as16Filter[n] = ReadAnalogIn(n);
			psAdFilter->fFilterOutput = psAdFilter->fFilterOutput * 0.05f + as16Filter[n] * 0.95f;
		}

		as16Vad[VREF] =   VDDA_REF * (*(int16_t*)VREFCAL_ADDRESS) / adc[VREF].fFilterOutput;

		as16Vad[HALL] = (adc[HALL].fFilterOutput * as16Vad[VREF] / AD_MAX);

		as16Vad[SHUNT] = (adc[SHUNT].fFilterOutput * as16Vad[VREF] / AD_MAX);

		as16Vad[VOUT] = (adc[VOUT].fFilterOutput * as16Vad[VREF] / AD_MAX);

		as16Vad[VIN] = (adc[VIN].fFilterOutput * as16Vad[VREF] / AD_MAX);

		as16Vad[V_DAC] = (adc[V_DAC].fFilterOutput * as16Vad[VREF] / AD_MAX);

		as16Vad[TEMP] = ((T110 - T30 ) / ((uint32_t)*TS110 - (uint32_t)*TS30)) * \
				((adc[TEMP].fFilterOutput * as16Vad[VREF] / VDDA_REF) - (uint32_t)*TS30) \
				+ TEMP_COMP;

	}
}
// *****************************************************************************
/// @brief		Controla a recepção serial
/// @fn			void RxProcess(void)
// *****************************************************************************
void RxProcess(void)
{
	uint8_t *ps8Data;
	ps8Data = IsUartNewData();

	char *ps8Print = NULL;

	if (ps8Data != NULL)
	{
		if((ps8Data[0] == '#'))
		{
			//ps8Print = Print("V=%.3f;I_S=%.3f;I_H=%.3f;P_S=%.3f;P_H=%.3f;V_OUT=%.3f\r\n",1000.111,2000.222,3000.333,4000.444,5000.555,6000.666);
			ps8Print = Print("V=%.3f;I_S=%.3f;I_H=%.3f;P_S=%.3f;P_H=%.3f;V_OUT=%.3f\r\n",f_voltageValue[VIN],fCurrentShunt,fCurrentHall,fPowerShunt,fPowerHall,f_voltageValue[VOUT]);
			Uart2Write(ps8Print, strlen(ps8Print));
		}
	}
}
// *****************************************************************************
/// @brief		Formata dados
/// @fn			char* Print(const char *ps8Fmt, ...)
/// @param[in]	ps8Fmt		@brief Ponteiro para os dados
/// @retval		Ponteiro para o texto
// *****************************************************************************
char* Print(const char *ps8Fmt, ...)
{
	static char as8Buffer[120];
	va_list sArgs;

	va_start(sArgs, ps8Fmt);
	vsnprintf(as8Buffer, 120, ps8Fmt, sArgs);
	va_end(sArgs);
	return as8Buffer;
}
// *****************************************************************************
/// @brief		Tarefa do display
/// @fn			void TaskDisplay(void)
// *****************************************************************************
void TaskDisplay(void)
{
	if(TimeDiff(u32RefreschDisplay) > REFRESH_DISPLAY )
	{
		u32RefreschDisplay = HAL_GetTick();
		LCD_Clear();
		sprintf(Buff_Display,"T-BMP: %.2fC",ftempBMP);
		LCD_Write_String(0,0,Buff_Display);
		sprintf(Buff_Display,"V_L:%.2fmV",f_voltageValue[VIN]);
		LCD_Write_String(0,1,Buff_Display);
		sprintf(Buff_Display,"I_S:%.2fmA",fCurrentShunt);
		LCD_Write_String(0,2,Buff_Display);
		sprintf(Buff_Display,"I_H:%.1fmA",fCurrentHall);
		LCD_Write_String(0,3,Buff_Display);
		sprintf(Buff_Display,"P_S:%.2fW",fPowerShunt);
		LCD_Write_String(0,4,Buff_Display);
		sprintf(Buff_Display,"P_H:%.2fW",fPowerHall);
		LCD_Write_String(0,5,Buff_Display);
	}
}
// *****************************************************************************
/// @brief		Leitura da tecla
/// @fn			void TaskKey(void)
// *****************************************************************************
void TaskKey(void)
{
	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
	{
		if(TimeDiff(u32KeyTime) > 500)
		{
			u32KeyTime = HAL_GetTick();
		}
	}
	else
		u32KeyTime = HAL_GetTick();
}
// *****************************************************************************
/// @brief		Leitura dos dados recebido na serial do BLE
/// @fn			void UartData (void)
// *****************************************************************************
void UartData (void)
{
	uint8_t u8size;
	uint8_t u8buffData[UART_BUFFER_SIZE];

	if(TimeDiff(u32TimeRefreshBLE)> UART_DATA_TIME)
	{
		u32TimeRefreshBLE = HAL_GetTick();
		u8size = GetUartRxDataSize();
		if(u8size)
		{
			UartGetData(u8size, (uint8_t*)u8buffData);
			Manage_Data_Blue(u8buffData);
		}
	}
}
// *****************************************************************************
/// @brief		Tratamento da mensagem recebida
/// @fn			uint8_t Manage_Data_Blue(uint8_t *u8Data)
// *****************************************************************************
uint8_t Manage_Data_Blue(uint8_t *u8Data)
{
	char identify;
	char *p;
	char cmd[10];
	char parametro[10];
	static uint8_t u8buffSend[100];

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

//	if(identify == '=')
//	{
//		if(!strcmp(cmd, (char*)SET_TEMP))
//		{
//			fSet_Temp = atof(parametro);
//			sprintf((char*)u8buffSend,"SETADO PARAMETRO %.2fC\r\n",fSet_Temp);
//			HAL_UART_Transmit_IT(&huart2, (uint8_t*)u8buffSend,strlen((char*)u8buffSend));
//		}
//	}

	if(identify == '?')
	{
		if(!strcmp(cmd, (char*)GET_VIN))
		{
			sprintf((char*)u8buffSend,"SOLICITADO Tensão VIN %.2fmV\r\n",f_voltageValue[VIN]);
			HAL_UART_Transmit_IT(&huart4, u8buffSend,strlen((char*)u8buffSend));
		}
		if(!strcmp(cmd, (char*)GET_IS))
		{
			sprintf((char*)u8buffSend,"SOLICITADO Corrente do SHUNT -> %.2fmA\r\n",fCurrentShunt);

			HAL_UART_Transmit_IT(&huart4, u8buffSend,strlen((char*)u8buffSend));
		}
		if(!strcmp(cmd, (char*)GET_IH))
		{
			sprintf((char*)u8buffSend,"SOLICITADO Corrente do HALL-> %.2fmA\r\n",fCurrentHall);
			HAL_UART_Transmit_IT(&huart4, (uint8_t*)u8buffSend,strlen((char*)u8buffSend));
		}
		if(!strcmp(cmd, (char*)GET_TEMPBMP))
		{
			sprintf((char*)u8buffSend,"SOLICITADO TEM. BMP %.2fC\r\n",ftempBMP);
			HAL_UART_Transmit_IT(&huart4, u8buffSend,strlen((char*)u8buffSend));
		}
		if(!strcmp(cmd, (char*)GET_PS))
		{
			sprintf((char*)u8buffSend,"SOLICITADO potência do SHUNT %.2fW\r\n",fPowerShunt);
			HAL_UART_Transmit_IT(&huart4, u8buffSend,strlen((char*)u8buffSend));
		}

		if(!strcmp(cmd, (char*)GET_PH))
		{
			sprintf((char*)u8buffSend,"SOLICITADO potência do HALL %.2fW\r\n",fPowerHall);
			HAL_UART_Transmit_IT(&huart4, u8buffSend,strlen((char*)u8buffSend));
		}

	}
	return 0;

	ERROR:
	return HAL_ERROR;

}
// *****************************************************************************
/// @brief		Aquisição dos valores do sensor BMP
/// @fn			void ReadBMP(void)
// *****************************************************************************
void ReadBMP(void)
{
	ftempBMP = BMP180_GetTemp() / (float)1000; //LENDO TEMPERATURA
	fpressBMP = BMP180_GetPress (0) / (float)100; //LENDO PRESSAO
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//	u16ValueDAC += 100;
	//	if(u16ValueDAC > 4096)
	//		u16ValueDAC = 1500;
	//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, u16ValueDAC);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hAdc)
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
			ConfigUART4();
		if(huart->Instance == USART2)
			ConfigUSART2();
	}
}
//********* (C) COPYRIGHT 2022 Ânderson F.W *****END OF FILE*********
