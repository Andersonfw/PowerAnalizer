//***************** (C) COPYRIGHT 2021 Ânderson F.W *****************
/// @brief
/// @file 		onewire.c
/// @author		Ânderson F. Weschenfelder
/// @version	V1.00
/// @date		02/08/21
// *****************************************************************************
// Includes --------------------------------------------------------------------
#include "onewire.h"
#include "main.h"

// Private typedef -------------------------------------------------------------
// Private macro ---------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Private function prototypes -------------------------------------------------

// Private functions -----------------------------------------------------------
// *****************************************************************************
/// @brief		Gera um delay de us utilizando um timer
/// @fn			void ONEWIRE_DELAY(uint16_t time_us)
/// @param[in]	time_us		@brief tempo em us
/// @retval		none
// *****************************************************************************
void ONEWIRE_DELAY(uint16_t time_us)
{
	htim6.Instance->CNT = 0;
	while(_DS18B20_TIMER.Instance->CNT <= time_us);

}
// *****************************************************************************
/// @brief		Seta o pino como saída em LOW
/// @fn			void ONEWIRE_LOW(OneWire_t *gp)
/// @param[in]	gp		@brief ponteiro para o gpio
/// @retval		none
// *****************************************************************************
void ONEWIRE_LOW(OneWire_t *gp)
{
	gp->GPIOx->BSRR = gp->GPIO_Pin<<16;
}	
// *****************************************************************************
/// @brief		Seta o pino como saída em HIGH
/// @fn			void ONEWIRE_HIGH(OneWire_t *gp)
/// @param[in]	gp		@brief ponteiro para o gpio
/// @retval		none
// *****************************************************************************
void ONEWIRE_HIGH(OneWire_t *gp)
{
	gp->GPIOx->BSRR = gp->GPIO_Pin;
}	
// *****************************************************************************
/// @brief		Configura o pino como entrada
/// @fn			void ONEWIRE_INPUT(OneWire_t *gp)
/// @param[in]	gp		@brief ponteiro para o gpio
/// @retval		none
// *****************************************************************************
void ONEWIRE_INPUT(OneWire_t *gp)
{
	GPIO_InitTypeDef	gpinit;
	gpinit.Mode = GPIO_MODE_INPUT;
	gpinit.Pull = GPIO_NOPULL;
	gpinit.Speed = GPIO_SPEED_FREQ_HIGH;
	gpinit.Pin = gp->GPIO_Pin;	
	HAL_GPIO_Init(gp->GPIOx,&gpinit);
}	
// *****************************************************************************
/// @brief		Configura o pino como saída open drain
/// @fn			void ONEWIRE_OUTPUT(OneWire_t *gp)
/// @param[in]	gp		@brief ponteiro para o gpio
/// @retval		none
// *****************************************************************************
void ONEWIRE_OUTPUT(OneWire_t *gp)
{
	GPIO_InitTypeDef	gpinit;
	gpinit.Mode = GPIO_MODE_OUTPUT_OD;
	gpinit.Pull = GPIO_NOPULL;
	gpinit.Speed = GPIO_SPEED_FREQ_HIGH;
	gpinit.Pin = gp->GPIO_Pin;
	HAL_GPIO_Init(gp->GPIOx,&gpinit);

}
// *****************************************************************************
/// @brief		Função que inicializa o baramento onewire
/// @fn			void OneWire_Init(OneWire_t* OneWireStruct, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
/// @param[in]	OneWireStruct	@brief ponteiro para struct Onewire
/// @param[in]	GPIOx			@brief Ponteiro para GPIO PORT
/// @param[in]	GPIO_Pin		@brief número GPIO_PIN
/// @retval		none
// *****************************************************************************
void OneWire_Init(OneWire_t* OneWireStruct, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) 
{	
	HAL_TIM_Base_Start(&_DS18B20_TIMER);

	OneWireStruct->GPIOx = GPIOx;
	OneWireStruct->GPIO_Pin = GPIO_Pin;
	ONEWIRE_OUTPUT(OneWireStruct);
	ONEWIRE_HIGH(OneWireStruct);
	OneWireDelay(1000);
	ONEWIRE_LOW(OneWireStruct);
	OneWireDelay(1000);
	ONEWIRE_HIGH(OneWireStruct);
	OneWireDelay(2000);
}
// *****************************************************************************
/// @brief		Reseta os dispositivos do barramento
/// @fn			inline uint8_t OneWire_Reset(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval		0 = OK, 1 = ERROR
// *****************************************************************************
inline uint8_t OneWire_Reset(OneWire_t* OneWireStruct)
{
	uint8_t i;
	
	/* Line low, and wait 480us */
	ONEWIRE_LOW(OneWireStruct);
	ONEWIRE_OUTPUT(OneWireStruct);
	ONEWIRE_DELAY(480);
	ONEWIRE_DELAY(20);
	/* Release line and wait for 70us */
	ONEWIRE_INPUT(OneWireStruct);
	ONEWIRE_DELAY(70);
	/* Check bit value */
	i = HAL_GPIO_ReadPin(OneWireStruct->GPIOx, OneWireStruct->GPIO_Pin);
	
	/* Delay for 410 us */
	ONEWIRE_DELAY(410);
	/* Return value of presence pulse, 0 = OK, 1 = ERROR */
	return i;
}
// *****************************************************************************
/// @brief		Escreve um bit no barramento onewire
/// @fn			inline void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	bit					@brief bit a ser escrito
/// @retval		none
// *****************************************************************************
inline void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit)
{
	if (bit) 
	{
		/* Set line low */
		ONEWIRE_LOW(OneWireStruct);
		ONEWIRE_OUTPUT(OneWireStruct);
		ONEWIRE_DELAY(10);
		
		/* Bit high */
		ONEWIRE_INPUT(OneWireStruct);
		
		/* Wait for 55 us and release the line */
		ONEWIRE_DELAY(55);
		ONEWIRE_INPUT(OneWireStruct);
	} 
	else 
	{
		/* Set line low */
		ONEWIRE_LOW(OneWireStruct);
		ONEWIRE_OUTPUT(OneWireStruct);
		ONEWIRE_DELAY(65);
		
		/* Bit high */
		ONEWIRE_INPUT(OneWireStruct);
		
		/* Wait for 5 us and release the line */
		ONEWIRE_DELAY(5);
		ONEWIRE_INPUT(OneWireStruct);
	}

}
// *****************************************************************************
/// @brief		Lê um bit no barramento onewire
/// @fn			inline uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval		bit lido
// *****************************************************************************
inline uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct) 
{
	uint8_t bit = 0;
	
	/* Line low */
	ONEWIRE_LOW(OneWireStruct);
	ONEWIRE_OUTPUT(OneWireStruct);
	ONEWIRE_DELAY(2);
	
	/* Release line */
	ONEWIRE_INPUT(OneWireStruct);
	ONEWIRE_DELAY(10);
	
	/* Read line value */
	if (HAL_GPIO_ReadPin(OneWireStruct->GPIOx, OneWireStruct->GPIO_Pin)) {
		/* Bit is HIGH */
		bit = 1;
	}
	
	/* Wait 50us to complete 60us period */
	ONEWIRE_DELAY(50);
	
	/* Return bit value */
	return bit;
}
// *****************************************************************************
/// @brief		Escreve um byte no barramento onewire
/// @fn			inline void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	byte				@brief byte a ser escrito
/// @retval		none
// *****************************************************************************
void OneWire_WriteByte(OneWire_t* OneWireStruct, uint8_t byte) {
	uint8_t i = 8;
	/* Write 8 bits */
	while (i--) {
		/* LSB bit is first */
		OneWire_WriteBit(OneWireStruct, byte & 0x01);
		byte >>= 1;
	}
}
// *****************************************************************************
/// @brief		Lê um byte no barramento onewire
/// @fn			inline uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval		byte lido
// *****************************************************************************
uint8_t OneWire_ReadByte(OneWire_t* OneWireStruct) {
	uint8_t i = 8, byte = 0;
	while (i--) {
		byte >>= 1;
		byte |= (OneWire_ReadBit(OneWireStruct) << 7);
	}
	
	return byte;
}
// *****************************************************************************
/// @brief		Função que tenta localizar o primeiro sensor no barramento onewire
/// @fn			uint8_t OneWire_First(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval		1 se encontrado dispositivo
// *****************************************************************************
uint8_t OneWire_First(OneWire_t* OneWireStruct) {
	/* Reset search values */
	OneWire_ResetSearch(OneWireStruct);

	/* Start with searching */
	return OneWire_Search(OneWireStruct, ONEWIRE_CMD_SEARCHROM);
}
// *****************************************************************************
/// @brief		Função que tenta localizar o proximo sensor no barramento onewire
/// @fn			uint8_t OneWire_Next(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval		1 se encontrado dispositivo
// *****************************************************************************
uint8_t OneWire_Next(OneWire_t* OneWireStruct) {
   /* Leave the search state alone */
   return OneWire_Search(OneWireStruct, ONEWIRE_CMD_SEARCHROM);
}
// *****************************************************************************
/// @brief		Função que reseta as flags de procura por dispositivos
/// @fn			uint8_t OneWire_ResetSearch(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval		none
// *****************************************************************************
void OneWire_ResetSearch(OneWire_t* OneWireStruct) {
	/* Reset the search state */
	OneWireStruct->LastDiscrepancy = 0;
	OneWireStruct->LastDeviceFlag = 0;
	OneWireStruct->LastFamilyDiscrepancy = 0;
}
// *****************************************************************************
/// @brief		Função que tenta localiza e lê a ROM dos dispositivos
/// @fn			uint8_t OneWire_Search(OneWire_t* OneWireStruct, uint8_t command)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	command				@brief comando de procura
/// @retval		1 se encontrado dispositivo
// *****************************************************************************
uint8_t OneWire_Search(OneWire_t* OneWireStruct, uint8_t command) {
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number, search_result;
	uint8_t id_bit, cmp_id_bit;
	uint8_t rom_byte_mask, search_direction;

	/* Initialize for search */
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;

	// if the last call was not the last one
	if (!OneWireStruct->LastDeviceFlag)
	{
		// 1-Wire reset
		if (OneWire_Reset(OneWireStruct)) 
		{
			/* Reset the search */
			OneWireStruct->LastDiscrepancy = 0;
			OneWireStruct->LastDeviceFlag = 0;
			OneWireStruct->LastFamilyDiscrepancy = 0;
			return 0;
		}

		// issue the search command 
		OneWire_WriteByte(OneWireStruct, command);  

		// loop to do the search
		do {
			// read a bit and its complement
			id_bit = OneWire_ReadBit(OneWireStruct);
			cmp_id_bit = OneWire_ReadBit(OneWireStruct);

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1)) {
				break;
			} else {
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit) {
					search_direction = id_bit;  // bit write value for search
				} else {
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < OneWireStruct->LastDiscrepancy) {
						search_direction = ((OneWireStruct->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					} else {
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == OneWireStruct->LastDiscrepancy);
					}
					
					// if 0 was picked then record its position in LastZero
					if (search_direction == 0) {
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9) {
							OneWireStruct->LastFamilyDiscrepancy = last_zero;
						}
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1) {
					OneWireStruct->ROM_NO[rom_byte_number] |= rom_byte_mask;
				} else {
					OneWireStruct->ROM_NO[rom_byte_number] &= ~rom_byte_mask;
				}
				
				// serial number search direction write bit
				OneWire_WriteBit(OneWireStruct, search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0) {
					//docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65)) {
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			OneWireStruct->LastDiscrepancy = last_zero;

			// check for last device
			if (OneWireStruct->LastDiscrepancy == 0) {
				OneWireStruct->LastDeviceFlag = 1;
			}

			search_result = 1;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !OneWireStruct->ROM_NO[0]) {
		OneWireStruct->LastDiscrepancy = 0;
		OneWireStruct->LastDeviceFlag = 0;
		OneWireStruct->LastFamilyDiscrepancy = 0;
		search_result = 0;
	}

	return search_result;
}
// *****************************************************************************
/// @brief		Função que
/// @fn			int OneWire_Verify(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval
// *****************************************************************************
int OneWire_Verify(OneWire_t* OneWireStruct) {
	unsigned char rom_backup[8];
	int i,rslt,ld_backup,ldf_backup,lfd_backup;

	// keep a backup copy of the current state
	for (i = 0; i < 8; i++)
	rom_backup[i] = OneWireStruct->ROM_NO[i];
	ld_backup = OneWireStruct->LastDiscrepancy;
	ldf_backup = OneWireStruct->LastDeviceFlag;
	lfd_backup = OneWireStruct->LastFamilyDiscrepancy;

	// set search to find the same device
	OneWireStruct->LastDiscrepancy = 64;
	OneWireStruct->LastDeviceFlag = 0;

	if (OneWire_Search(OneWireStruct, ONEWIRE_CMD_SEARCHROM)) {
		// check if same device found
		rslt = 1;
		for (i = 0; i < 8; i++) {
			if (rom_backup[i] != OneWireStruct->ROM_NO[i]) {
				rslt = 1;
				break;
			}
		}
	} else {
		rslt = 0;
	}

	// restore the search state 
	for (i = 0; i < 8; i++) {
		OneWireStruct->ROM_NO[i] = rom_backup[i];
	}
	OneWireStruct->LastDiscrepancy = ld_backup;
	OneWireStruct->LastDeviceFlag = ldf_backup;
	OneWireStruct->LastFamilyDiscrepancy = lfd_backup;

	// return the result of the verify
	return rslt;
}
// *****************************************************************************
/// @brief		Função que
/// @fn			void OneWire_TargetSetup(OneWire_t* OneWireStruct, uint8_t family_code)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	family_code			@brief
/// @retval		none
// *****************************************************************************
void OneWire_TargetSetup(OneWire_t* OneWireStruct, uint8_t family_code) {
   uint8_t i;

	// set the search state to find SearchFamily type devices
	OneWireStruct->ROM_NO[0] = family_code;
	for (i = 1; i < 8; i++) {
		OneWireStruct->ROM_NO[i] = 0;
	}
	
	OneWireStruct->LastDiscrepancy = 64;
	OneWireStruct->LastFamilyDiscrepancy = 0;
	OneWireStruct->LastDeviceFlag = 0;
}
// *****************************************************************************
/// @brief		Função que
/// @fn			void OneWire_FamilySkipSetup(OneWire_t* OneWireStruct)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @retval		none
// *****************************************************************************
void OneWire_FamilySkipSetup(OneWire_t* OneWireStruct) {
	// set the Last discrepancy to last family discrepancy
	OneWireStruct->LastDiscrepancy = OneWireStruct->LastFamilyDiscrepancy;
	OneWireStruct->LastFamilyDiscrepancy = 0;

	// check for end of list
	if (OneWireStruct->LastDiscrepancy == 0) {
		OneWireStruct->LastDeviceFlag = 1;
	}
}
// *****************************************************************************
/// @brief		Função que retoma o número da ROM de um dispositivo
/// @fn			uint8_t OneWire_GetROM(OneWire_t* OneWireStruct, uint8_t index)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	index				@brief identificação do dispositivo
/// @retval		Número da ROM
// *****************************************************************************
uint8_t OneWire_GetROM(OneWire_t* OneWireStruct, uint8_t index) {
	return OneWireStruct->ROM_NO[index];
}
// *****************************************************************************
/// @brief		Função que
/// @fn			void OneWire_Select(OneWire_t* OneWireStruct, uint8_t* addr)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	addr				@brief
/// @retval		none
// *****************************************************************************
void OneWire_Select(OneWire_t* OneWireStruct, uint8_t* addr) {
	uint8_t i;
	OneWire_WriteByte(OneWireStruct, ONEWIRE_CMD_MATCHROM);
	
	for (i = 0; i < 8; i++) {
		OneWire_WriteByte(OneWireStruct, *(addr + i));
	}
}
// *****************************************************************************
/// @brief		Função que seleciona o sensor no barrramento pela sua ROM para se comunicar
/// @fn			void OneWire_SelectWithPointer(OneWire_t* OneWireStruct, uint8_t *ROM)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	ROM					@brief ponteiro para a ROM
/// @retval		none
// *****************************************************************************
void OneWire_SelectWithPointer(OneWire_t* OneWireStruct, uint8_t *ROM) {
	uint8_t i;
	OneWire_WriteByte(OneWireStruct, ONEWIRE_CMD_MATCHROM);
	
	for (i = 0; i < 8; i++) {
		OneWire_WriteByte(OneWireStruct, *(ROM + i));
	}	
}
// *****************************************************************************
/// @brief		Função que copia a ROM lida no barramento para a struct do dispositivo
/// @fn			void OneWire_GetFullROM(OneWire_t* OneWireStruct, uint8_t *firstIndex)
/// @param[in]	OneWireStruct		@brief ponteiro para a struct onewire
/// @param[in]	firstIndex			@brief ponteiro para a ROM
/// @retval		none
// *****************************************************************************
void OneWire_GetFullROM(OneWire_t* OneWireStruct, uint8_t *firstIndex) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		*(firstIndex + i) = OneWireStruct->ROM_NO[i];
	}
}
// *****************************************************************************
/// @brief		Função que calcula o CRC
/// @fn			uint8_t OneWire_CRC8(uint8_t *addr, uint8_t len)
/// @param[in]	addr		@brief ponteiro para o endereço
/// @param[in]	len			@brief tamanho do dado
/// @retval		CRC calculado
// *****************************************************************************
uint8_t OneWire_CRC8(uint8_t *addr, uint8_t len) {
	uint8_t crc = 0, inbyte, i, mix;
	
	while (len--) {
		inbyte = *addr++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) {
				crc ^= 0x8C;
			}
			inbyte >>= 1;
		}
	}
	
	/* Return calculated CRC */
	return crc;
}
//***************** (C) COPYRIGHT 2021 Ânderson F.W *****END OF FILE*********
