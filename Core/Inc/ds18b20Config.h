//***************** (C) COPYRIGHT 2021 Ânderson F.W *****************
/// @brief
/// @file 		ds18b20Config.h
/// @author		Ânderson F. Weschenfelder
/// @version	V1.00
/// @date		02/08/21
// *****************************************************************************

// Prevençâontra inclusão recursiva -----------------------------------------
#ifndef	_DS18B20CONFIG_H
#define	_DS18B20CONFIG_H

// Includes --------------------------------------------------------------------
// Exported constants ----------------------------------------------------------
//	Init timer on cube    1us per tick				example 80 MHz cpu >>> Prescaler=(800-1)      counter period=Max
//#define	_DS18B20_USE_FREERTOS		    				1
#define _DS18B20_MAX_SENSORS		    				1
#define	_DS18B20_GPIO												ds18b20_GPIO_Port
#define	_DS18B20_PIN												ds18b20_Pin

#define	_DS18B20_CONVERT_TIMEOUT_MS					5000		
#if (_DS18B20_USE_FREERTOS==1)
#define	_DS18B20_UPDATE_INTERVAL_MS					10000					//  (((	if==0  >> Ds18b20_ManualConvert()  )))    ((( if>0  >>>> Auto refresh ))) 
#endif

#define	_DS18B20_TIMER											htim6

#endif
//***************** (C) COPYRIGHT 2021 Ânderson F.W *****END OF FILE*********
