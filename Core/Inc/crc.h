/*
 * crc.h
 *
 *  Created on: May 18, 2022
 *      Author: ander
 */

// Prevenção contra inclusão recursiva -----------------------------------------
#ifndef __CRC_H
#define __CRC_H
// Includes --------------------------------------------------------------------
#include "main.h"
// Exported constants ----------------------------------------------------------
#define CRC_LOOKUP_TABLE
// Exported types --------------------------------------------------------------
// Exported macro --------------------------------------------------------------
// Exported functions ----------------------------------------------------------
uint16_t CRC16(uint8_t  *pu8Data, int8_t s8DataLength, uint16_t  u16InitCRC);
uint8_t crc8x_simple(uint8_t *pu8Data,int8_t  s8DataLength, uint8_t u8InitCRC);
uint8_t crc8x_fast(uint8_t *pu8Data,int8_t  s8DataLength, uint8_t u8InitCRC);

#endif
