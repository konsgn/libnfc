/*-
 * Free/Libre Near Field Communication (NFC) library
 *
 * Libnfc historical contributors:
 * Copyright (C) 2009      Roel Verdult
 * Copyright (C) 2009-2013 Romuald Conty
 * Copyright (C) 2010-2012 Romain Tartière
 * Copyright (C) 2010-2013 Philippe Teuwen
 * Copyright (C) 2012-2013 Ludovic Rousseau
 * See AUTHORS file for a more comprehensive list of contributors.
 * Additional contributors of this file:
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

/**
* @file iso14443-subr.c
* @brief Defines some function extracted for ISO/IEC 14443
*/

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif // HAVE_CONFIG_H

#include <stdio.h>
#include <string.h>

#include <nfc/nfc.h>
#include "nfc-internal.h"


/**
 * @brief CRC_A
 *
 */
void
iso14443a_crc(uint8_t *pbtData, size_t szLen, uint8_t *pbtCrc)
{
  uint8_t  bt;
  uint32_t wCrc = 0x6363;

  do {
    bt = *pbtData++;
    bt = (bt ^ (uint8_t)(wCrc & 0x00FF));
    bt = (bt ^ (bt << 4));
    wCrc = (wCrc >> 8) ^ ((uint32_t) bt << 8) ^ ((uint32_t) bt << 3) ^ ((uint32_t) bt >> 4);
  } while (--szLen);

  *pbtCrc++ = (uint8_t)(wCrc & 0xFF);
  *pbtCrc = (uint8_t)((wCrc >> 8) & 0xFF);
}

/**
 * @brief Append CRC_A
 *
 */
void
iso14443a_crc_append(uint8_t *pbtData, size_t szLen)
{
  iso14443a_crc(pbtData, szLen, pbtData + szLen);
}

/**
 * @brief CRC_B
 *
 */
void
iso14443b_crc(uint8_t *pbtData, size_t szLen, uint8_t *pbtCrc)
{
  uint8_t  bt;
  uint32_t wCrc = 0xFFFF;

  do {
    bt = *pbtData++;
    bt = (bt ^ (uint8_t)(wCrc & 0x00FF));
    bt = (bt ^ (bt << 4));
    wCrc = (wCrc >> 8) ^ ((uint32_t) bt << 8) ^ ((uint32_t) bt << 3) ^ ((uint32_t) bt >> 4);
  } while (--szLen);
  wCrc = ~wCrc;
  *pbtCrc++ = (uint8_t)(wCrc & 0xFF);
  *pbtCrc = (uint8_t)((wCrc >> 8) & 0xFF);
}

/**
 * @brief Append CRC_B
 *
 */
void
iso14443b_crc_append(uint8_t *pbtData, size_t szLen)
{
  iso14443b_crc(pbtData, szLen, pbtData + szLen);
}

/**
 * @brief Locate historical bytes
 * @see ISO/IEC 14443-4 (5.2.7 Historical bytes)
 */
uint8_t *
iso14443a_locate_historical_bytes(uint8_t *pbtAts, size_t szAts, size_t *pszTk)
{
  if (szAts) {
    size_t offset = 1;
    if (pbtAts[0] & 0x10) { // TA
      offset++;
    }
    if (pbtAts[0] & 0x20) { // TB
      offset++;
    }
    if (pbtAts[0] & 0x40) { // TC
      offset++;
    }
    if (szAts > offset) {
      *pszTk = (szAts - offset);
      return (pbtAts + offset);
    }
  }
  *pszTk = 0;
  return NULL;
}

/**
 * @brief Add cascade tags (0x88) in UID
 * @see ISO/IEC 14443-3 (6.4.4 UID contents and cascade levels)
 */
void
iso14443_cascade_uid(const uint8_t abtUID[], const size_t szUID, uint8_t *pbtCascadedUID, size_t *pszCascadedUID)
{
  switch (szUID) {
    case 7:
      pbtCascadedUID[0] = 0x88;
      memcpy(pbtCascadedUID + 1, abtUID, 7);
      *pszCascadedUID = 8;
      break;

    case 10:
      pbtCascadedUID[0] = 0x88;
      memcpy(pbtCascadedUID + 1, abtUID, 3);
      pbtCascadedUID[4] = 0x88;
      memcpy(pbtCascadedUID + 5, abtUID + 3, 7);
      *pszCascadedUID = 12;
      break;

    case 4:
    default:
      memcpy(pbtCascadedUID, abtUID, szUID);
      *pszCascadedUID = szUID;
      break;
  }
}

/**
 * @brief Frame raw transmissions according to iso14443-4
 * @see ISO/IEC 14443-4 (7.1 Block Format)
 *
 * @param Data struct pointer to data sequence to transmit
 * @param szDataBits containing amount of data to send
 * @param FSC_FSD size of max array width (max transmitted/recieved bytes)
 * @param ptxArray points to the array to fill.
 * @note ptxArray is an array of type [min amount of blocks to fit all][FSC_FSD+1] the +1 to array width allows for sz of tx bytes for that block 
 * 
 */
void
iso14443_block_frame_data(bool *BlockNumber,const uint8_t *Data, const size_t szDataBits,const size_t FSC_FSD, uint8_t *ptxArray)
{
 size_t bytes= (szDataBits/8);
 size_t data_per_block= FSC_FSD-3; // size of each tx block -1 PCB byte -2 crc
 size_t block_count= (bytes/data_per_block)+1;
	for (int i=0;i<block_count;i++){
		if(bytes>data_per_block){ 
			*ptxArray++ = FSC_FSD-2; //first byte of array defines tx bytes for that block minus crc allocation
			*ptxArray++ = 0x12|((uint8_t)*BlockNumber&0x01); //to create an alternating block number distinguishing successive blocks
			*BlockNumber=!*BlockNumber;
			memcpy(ptxArray,Data+(i*data_per_block),data_per_block);
			ptxArray+=data_per_block+2; //add 2 for crc reservation
			bytes-=data_per_block;
		}
		else {
			*ptxArray++ = bytes+1; //first byte of array defines tx bytes for that block not including crc
			*ptxArray++ = 0x02|((uint8_t)*BlockNumber&0x01); //to create an alternating block number distinguishing successive blocks
			*BlockNumber=!*BlockNumber;
			memcpy(ptxArray,Data+(i*data_per_block),bytes);
			bytes=0;
		}	
	}	
//printf(" BlockNumber: %08x",BlockNumber);
}

