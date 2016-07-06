/*-
 * Free/Libre Near Field Communication (NFC) library
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
 *
 */

#include <stdlib.h>
#include <string.h>



#include "rc522.h"
#include "rc522-internal.h"

#include "nfc/nfc.h"
#include "nfc-internal.h"
#include "timing.h"

// This divides by 8 with rounding towards infinity
#define BITS2BYTES(x) ((x >> 3) + ((x & 7) ? 1 : 0))
#define CHIP_DATA(x) ((struct rc522_chip_data *) (x)->chip_data)
#define CHK(x) ret = (x); if (ret < 0) { return ret; }

#define LOG_CATEGORY "libnfc.chip.rc522"
#define LOG_GROUP NFC_LOG_GROUP_CHIP

#define TIMEOUT_DEFAULT 50
#define TIMEOUT_NEVER 0

const nfc_modulation_type rc522_initiator_modulation[] = { NMT_ISO14443A, 0 };
const nfc_modulation_type rc522_target_modulation[] = { 0 };

const nfc_baud_rate rc522_iso14443a_supported_baud_rates[] = { NBR_847, NBR_424, NBR_212, NBR_106, 0 };

struct rc522_chip_data {
	const struct rc522_io * io;
	uint8_t version;
	int default_timeout;
};


int rc522_data_new(struct nfc_device * pnd, const struct rc522_io * io) {
	pnd->chip_data = malloc(sizeof(struct rc522_chip_data));
	if (!pnd->chip_data) {
		perror("malloc");
		return NFC_ESOFT;
	}

	CHIP_DATA(pnd)->io = io;
	CHIP_DATA(pnd)->version = RC522_UNKNOWN;
	CHIP_DATA(pnd)->default_timeout = 500;
	return NFC_SUCCESS;
}

void rc522_data_free(struct nfc_device * pnd) {
	free(pnd->chip_data);
}

int rc522_read_bulk(struct nfc_device * pnd, uint8_t reg, uint8_t * val, size_t len) {
	int ret = CHIP_DATA(pnd)->io->read(pnd, reg, val, len);
	if (ret) {
		log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "Unable to read register %02X (err: %d)", reg, ret);
		return ret;
	}

#ifdef LOG
	char action[8];
	snprintf(action, sizeof(action), "RD %02X", reg);
	LOG_HEX(NFC_LOG_GROUP_CHIP, action, val, len);
#endif

	return NFC_SUCCESS;
}

int rc522_write_bulk(struct nfc_device * pnd, uint8_t reg, const uint8_t * val, size_t len) {
	int ret = CHIP_DATA(pnd)->io->write(pnd, reg, val, len);
	if (ret) {
		log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "Unable to write register %02X!", reg);
		return ret;
	}

#ifdef LOG
	char action[8];
	snprintf(action, sizeof(action), "WR %02X", reg);
	LOG_HEX(NFC_LOG_GROUP_CHIP, action, val, len);
#endif

	return NFC_SUCCESS;
}

int rc522_read_reg(struct nfc_device * pnd, uint8_t reg) {
	uint8_t val;
	int ret;
	CHK(rc522_read_bulk(pnd, reg, &val, 1));
	return val;
}

int rc522_write_reg(struct nfc_device * pnd, uint8_t reg, uint8_t val) {
	return rc522_write_bulk(pnd, reg, &val, 1);
}

int rc522_write_reg_mask(struct nfc_device * pnd, uint8_t reg, uint8_t val, uint8_t mask) {
	if (mask != 0xFF) {
		int oldval = rc522_read_reg(pnd, reg);
		if (oldval < 0) {
			return oldval;
		}

		val = (val & mask) | (oldval & ~mask);
	}

	return rc522_write_reg(pnd, reg, val);
}

int rc522_start_command(struct nfc_device * pnd, rc522_cmd cmd) {
	bool needsRX;

	// Disabling RX saves energy, so based on the command we'll also update the RxOff flag
	switch (cmd) {
		case CMD_IDLE:
		case CMD_MEM:
		case CMD_GENRANDOMID:
		case CMD_CALCCRC:
			needsRX = true;
			break;

		case CMD_TRANSMIT:
		case CMD_SOFTRESET:
			needsRX = false;
			break;

		case CMD_RECEIVE:
		case CMD_TRANSCEIVE:
		case CMD_MFAUTHENT:
			needsRX = true;
			break;

		case CMD_NOCMDCHANGE:
			return NFC_SUCCESS;

		default:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "Attempted to execute non-existant command: %02X", cmd);
			return NFC_ESOFT;
	}

	uint8_t regval = cmd;
	if (!needsRX) {
		regval |= REG_CommandReg_RcvOff;
	}
	

	return rc522_write_reg(pnd, REG_CommandReg, regval);
}

int rc522_wait_wakeup(struct nfc_device * pnd) {
	// NXP does not mention in the datasheet how much time does it take for RC522 to come back to life, so we'll wait up to 50ms
	timeout_t to;
	timeout_init(&to, 50);

	while (timeout_check(&to)) {
		int ret = rc522_read_reg(pnd, REG_CommandReg);
		if (ret < 0 && ret != NFC_ETIMEOUT) {
			return ret;
		}

		// If the powerdown bit is zero the RC522 is ready to kick asses!
		if ((ret & REG_CommandReg_PowerDown) == 0) {
			return NFC_SUCCESS;
		}
	}

	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_wait_wakeup timeout!");
	return NFC_ETIMEOUT;
}

int rc522_send_baudrate(struct nfc_device * pnd, uint32_t baudrate) {
	uint8_t regval;

	// MFRC522 datasheet 8.1.3.2
	switch (baudrate) {
		case 7200:
			regval = 0xFA;
			break;
		case 9600:
			regval = 0xEB;
			break;
		case 14400:
			regval = 0xDA;
			break;
		case 19200:
			regval = 0xCB;
			break;
		case 38400:
			regval = 0xAB;
			break;
		case 57600:
			regval = 0x9A;
			break;
		case 115200:
			regval = 0x7A;
			break;
		case 128000:
			regval = 0x74;
			break;
		case 230400:
			regval = 0x5A;
			break;
		case 460800:
			regval = 0x3A;
			break;
		case 921600:
			regval = 0x1C;
			break;
		case 1288000:
			regval = 0x15;
			break;
		default:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "rc522_write_baudrate unsupported baud rate: %d bps.", baudrate);
			return NFC_EDEVNOTSUPP;
	}

	return rc522_write_reg(pnd, REG_SerialSpeedReg, regval);
}

int rc522_soft_reset(struct nfc_device * pnd) {
	int ret;

	// 1. Execute reset command
	CHK(rc522_start_command(pnd, CMD_SOFTRESET));

	// 2. If using an UART, reset baud rate to RC522 default speed
	if (CHIP_DATA(pnd)->io->reset_baud_rate) {
		CHK(CHIP_DATA(pnd)->io->reset_baud_rate(pnd));
	}

	// 3. Wait for the RC522 to come back to life, as we shouldn't modify any register till that happens
	CHK(rc522_wait_wakeup(pnd));

	// 4. If using an UART, restore baud rate to user's choice
	if (CHIP_DATA(pnd)->io->upgrade_baud_rate) {
		CHK(CHIP_DATA(pnd)->io->upgrade_baud_rate(pnd));
	}

	return NFC_SUCCESS;
}

int rc522_set_rf_baud_rate(struct nfc_device * pnd, nfc_baud_rate speed) {
	uint8_t txVal, rxVal;
	int ret;

	switch (speed) {
		case NBR_106:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Updating RF baud rate to 106kbps.");
			txVal = REG_TxModeReg_TxSpeed_106k;
			rxVal = REG_RxModeReg_RxSpeed_106k;
			break;

		case NBR_212:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Updating RF baud rate to 212kbps.");
			txVal = REG_TxModeReg_TxSpeed_212k;
			rxVal = REG_RxModeReg_RxSpeed_212k;
			break;

		case NBR_424:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Updating RF baud rate to 424kbps.");
			txVal = REG_TxModeReg_TxSpeed_424k;
			rxVal = REG_RxModeReg_RxSpeed_424k;
			break;

		case NBR_847:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Updating RF baud rate to 847kbps.");
			txVal = REG_TxModeReg_TxSpeed_847k;
			rxVal = REG_RxModeReg_RxSpeed_847k;
			break;

		default:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "Attempted to switch RF baud rate to 0x%08X.", speed);
			return NFC_EINVARG;
	}

	CHK(rc522_write_reg_mask(pnd, REG_TxModeReg, txVal, REG_TxModeReg_TxSpeed_MASK));
	CHK(rc522_write_reg_mask(pnd, REG_RxModeReg, rxVal, REG_RxModeReg_RxSpeed_MASK));

	return NFC_SUCCESS;
}

int rc522_initiator_select_passive_target(struct nfc_device *pnd,
                                      const nfc_modulation nm,
                                      const uint8_t *pbtInitData, const size_t szInitData,
                                      nfc_target *pnt)
{
  return rc522_initiator_select_passive_target_ext(pnd, nm, pbtInitData, szInitData, pnt, 3000); 
}

int rc522_initiator_select_passive_target_ext(struct nfc_device * pnd, const nfc_modulation nm, const uint8_t * pbtInitData, const size_t szInitData, nfc_target * pnt, int timeout) {	
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_initiator_select_passive_target_ext szintdata:%d",szInitData);
	
	#endif
	int ret;
	uint8_t  abtTargetsData[64];
	size_t  szTargetsData = sizeof(abtTargetsData);
	nfc_target nttmp;
	memset(&nttmp, 0x00, sizeof(nfc_target));
	
	if (nm.nmt != NMT_ISO14443A) {
		return NFC_EINVARG;
	}

	CHK(rc522_set_rf_baud_rate(pnd, nm.nbr));

	// TODO Verify
	do{
	  if ((ret = rc522_inListPassiveTarget(pnd, pbtInitData, szInitData, pnt, timeout)) < 0) {
        if ((ret == NFC_ERFTRANS)) { // Chip timeout
          continue;
        } else
          return ret;
      }
		//if ((nm.nmt == NMT_ISO14443A) && (nm.nbr != NBR_106)) {
		  ////uint8_t pncmd_inpsl[4] = { InPSL, 0x01 };
		  ////pncmd_inpsl[2] = nm.nbr - 1;
		  ////pncmd_inpsl[3] = nm.nbr - 1;
		  //if ((ret = pn53x_transceive(pnd, pbtInitData, szInitData, NULL, 0, 0)) < 0) {
			//return ret;
		  //}
		//}
	}while (pnd->bInfiniteSelect);

	 if ((ret = rc522_initiator_transceive_bytes(pnd, pbtInitData, szInitData, abtTargetsData, sizeof(abtTargetsData), timeout)) < 0) {
		if ((ret == NFC_ERFTRANS)) { // Chip timeout
		  return NFC_ENOTIMPL;
		} else
		  return ret;
	  }
	
	return NFC_ENOTIMPL;
}

int 
rc522_inListPassiveTarget(struct nfc_device *pnd,
                          const uint8_t *pbtInitiatorData, const size_t szInitiatorData,
                          nfc_target *pnt,
                          int timeout)
{
	int ret;
	uint8_t  Buff[15] = {0,};
	nfc_target nttmp;
	memset(&nttmp, 0x00, sizeof(nfc_target));
	
	if(szInitiatorData>4)
		return NFC_ENOTIMPL; //TODO Implement proper cascade levels...check iso14443-subr.c  

	CHK(rc522_query_a_tags(pnd, Buff, timeout));	
	nttmp.nti.nai.abtAtqa[0]=Buff[1];
	nttmp.nti.nai.abtAtqa[1]=Buff[0];
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "AtqA response is: 0x%02x%02x",Buff[1],Buff[0]);
	if((Buff[0] >> 6)&0x03)return NFC_ENOTIMPL;//TODO implement read UID longer than 4

	CHK(rc522_select_anticollision_loop(pnd, Buff, timeout));
	memcpy(&(nttmp.nti.nai.abtUid),&Buff,4);
	nttmp.nti.nai.szUidLen=4;
	nttmp.nti.nai.btSak=Buff[5];

	if(pnt) memcpy(pnt,&nttmp,sizeof(nfc_target));
	//// Set the optional initiator data (for ISO14443A selecting a specific UID).
	//if (pbtInitiatorData)
	//memcpy(abtCmd + 3, pbtInitiatorData, szInitiatorData);
	//int res = 0;
	//if ((res = pn53x_transceive(pnd, abtCmd, 3 + szInitiatorData, pbtTargetsData, *pszTargetsData, timeout)) < 0) {
	//return res;
	//}
	//*pszTargetsData = (size_t) res;
	//return pbtTargetsData[0];
	return NFC_SUCCESS;
}

int 
rc522_select_anticollision_loop(struct nfc_device *pnd, uint8_t * UIDSAK, int timeout)
{
	int ret;
	uint8_t Buff[5];
	uint8_t  abtCmd[6] = {SEL,0x20,0,}; //NVB is 0x20 to select all cards
	CHK(rc522_transceive(pnd, abtCmd, 2*8, Buff, 5, timeout)); //TODO implement anticollision & Bcc check
	abtCmd[1]+=0x40; //adding 4 bytes of UID to select command so we can get SAK.
	memcpy(&abtCmd[2],&Buff,4); // adding UID recieved to select command
	CHK(rc522_transceive(pnd, abtCmd, 6*8, Buff[4], 2, timeout)); //TODO implement Sak check?
	memcpy(&UIDSAK,&Buff,5); 
	// implement ATS request if necessary?
	return NFC_SUCCESS;
}

int
rc522_query_a_tags(struct nfc_device *pnd, uint8_t * retReqa, int timeout)
{
	int ret;
	uint8_t  abtCmd[1] = {REQA};
	CHK(rc522_transceive(pnd, abtCmd, 7, retReqa, 2, timeout)); //TODO implement anticollision
	return NFC_SUCCESS;
}

//int
//rc522_inDeselect(struct nfc_device *pnd, const uint8_t ui8Target)
//{
  //if (CHIP_DATA(pnd)->type == RCS360) {
    //// We should do act here *only* if a target was previously selected
    //uint8_t  abtStatus[PN53x_EXTENDED_FRAME__DATA_MAX_LEN];
    //size_t  szStatus = sizeof(abtStatus);
    //uint8_t  abtCmdGetStatus[] = { GetGeneralStatus };
    //int res = 0;
    //if ((res = pn53x_transceive(pnd, abtCmdGetStatus, sizeof(abtCmdGetStatus), abtStatus, szStatus, -1)) < 0) {
      //return res;
    //}
    //szStatus = (size_t) res;
    //if ((szStatus < 3) || (abtStatus[2] == 0)) {
      //return NFC_SUCCESS;
    //}
    //// No much choice what to deselect actually...
    //uint8_t  abtCmdRcs360[] = { InDeselect, 0x01, 0x01 };
    //return (pn53x_transceive(pnd, abtCmdRcs360, sizeof(abtCmdRcs360), NULL, 0, -1));
  //}
  //uint8_t  abtCmd[] = { InDeselect, ui8Target };
  //return (pn53x_transceive(pnd, abtCmd, sizeof(abtCmd), NULL, 0, -1));
//}

void rc522_timeout_init(struct nfc_device * pnd, timeout_t * to, int timeout) {
	if (timeout == TIMEOUT_NEVER) {
		log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_timeout_init: creating timeout which doesn't expire.");
		timeout_never(to);
	} else {
		if (timeout == TIMEOUT_DEFAULT) {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_timeout_init: creating with default time (%d ms).", CHIP_DATA(pnd)->default_timeout);
			timeout = CHIP_DATA(pnd)->default_timeout;
		} else {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_timeout_init: creating with custom time of %d ms.", timeout);
		}

		timeout_init(to, timeout);
	}
}

int rc522_rf_tx(struct nfc_device * pnd, const uint8_t * txData, const size_t txBits, timeout_t * timeout, bool transceive) {
	size_t txBytes = BITS2BYTES(txBits);
	size_t transmitted = MIN(txBytes, FIFO_SIZE);
	int ret;

	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_tx: sending %d bits (%d bytes).", txBits, txBytes);

	CHK(rc522_write_reg(pnd, REG_ComIrqReg, REG_ComIrqReg_TxIRq | REG_ComIrqReg_RxIRq | REG_ComIrqReg_HiAlertIRq | REG_ComIrqReg_LoAlertIRq | REG_ComIrqReg_ErrIRq | REG_ComIrqReg_TimerIRq));
		//CHK(rc522_write_reg(pnd, REG_ComIrqReg, 0xff^REG_ComIrqReg_Set1 ));
	CHK(rc522_write_bulk(pnd, REG_FIFODataReg, txData, transmitted));
	
	//set bit count before starting tx
	CHK(rc522_write_reg(pnd, REG_BitFramingReg, REG_BitFramingReg_RxAlign_PACK(0) | REG_BitFramingReg_TxLastBits_PACK(txBits)));
	if (transceive) {
		// If transceiving we must first start the command and then configure framing and start transmission
		CHK(rc522_start_command(pnd, CMD_TRANSCEIVE));
		CHK(rc522_write_reg(pnd, REG_BitFramingReg, REG_BitFramingReg_StartSend | REG_BitFramingReg_RxAlign_PACK(0) | REG_BitFramingReg_TxLastBits_PACK(txBits)));
	} else {
		// If only transmitting we must configure framing and then start the transmission
		CHK(rc522_write_reg(pnd, REG_BitFramingReg, REG_BitFramingReg_RxAlign_PACK(0) | REG_BitFramingReg_TxLastBits_PACK(txBits)));
		CHK(rc522_start_command(pnd, CMD_TRANSMIT));
	}

	while (1) {
		if (!timeout_check(timeout)) {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_tx: transmission timeout.");
			return NFC_ETIMEOUT;
		}

		int irqs = CHK(rc522_read_reg(pnd, REG_ComIrqReg));

		if (irqs & REG_ComIrqReg_ErrIRq) {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_tx: RC522 set ErrIRq flag.");
			// If the RC522 detects an error abort the transmission and notify the caller
			return NFC_ECHIP;
		}

		if (irqs & REG_ComIrqReg_TxIRq) {
			// Check if the FIFO has underflowed (ie the transmission has ended before we've feeded all the bytes to the FIFO)
			if (transmitted < txBytes) {
				log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "rc522_rf_tx: couldn't feed bytes fast enough. Only %d out of %d bytes have been sent. Aborting transmission.", transmitted, txBytes);
				return NFC_ESOFT;
			}
			// Otherwise we're done
			break;
		}

		if ((irqs & REG_ComIrqReg_LoAlertIRq) && transmitted < txBytes) {
			// Okay, now attempt to write as many bytes as possible. This IRQ is generated based on the water level, so we know for sure we can feed at least FIFO_SIZE - DEFAULT_WATER_LEVEL bytes.
			size_t chunkSize = MIN(txBytes - transmitted, FIFO_SIZE - DEFAULT_WATER_LEVEL);
			CHK(rc522_write_bulk(pnd, REG_FIFODataReg, txData + transmitted, chunkSize));
			transmitted += chunkSize;

			// TODO: Should we clear the flag before or after feeding the data?
			CHK(rc522_write_reg(pnd, REG_ComIrqReg, REG_ComIrqReg_LoAlertIRq));

			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_tx: fed another %d bytes to FIFO.", chunkSize);
		}
	}
	//if (transceive) {
		//CHK(rc522_write_reg_mask(pnd, REG_BitFramingReg, !REG_BitFramingReg_StartSend,REG_BitFramingReg_StartSend));
	//}
		
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_tx: transmission finished.");
	return NFC_SUCCESS;
}

int rc522_rf_rx(struct nfc_device * pnd, uint8_t * rxData, const size_t rxMaxBytes, timeout_t * timeout, bool transceive) {
	int ret;
	size_t received = 0;
	

			
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: receiving up to %d bytes.", rxMaxBytes);

	if (!transceive) {
	// Clear all irq's as early as possible... only if not transcieving
	CHK(rc522_write_reg(pnd, REG_ComIrqReg, 0xff^REG_ComIrqReg_Set1));
		//CHK(rc522_write_reg(pnd, REG_ComIrqReg, REG_ComIrqReg_TxIRq | REG_ComIrqReg_RxIRq | REG_ComIrqReg_LoAlertIRq | REG_ComIrqReg_ErrIRq));
		CHK(rc522_write_reg(pnd, REG_ComIrqReg, 0xff^REG_ComIrqReg_Set1));	
		//rc522_abort(pnd);
		CHK(rc522_start_command(pnd, CMD_RECEIVE));
	}
	else {
		//// Clear all irq's except recieve
		//CHK(rc522_write_reg(pnd, REG_ComIrqReg, 0x7f^REG_ComIrqReg_RxIRq));
		
		//CHK(rc522_read_reg(pnd, REG_FIFOLevelReg));
		//CHK(rc522_read_reg(pnd, REG_RxModeReg));
		//CHK(rc522_read_reg(pnd, REG_TxModeReg));
		//CHK(rc522_read_reg(pnd, REG_Status1Reg));
		//CHK(rc522_read_reg(pnd, REG_Status2Reg));
	}
	while (1) {
		if (!timeout_check(timeout)) {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: transmission timeout.");
			return NFC_ETIMEOUT;
		}

		int irqs = CHK(rc522_read_reg(pnd, REG_ComIrqReg));
		//int errs = CHK(rc522_read_reg(pnd, REG_ErrorReg));

		if (irqs & REG_ComIrqReg_TimerIRq) {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: RC522 set timer_ovrflow flag.");
			// If the RC522 detects an timeout abort the RX and notify the caller
			return NFC_ETIMEOUT;
		}

		if (irqs & REG_ComIrqReg_ErrIRq) {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: RC522 has Error");
			int errs = CHK(rc522_read_reg(pnd, REG_ErrorReg));
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: Errors:%02X", errs);
			// If the RC522 detects an error abort the transmission and notify the caller
			return NFC_ECHIP;
		}

		if (irqs & REG_ComIrqReg_RxIRq) {
			break;
		}

		if (irqs & REG_ComIrqReg_HiAlertIRq) {
			size_t chunkSize = FIFO_SIZE - DEFAULT_WATER_LEVEL;
			if (rxMaxBytes - received < chunkSize) {
				log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: RX buffer overflow (buffer contains %d bytes and may hold up to %d bytes, but needs %d more).", received, rxMaxBytes, chunkSize);
				return NFC_EOVFLOW;
			}

			CHK(rc522_read_bulk(pnd, REG_FIFODataReg, rxData + received, chunkSize));
			received += chunkSize;

			// TODO: Should we clear the flag before or after feeding the data?
			CHK(rc522_write_reg(pnd, REG_ComIrqReg, REG_ComIrqReg_HiAlertIRq));

			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: read another %d bytes from FIFO.", chunkSize);
		}
	}

	CHK(rc522_read_reg(pnd, REG_FIFOLevelReg));
	size_t remaining = REG_FIFOLevelReg_Level_UNPACK(ret);

	if (rxMaxBytes - received < remaining) {
		log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: RX buffer overflow (buffer contains %d bytes and may hold up to %d bytes, but needs %d more).", received, rxMaxBytes, remaining);
		return NFC_EOVFLOW;
	}

	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: reading last %d bytes from FIFO.", remaining);
	CHK(rc522_read_bulk(pnd, REG_FIFODataReg, rxData + received, remaining));
	received += remaining;

	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_rf_rx: receive finished. Read %d bytes.", received);

	return received;
}

int rc522_transceive(struct nfc_device * pnd, const uint8_t * txData, const size_t txBits, uint8_t * rxData, const size_t rxMaxBytes, int timeout) {
	int ret;

	bool doTX = ((txData != NULL) && txBits > 0);
	bool doRX = ((rxData != NULL) );
	bool isTransceive = doTX && doRX;

	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_transceive: is doTX: %d", doTX);
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_transceive: is rxData,max: %d:%d", rxData,rxMaxBytes);
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "rc522_transceive: is transcieve: %d", isTransceive);
	CHK(rc522_abort(pnd));

	timeout_t to;
	rc522_timeout_init(pnd, &to, timeout);

	if (doTX) {
		ret = rc522_rf_tx(pnd, txData, txBits, &to, isTransceive);
		if (ret < 0) {
			rc522_abort(pnd);
			return ret;
		}
	}

	if (doRX) {
		ret = rc522_rf_rx(pnd, rxData, rxMaxBytes, &to, isTransceive);
		if (ret < 0) {
			rc522_abort(pnd);
		}
	}

	return ret;
}

int rc522_initiator_transceive_bits(struct nfc_device * pnd, const uint8_t * txData, const size_t txBits, const uint8_t * pbtTxPar, uint8_t * rxData, uint8_t * pbtRxPar) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_initiator_transceive_bits");
	#endif
	int ret;
	// TODO: Do something with pbtTxPar and pbtRxPar
	CHK(rc522_transceive(pnd, txData, txBits, rxData, ~0, TIMEOUT_DEFAULT));
	return ret * 8;
}

int rc522_initiator_transceive_bytes(struct nfc_device * pnd, const uint8_t * txData, const size_t txSize, uint8_t * rxData, const size_t rxMaxBytes, int timeout) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_initiator_transceive_bytes");
	#endif
	return rc522_transceive(pnd, txData, txSize * 8, rxData, rxMaxBytes, timeout);
}

int rc522_get_supported_modulation(struct nfc_device * pnd, const nfc_mode mode, const nfc_modulation_type ** const supported_mt) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_get_supported_modulation");
	#endif
	(void) pnd;

	switch (mode) {
		case N_INITIATOR:
			*supported_mt = rc522_initiator_modulation;
			break;

		case N_TARGET:
			*supported_mt = rc522_target_modulation;
			break;

		default:
			return NFC_EINVARG;
	}

	return NFC_SUCCESS;
}

int rc522_get_supported_baud_rate(struct nfc_device * pnd, const nfc_mode mode, const nfc_modulation_type nmt, const nfc_baud_rate ** const supported_br) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_get_supported_baud_rate");
	#endif
	(void) pnd;

	switch (mode) {
		case N_INITIATOR:
			switch (nmt) {
				case NMT_ISO14443A:
					*supported_br = rc522_iso14443a_supported_baud_rates;
					break;

				default:
					return NFC_EINVARG;
			}
			break;

		case N_TARGET:
		default:
			return NFC_EINVARG;
	}

	return NFC_SUCCESS;
}

int rc522_set_property_bool(struct nfc_device * pnd, const nfc_property property, const bool enable) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_set_property_bool property:%x bool:%x",property,enable);
	#endif
	int ret;

	switch (property) {
		case NP_HANDLE_CRC:
			if (pnd->bCrc == enable) {
				return NFC_SUCCESS;
			}

			CHK(rc522_write_reg_mask(pnd, REG_TxModeReg, enable ? ~0 : 0, REG_TxModeReg_TxCRCEn));
			CHK(rc522_write_reg_mask(pnd, REG_RxModeReg, enable ? ~0 : 0, REG_RxModeReg_RxCRCEn));

			pnd->bCrc = enable;
			return NFC_SUCCESS;

		case NP_HANDLE_PARITY:
			if (pnd->bPar == enable) {
				return NFC_SUCCESS;
			}

			// Note it's parity DISABLE (ie active low)
			CHK(rc522_write_reg_mask(pnd, REG_MfRxReg, enable ? 0 : ~0, REG_MfRxReg_ParityDisable));

			pnd->bPar = enable;
			return NFC_SUCCESS;

		case NP_EASY_FRAMING:
			pnd->bEasyFraming = enable;
			return NFC_SUCCESS;

		case NP_ACTIVATE_FIELD:
			return rc522_write_reg_mask(pnd, REG_TxControlReg, enable ? ~0 : 0, REG_TxControlReg_Tx2RFEn | REG_TxControlReg_Tx1RFEn);
		
		case NP_ACTIVATE_CRYPTO1:
			return rc522_write_reg_mask(pnd, REG_Status2Reg, enable ? ~0 : 0, REG_Status2Reg_MFCrypto1On);

		case NP_FORCE_ISO14443_A:
			// ISO14443-A is the only mode supported by MFRC522
			return NFC_SUCCESS;

		case NP_FORCE_SPEED_106:
			if (!enable) {
				return NFC_SUCCESS;
			}

			return rc522_set_rf_baud_rate(pnd, NBR_106);

		case NP_ACCEPT_MULTIPLE_FRAMES:
			// TODO: Figure out what this does and implement it
			// HACK: Return NFC_SUCCESS so nfc-anticol runs
			return NFC_SUCCESS;

		case NP_AUTO_ISO14443_4:
			// TODO: Figure out what this does and implement it
			// HACK: Return NFC_SUCCESS so nfc-anticol runs
			return NFC_SUCCESS;

		case NP_ACCEPT_INVALID_FRAMES:
			// TODO: Figure out what this does and implement it
			// HACK: Return NFC_SUCCESS so nfc-anticol runs
			return NFC_SUCCESS;

		case NP_INFINITE_SELECT:
			// TODO: The RC522 can't do scans on its own, what now?
			// HACK: Return NFC_SUCCESS so nfc-anticol runs
			return NFC_SUCCESS;

		case NP_FORCE_ISO14443_B:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "Attempted to enable ISO14443B");
			if (enable) {
				return NFC_EDEVNOTSUPP;
			}
			return NFC_SUCCESS;

		case NP_TIMEOUT_COMMAND:
		case NP_TIMEOUT_ATR:
		case NP_TIMEOUT_COM:
			break;
	}

	return NFC_EINVARG;
}

int rc522_set_property_int(struct nfc_device * pnd, const nfc_property property, const int value) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_set_property_int");
	#endif
	switch (property) {
		case NP_TIMEOUT_COMMAND:
			if (value >= 0) {
				CHIP_DATA(pnd)->default_timeout = value;
				return NFC_SUCCESS;
			}
			break;

		case NP_TIMEOUT_ATR:
			// TODO: Figure out what this does and implement it
			return NFC_ENOTIMPL;

		case NP_TIMEOUT_COM:
			// TODO: Figure out what this does and implement it
			return NFC_ENOTIMPL;

		case NP_HANDLE_CRC:
		case NP_HANDLE_PARITY:
		case NP_EASY_FRAMING:
		case NP_ACTIVATE_FIELD:
		case NP_ACTIVATE_CRYPTO1:
		case NP_FORCE_ISO14443_A:
		case NP_FORCE_SPEED_106:
		case NP_ACCEPT_MULTIPLE_FRAMES:
		case NP_AUTO_ISO14443_4:
		case NP_ACCEPT_INVALID_FRAMES:
		case NP_INFINITE_SELECT:
		case NP_FORCE_ISO14443_B:
			break;
	}

	return NFC_EINVARG;
}

int rc522_initiator_init(struct nfc_device * pnd) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_initiator_init");
	#endif

	
	// TODO: Should we be doing something here?
	return NFC_SUCCESS;
}

int rc522_abort(struct nfc_device * pnd) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_abort");
	#endif
	int ret;

	// Halt any running commands
	CHK(rc522_start_command(pnd, CMD_IDLE));
	// Clear FIFO
	CHK(rc522_write_reg(pnd, REG_FIFOLevelReg, REG_FIFOLevelReg_FlushBuffer));

	return NFC_SUCCESS;
}

int rc522_powerdown(struct nfc_device * pnd) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_powerdown");
	#endif
	//return rc522_write_reg(pnd, REG_CommandReg, REG_CommandReg_RcvOff | REG_CommandReg_PowerDown | CMD_NOCMDCHANGE);
	return rc522_write_reg(pnd, REG_CommandReg, REG_CommandReg_RcvOff | CMD_SOFTRESET); 
}

// NXP MFRC522 datasheet section 16.1.1
const uint8_t MFRC522_V1_SELFTEST[FIFO_SIZE] = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
const uint8_t MFRC522_V2_SELFTEST[FIFO_SIZE] = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95, 0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82, 0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81, 0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D, 0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};

int rc522_self_test(struct nfc_device * pnd) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_self_test");
	#endif
	const uint8_t * correct;
	switch (CHIP_DATA(pnd)->version) {
		case MFRC522_V1:
			correct = MFRC522_V1_SELFTEST;
			break;

		case MFRC522_V2:
			correct = MFRC522_V2_SELFTEST;
			break;

		default:
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Aborting self test for unknown version %02X.", CHIP_DATA(pnd)->version);
			return NFC_EDEVNOTSUPP;
	}

	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Executing self test");

	uint8_t zeroes[25];
	memset(zeroes, 0x00, sizeof(zeroes));

	int ret;
	//Make sure that used registers are set as expected in terms of meaning by set(1|2)
	CHK(rc522_write_reg(pnd, REG_DivIrqReg, 0x80));
	CHK(rc522_write_reg(pnd, REG_ComIrqReg, 0x80));
	
	// MFRC522 datasheet section 16.1.1
	// 1. Perform a soft reset
	CHK(rc522_soft_reset(pnd));
	// 2. Clear the internal buffer by writing 25 bytes of 0x00 and execute the Mem command
	CHK(rc522_write_bulk(pnd, REG_FIFODataReg, zeroes, sizeof(zeroes)));
	CHK(rc522_start_command(pnd, CMD_MEM));
	// 3. Enable the self test by writing 0x09 to the AutoTestReg register
	CHK(rc522_write_reg(pnd, REG_AutoTestReg, REG_AutoTestReg_SelfTest_Enabled));
	// 4. Write 0x00h to the FIFO buffer
	CHK(rc522_write_reg(pnd, REG_FIFODataReg, 0x00));
	// 5. Start the self test with the CalcCRC command
	CHK(rc522_start_command(pnd, CMD_CALCCRC));

	// 6. Wait for the RC522 to calculate the selftest values
	// The official datasheet does not mentions how much time does it take, let's use 50ms
	timeout_t to;
	timeout_init(&to, 150);

	while (1) {
		if (!timeout_check(&to)) {
			log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Self test timeout");
			return NFC_ETIMEOUT;
		}

		CHK(rc522_read_reg(pnd, REG_DivIrqReg));

		// If the RC522 has finished calculating the CRC proceed
		if ((ret & REG_DivIrqReg_CRCIRq)==0) {
			break;
		}
		
	}

	uint8_t response[FIFO_SIZE];
	// 7. Read selftest result
	CHK(rc522_read_bulk(pnd, REG_FIFODataReg, response, FIFO_SIZE));
	// 8. Disable selftest operation mode
	CHK(rc522_write_reg_mask(pnd, REG_AutoTestReg, REG_AutoTestReg_SelfTest_Disabled, REG_AutoTestReg_SelfTest_MASK));

	if (memcmp(correct, response, FIFO_SIZE) != 0) {
		log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Self test values didn't match");
		return NFC_ECHIP;
	}

	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Self test executed successfully!");
	return NFC_SUCCESS;
}

int rc522_init(struct nfc_device * pnd) {
	#ifdef func_DEBUG 
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Function: rc522_init");
	#endif
	int ret;
	
	CHK(rc522_start_command(pnd, CMD_IDLE));

	int version = CHK(rc522_read_reg(pnd, REG_VersionReg));
	CHIP_DATA(pnd)->version = version;

	//gets overridden by self-test reset- needs to be set after soft-reset
	//CHK(rc522_write_reg(pnd, REG_TxASKReg, 0x40));
	//CHK(rc522_write_reg(pnd, REG_ModeReg, 0x3D));
	//CHK(rc522_write_reg(pnd, REG_TxControlReg, 0x83));

	ret = rc522_self_test(pnd);
	if (ret == NFC_EDEVNOTSUPP) {
		// TODO: Implement another test, maybe?
		ret = rc522_soft_reset(pnd);
	}
	//adding enable a global 25ms timeout in the chipset
	CHK(rc522_write_reg(pnd, REG_TModeReg, 0x80));
	CHK(rc522_write_reg(pnd, REG_TPrescalerReg, 0xA9));
	CHK(rc522_write_reg(pnd, REG_TReloadRegH, 0x03));
	CHK(rc522_write_reg(pnd, REG_TReloadRegL, 0xE8));
	
	
	CHK(rc522_write_reg(pnd, REG_TxASKReg, 0x40));
	CHK(rc522_write_reg(pnd, REG_ModeReg, 0x3D));
	CHK(rc522_write_reg(pnd, REG_TxControlReg, 0x83));

	return ret;
}
