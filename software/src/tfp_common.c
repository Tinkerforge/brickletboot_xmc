/* brickletboot
 * Copyright (C) 2016 Olaf Lüke <olaf@tinkerforge.com>
 *
 * tfp_common.c: Tinkerforge Protocol (TFP) common messages
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "tfp_common.h"

#include <string.h>

#include "boot.h"
#include "xmc_gpio.h"
#include "xmc_flash.h"
#include "xmc_scu.h"

#include "configs/config.h"

#include "bricklib2/protocols/tfp/tfp.h"

#define TFP_COMMON_FID_SET_BOOTLOADER_MODE 235
#define TFP_COMMON_FID_GET_BOOTLOADER_MODE 236
#define TFP_COMMON_FID_SET_WRITE_FIRMWARE_POINTER 237
#define TFP_COMMON_FID_WRITE_FIRMWARE 238
#define TFP_COMMON_FID_SET_STATUS_LED_CONFIG 239
#define TFP_COMMON_FID_GET_STATUS_LED_CONFIG 240
#define TFP_COMMON_FID_GET_PROTOCOL1_BRICKLET_NAME 241 // unused ?
#define TFP_COMMON_FID_GET_CHIP_TEMPERATURE 242 // unused ?
#define TFP_COMMON_FID_RESET 243
#define TFP_COMMON_FID_GET_ADC_CALIBRATION 250 // unused ?
#define TFP_COMMON_FID_ADC_CALIBRATE 251 // unused ?
#define TFP_COMMON_FID_CO_MCU_ENUMERATE 252
#define TFP_COMMON_FID_ENUMERATE_CALLBACK 253
#define TFP_COMMON_FID_ENUMERATE 254
#define TFP_COMMON_FID_GET_IDENTITY 255

#define TFP_COMMON_ENUMERATE_CALLBACK_UID_LENGTH 8
#define TFP_COMMON_ENUMERATE_CALLBACK_VERSION_LENGTH 3
#define TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE 64 // = page size of samd* processors

#define TFP_COMMON_ENUMERATE_TYPE_AVAILABLE 0
#define TFP_COMMON_ENUMERATE_TYPE_ADDED     1
#define TFP_COMMON_ENUMERATE_TYPE_REMOVED   2

#define TFP_COMMON_STATUS_LED_OFF                       0
#define TFP_COMMON_STATUS_LED_ON                        1
#define TFP_COMMON_STATUS_LED_SHOW_COMMUNICATION_STATUS 2

#define TFP_COMMON_WRITE_FIRMWARE_STATUS_OK              0
#define TFP_COMMON_WRITE_FIRMWARE_STATUS_INVALID_POINTER 1

#define TFP_COMMON_XMC1_BLOCK_SIZE 16
#define TFP_COMMON_XMC1_PAGE_SIZE 256
#define TFP_COMMON_XMC1_SECTOR_SIZE 4096
#define TFP_COMMON_XMC1_PAGE_MASK ((BOOTLOADER_FIRMWARE_SIZE-1) & ~(TFP_COMMON_XMC1_PAGE_SIZE-1))

#define TFP_COMMON_WAIT_BEFORE_RESET 250 // in ms

typedef struct {
	TFPMessageHeader header;
	uint8_t mode;
} __attribute__((__packed__)) TFPCommonSetBootloaderMode;

typedef struct {
	TFPMessageHeader header;
	uint8_t status;
} __attribute__((__packed__)) TFPCommonSetBootloaderModeReturn;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) TFPCommonGetBootloaderMode;

typedef struct {
	TFPMessageHeader header;
	uint8_t mode;
} __attribute__((__packed__)) TFPCommonGetBootloaderModeReturn;

typedef struct {
	TFPMessageHeader header;
	uint32_t pointer;
} __attribute__((__packed__)) TFPCommonSetWriteFirmwarePointer;

typedef struct {
	TFPMessageHeader header;
	uint8_t data[TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE];
} __attribute__((__packed__)) TFPCommonWriteFirmware;

typedef struct {
	TFPMessageHeader header;
	uint8_t status;
} __attribute__((__packed__)) TFPCommonWriteFirmwareReturn;

typedef struct {
	TFPMessageHeader header;
	uint8_t config;
} __attribute__((__packed__)) TFPCommonSetStatusLEDConfig;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) TFPCommonGetStatusLEDConfig;

typedef struct {
	TFPMessageHeader header;
	uint8_t config;
} __attribute__((__packed__)) TFPCommonGetStatusLEDConfigReturn;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) TFPCommonGetChipTemperature;

typedef struct {
	TFPMessageHeader header;
	int16_t temperature;
} __attribute__((__packed__)) TFPCommonGetChipTemperatureReturn;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) TFPCommonReset;

typedef struct {
	TFPMessageHeader header;
	uint32_t uid;
} __attribute__((packed)) TFPCommonCoMCUEnumerateReturn;

typedef struct {
	TFPMessageHeader header;
} __attribute__((packed)) TFPCommonCoMCUEnumerate;

typedef struct {
	TFPMessageHeader header;
	char uid[TFP_COMMON_ENUMERATE_CALLBACK_UID_LENGTH];
	char connected_uid[TFP_COMMON_ENUMERATE_CALLBACK_UID_LENGTH];
	char position;
	uint8_t version_hw[TFP_COMMON_ENUMERATE_CALLBACK_VERSION_LENGTH];
	uint8_t version_fw[TFP_COMMON_ENUMERATE_CALLBACK_VERSION_LENGTH];
	uint16_t device_identifier;
	uint8_t enumeration_type;
} __attribute__((__packed__)) TFPCommonEnumerateCallback;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) TFPCommonEnumerate;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) TFPCommonGetIdentity;

typedef struct {
	TFPMessageHeader header;
	char uid[TFP_COMMON_ENUMERATE_CALLBACK_UID_LENGTH];
	char connected_uid[TFP_COMMON_ENUMERATE_CALLBACK_UID_LENGTH];
	char position;
	uint8_t version_hw[TFP_COMMON_ENUMERATE_CALLBACK_VERSION_LENGTH];
	uint8_t version_fw[TFP_COMMON_ENUMERATE_CALLBACK_VERSION_LENGTH];
	uint16_t device_identifier;
} __attribute__((__packed__)) TFPCommonGetIdentityReturn;

#define TFP_FLASH_ERASE_WAIT_CYCLES 100000

#define TFP_COMMON_RETURN_MESSAGE_LENGTH 80


// This is not available if called from outside of bootloader, make sure that
// it is only used in bootloader mode!
static uint32_t tfp_common_firmware_pointer = 0;
static uint8_t tfp_common_firmware_page[TFP_COMMON_XMC1_PAGE_SIZE];

uint32_t tfp_common_get_uid(void) {
	// TODO: FIXME

	// For normal Bricks the last bit is always 1
	// For RED Bricks the last two bits are always 10
	// For co processor Bricklets the last three bits are always 100
	return 12345; //
}

BootloaderHandleMessageReturn tfp_common_set_bootloader_mode(const TFPCommonSetBootloaderMode *data, void *_return_message, BootloaderStatus *bs) {
	TFPCommonSetBootloaderModeReturn *sbmr = _return_message;
	sbmr->header = data->header;
	sbmr->header.length = sizeof(TFPCommonSetBootloaderModeReturn);

	if(data->mode > BOOT_MODE_FIRMWARE) {
		sbmr->status = TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_INVALID_MODE;
	} else if(bs->boot_mode == data->mode) {
		sbmr->status = TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_NO_CHANGE;
	} else if(data->mode == BOOT_MODE_BOOTLOADER) {
		// From Firmware to Bootloader
		sbmr->status = TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_OK;

		bs->boot_mode = BOOT_MODE_FIRMWARE_WAIT_FOR_ERASE_AND_REBOOT;
		bs->reboot_started_at = bs->system_timer_tick;
	} else if(data->mode == BOOT_MODE_FIRMWARE) {
		// From Bootloader to Firmware
		sbmr->status = boot_can_jump_to_firmware();
		if(sbmr->status == TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_OK) {
			bs->boot_mode = BOOT_MODE_BOOTLOADER_WAIT_FOR_REBOOT;
			bs->reboot_started_at = bs->system_timer_tick;
		}
	}

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}


BootloaderHandleMessageReturn tfp_common_get_bootloader_mode(const TFPCommonGetBootloaderMode *data, void *_return_message, BootloaderStatus *bs) {
	TFPCommonGetBootloaderModeReturn *gbmr = _return_message;
	gbmr->header = data->header;
	gbmr->header.length = sizeof(TFPCommonGetBootloaderModeReturn);

	gbmr->mode = bs->boot_mode;

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}

BootloaderHandleMessageReturn tfp_common_set_write_firmware_pointer(const TFPCommonSetWriteFirmwarePointer *data, void *_return_message, BootloaderStatus *bs) {
	if(bs->boot_mode != BOOT_MODE_BOOTLOADER) {
		return HANDLE_MESSAGE_RETURN_NOT_SUPPORTED;
	}

	tfp_common_firmware_pointer = data->pointer;

	return HANDLE_MESSAGE_RETURN_EMPTY;
}

BootloaderHandleMessageReturn /*__attribute__ ((section (".ram_code")))*/ tfp_common_write_firmware(const TFPCommonWriteFirmware *data, void *_return_message, BootloaderStatus *bs) {
	if(bs->boot_mode != BOOT_MODE_BOOTLOADER) {
		return HANDLE_MESSAGE_RETURN_NOT_SUPPORTED;
	}

	TFPCommonWriteFirmwareReturn *wfr = _return_message;
	wfr->header = data->header;
	wfr->header.length = sizeof(TFPCommonWriteFirmwareReturn);

	if((tfp_common_firmware_pointer > (BOOTLOADER_FIRMWARE_SIZE-TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE)) ||
	   ((tfp_common_firmware_pointer % TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE) != 0)) {
		wfr->status = TFP_COMMON_WRITE_FIRMWARE_STATUS_INVALID_POINTER;
		return HANDLE_MESSAGE_RETURN_INVALID_PARAMETER;
	}

	const uint8_t chunk_num = (tfp_common_firmware_pointer/TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE) % (TFP_COMMON_XMC1_PAGE_SIZE/TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE);
	memcpy(tfp_common_firmware_page + TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE*chunk_num, data->data, TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE);

	// If this is the last chunk of one page, we write it to flash
	if(chunk_num == ((TFP_COMMON_XMC1_PAGE_SIZE/TFP_COMMON_BOOTLOADER_WRITE_CHUNK_SIZE) - 1)) {
		__disable_irq();
		XMC_FLASH_ErasePage((uint32_t*)(BOOTLOADER_FIRMWARE_START_POS + (tfp_common_firmware_pointer & TFP_COMMON_XMC1_PAGE_MASK)));
//		for(uint32_t i = 0; i < TFP_FLASH_ERASE_WAIT_CYCLES; i++) {	__NOP(); }
		XMC_FLASH_ProgramVerifyPage((uint32_t*)(BOOTLOADER_FIRMWARE_START_POS + (tfp_common_firmware_pointer & TFP_COMMON_XMC1_PAGE_MASK)), (uint32_t*)tfp_common_firmware_page);
//		for(uint32_t i = 0; i < TFP_FLASH_ERASE_WAIT_CYCLES; i++) {	__NOP(); }
		__enable_irq();
	}

	wfr->status = TFP_COMMON_WRITE_FIRMWARE_STATUS_OK;

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}

BootloaderHandleMessageReturn tfp_common_set_status_led_config(const TFPCommonSetStatusLEDConfig *data, void *_return_message, BootloaderStatus *bs) {
	if(data->config >= TFP_COMMON_STATUS_LED_SHOW_COMMUNICATION_STATUS) {
		return HANDLE_MESSAGE_RETURN_INVALID_PARAMETER;
	}

	bs->status_led_config = data->config;

	// Set LED according to value
	if(bs->status_led_config == TFP_COMMON_STATUS_LED_OFF) {
		XMC_GPIO_SetOutputHigh(BOOTLOADER_STATUS_LED_PIN);
	} else {
		XMC_GPIO_SetOutputLow(BOOTLOADER_STATUS_LED_PIN);
	}

	return HANDLE_MESSAGE_RETURN_EMPTY;
}

BootloaderHandleMessageReturn tfp_common_get_status_led_config(const TFPCommonGetStatusLEDConfig *data, void *_return_message, BootloaderStatus *bs) {
	TFPCommonGetStatusLEDConfigReturn *gslcr = _return_message;
	gslcr->header = data->header;
	gslcr->header.length = sizeof(TFPCommonGetStatusLEDConfigReturn);

	gslcr->config = bs->status_led_config;

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}

BootloaderHandleMessageReturn tfp_common_get_chip_temperature(const TFPCommonGetChipTemperature *data, void *_return_message) {
	TFPCommonGetChipTemperatureReturn *gctr = _return_message;
	gctr->header        = data->header;
	gctr->header.length = sizeof(TFPCommonGetChipTemperatureReturn);
	gctr->temperature   = XMC_SCU_CalcTemperature() - 273;

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}

BootloaderHandleMessageReturn tfp_common_reset(const TFPCommonReset *data, void *_return_message, BootloaderStatus *bs) {
	if(bs->boot_mode == BOOT_MODE_BOOTLOADER) {
		bs->boot_mode = BOOT_MODE_BOOTLOADER_WAIT_FOR_REBOOT;
		bs->reboot_started_at = bs->system_timer_tick;
	} else if(bs->boot_mode == BOOT_MODE_FIRMWARE) {
		bs->boot_mode = BOOT_MODE_FIRMWARE_WAIT_FOR_REBOOT;
		bs->reboot_started_at = bs->system_timer_tick;
	}

	// We can ignore all other cases, in the other cases we will be rebooting shortly anyway

	return HANDLE_MESSAGE_RETURN_EMPTY;
}

BootloaderHandleMessageReturn tfp_common_get_identity(const TFPCommonGetIdentity *data, void *_return_message) {
	TFPCommonGetIdentityReturn *gir = _return_message;
	gir->header        = data->header;
	gir->header.uid    = tfp_common_get_uid();
	gir->header.length = sizeof(TFPCommonGetIdentityReturn);

	tfp_uid_uint32_to_base58(tfp_common_get_uid(), gir->uid);
	memset(gir->connected_uid, 0, TFP_COMMON_ENUMERATE_CALLBACK_UID_LENGTH);

	gir->version_hw[0] = BOOTLOADER_HW_VERSION_MAJOR;
	gir->version_hw[1] = BOOTLOADER_HW_VERSION_MINOR;
	gir->version_hw[2] = BOOTLOADER_HW_VERSION_REVISION;

	gir->version_fw[0] = (BOOTLOADER_FIRMWARE_CONFIGURATION_POINTER->firmware_version >> 16) & 0xFF;
	gir->version_fw[1] = (BOOTLOADER_FIRMWARE_CONFIGURATION_POINTER->firmware_version >> 8)  & 0xFF;
	gir->version_fw[2] = (BOOTLOADER_FIRMWARE_CONFIGURATION_POINTER->firmware_version >> 0)  & 0xFF;

	gir->device_identifier = BOOTLOADER_DEVICE_IDENTIFIER;

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}

BootloaderHandleMessageReturn tfp_common_enumerate(const TFPCommonEnumerate *data, void *_return_message) {
	// The function itself does not return anything, but we return the callback here instead.
	// We use get_identity for uids, fw version and hw version.
	// The layout of the struct it the same.
	tfp_common_get_identity((void*)data, _return_message);

	TFPCommonEnumerateCallback *ec = _return_message;
	ec->header.length           = sizeof(TFPCommonEnumerateCallback);
	ec->header.fid              = TFP_COMMON_FID_ENUMERATE_CALLBACK;
	ec->header.sequence_num     = 0; // Sequence number for callback is 0
	ec->header.return_expected  = 1;
	ec->header.authentication   = 0; // TODO
	ec->header.other_options    = 0;
	ec->header.error            = 0;
	ec->header.future_use       = 0;

	ec->enumeration_type = TFP_COMMON_ENUMERATE_TYPE_AVAILABLE;

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}

BootloaderHandleMessageReturn tfp_common_co_mcu_enumerate(const TFPCommonCoMCUEnumerate *data, void *_return_message) {
	// This is the same as enumerate, but with TFP_COMMON_ENUMERATE_TYPE_ADDED (initial enumerate)
	// This gets triggered by the Brick
	tfp_common_enumerate((void*)data, _return_message);
	((TFPCommonEnumerateCallback*)data)->enumeration_type = TFP_COMMON_ENUMERATE_TYPE_ADDED;

	return HANDLE_MESSAGE_RETURN_NEW_MESSAGE;
}

void tfp_common_handle_reset(BootloaderStatus *bs) {
	switch(bs->boot_mode) {
		case BOOT_MODE_BOOTLOADER:
		case BOOT_MODE_FIRMWARE: {
			return;
		}

		case BOOT_MODE_BOOTLOADER_WAIT_FOR_REBOOT:
		case BOOT_MODE_FIRMWARE_WAIT_FOR_REBOOT: {
			if((bs->system_timer_tick - bs->reboot_started_at) >= TFP_COMMON_WAIT_BEFORE_RESET) {
				NVIC_SystemReset();
			}
			return;
		}

		case BOOT_MODE_FIRMWARE_WAIT_FOR_ERASE_AND_REBOOT: {
			if((bs->system_timer_tick - bs->reboot_started_at) >= TFP_COMMON_WAIT_BEFORE_RESET) {
				// Turn all interrupts off here! The firmware code should not be
				// able to do anything as soon as we are at this point. Otherwise we might
				// have a race condition between the memory erase and the reset.
				__disable_irq();

				XMC_FLASH_ErasePage((uint32_t*)BOOTLOADER_FIRMWARE_START_POS);
				for(uint32_t i = 0; i < TFP_FLASH_ERASE_WAIT_CYCLES; i++) {	__NOP(); }

				NVIC_SystemReset();
			}
		}
	}
}

void tfp_common_handle_message(const void *message, const uint8_t length, BootloaderStatus *bs) {
	// Do we need to check for UID here? Or do we define that the Brick already checks this?
#if 0
	const uint32_t message_uid = tfp_get_fid_from_message(message);
	if((message_uid != tfp_common_get_uid()) && (message_uid != 0)) {
		spitfp_send_ack(&bs->st);
		return;
	}
#endif

	// TODO: Wait for ~1 second after startup for CoMCUEnumerate message. If there is none,
	//       send an "answer" to it anyway

#if 0
	const uint8_t message_length = tfp_get_length_from_message(message);
	if(length != message_length) {
		// TODO: What do we do here? Send ACK or ignore message?
		//       Should we just not check at all?
		return;
	}
#endif

	uint8_t return_message[TFP_COMMON_RETURN_MESSAGE_LENGTH];
	BootloaderHandleMessageReturn handle_message_return = HANDLE_MESSAGE_RETURN_EMPTY;

	switch(tfp_get_fid_from_message(message)) {
		case TFP_COMMON_FID_SET_BOOTLOADER_MODE:        handle_message_return = tfp_common_set_bootloader_mode(message, return_message, bs);        break;
		case TFP_COMMON_FID_GET_BOOTLOADER_MODE:        handle_message_return = tfp_common_get_bootloader_mode(message, return_message, bs);        break;
		case TFP_COMMON_FID_SET_WRITE_FIRMWARE_POINTER: handle_message_return = tfp_common_set_write_firmware_pointer(message, return_message, bs); break;
		case TFP_COMMON_FID_WRITE_FIRMWARE:             handle_message_return = tfp_common_write_firmware(message, return_message, bs);             break;
		case TFP_COMMON_FID_SET_STATUS_LED_CONFIG:      handle_message_return = tfp_common_set_status_led_config(message, return_message, bs);      break;
		case TFP_COMMON_FID_GET_STATUS_LED_CONFIG:      handle_message_return = tfp_common_get_status_led_config(message, return_message, bs);      break;
		case TFP_COMMON_FID_GET_CHIP_TEMPERATURE:       handle_message_return = tfp_common_get_chip_temperature(message, return_message);           break;
		case TFP_COMMON_FID_RESET:                      handle_message_return = tfp_common_reset(message, return_message, bs);                      break;
		case TFP_COMMON_FID_CO_MCU_ENUMERATE:           handle_message_return = tfp_common_co_mcu_enumerate(message, return_message);               break;
		case TFP_COMMON_FID_ENUMERATE:                  handle_message_return = tfp_common_enumerate(message, return_message);                      break;
		case TFP_COMMON_FID_GET_IDENTITY:               handle_message_return = tfp_common_get_identity(message, return_message);                   break;
		default: {
			if(bs->boot_mode == BOOT_MODE_FIRMWARE) {
				handle_message_return = bs->firmware_handle_message_func(message, return_message);
			} else {
				handle_message_return = HANDLE_MESSAGE_RETURN_NOT_SUPPORTED;
			}

			break;
		}
	}

	bool has_message = true;
	if(handle_message_return != HANDLE_MESSAGE_RETURN_NEW_MESSAGE) {
		has_message = tfp_is_return_expected(message);
		if(has_message) {
			TFPMessageHeader *in_header = (TFPMessageHeader*)message;
			TFPMessageHeader *ret_header = (TFPMessageHeader*)return_message;

			// Copy header from incoming message to outgoing message
			*ret_header = *in_header;

			// For these empty and error returns there is no payload (i.e. length = min length)
			ret_header->length = TFP_MESSAGE_MIN_LENGTH;
			switch(handle_message_return) {
				case HANDLE_MESSAGE_RETURN_EMPTY:             ret_header->error = TFP_MESSAGE_ERROR_CODE_OK;                break;
				case HANDLE_MESSAGE_RETURN_NOT_SUPPORTED:     ret_header->error = TFP_MESSAGE_ERROR_CODE_NOT_SUPPORTED;     break;
				case HANDLE_MESSAGE_RETURN_INVALID_PARAMETER: ret_header->error = TFP_MESSAGE_ERROR_CODE_INVALID_PARAMETER; break;
				default: break;
			}
		}
	}

	if(has_message) {
		spitfp_send_ack_and_message(bs, return_message, tfp_get_length_from_message(return_message));
	} else {
		spitfp_send_ack(bs);
	}
}
