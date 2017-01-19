/* brickletboot
 * Copyright (C) 2016 Olaf Lüke <olaf@tinkerforge.com>
 *
 * firmware_entry.c: Entry function for firmware
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

#include "bricklib2/bootloader/bootloader.h"

#include "bootloader_spitfp.h"
#include "tfp_common.h"

#ifdef BOOTLOADER_FUNCTION_AEABI_IDIV
int __aeabi_idiv(int a, int b);
#endif

#ifdef BOOTLOADER_FUNCTION_AEABI_UIDIV
unsigned int __aeabi_uidiv(unsigned int a, unsigned int b);
#endif

#ifdef BOOTLOADER_FUNCTION_AEABI_IDIVMOD
uint64_t __aeabi_idivmod(int a, int b);
#endif

#ifdef BOOTLOADER_FUNCTION_AEABI_UIDIVMOD
uint64_t __aeabi_uidivmod(unsigned int a, unsigned int b);
#endif

void firmware_entry_handle(BootloaderFunctions *bf, BootloaderStatus *bs) {
#ifdef BOOTLOADER_FUNCTION_SPITFP_TICK
	bf->spitfp_tick = spitfp_tick;
#endif

#ifdef BOOTLOADER_FUNCTION_SEND_ACK_AND_MESSAGE
	bf->spitfp_send_ack_and_message = spitfp_send_ack_and_message;
#endif

#ifdef BOOTLOADER_FUNCTION_SPITFP_IS_SEND_POSSIBLE
	bf->spitfp_is_send_possible = spitfp_is_send_possible;
#endif

#ifdef BOOTLOADER_FUNCTION_GET_UID
	bf->get_uid = tfp_common_get_uid;
#endif

#ifdef BOOTLOADER_FUNCTION_DSU_CRC32_CAL
	bf->dsu_crc32_cal = dsu_crc32_cal;
#endif

#ifdef BOOTLOADER_FUNCTION_SPI_INIT
	bf->spi_init = spi_init;
#endif

#ifdef BOOTLOADER_FUNCTION_TINYDMA_GET_CHANNEL_CONFIG_DEFAULTS
	bf->tinydma_get_channel_config_defaults = tinydma_get_channel_config_defaults;
#endif

#ifdef BOOTLOADER_FUNCTION_TINYDMA_INIT
	bf->tinydma_init = tinydma_init;
#endif

#ifdef BOOTLOADER_FUNCTION_TINYDMA_START_TRANSFER
	bf->tinydma_start_transfer = tinydma_start_transfer;
#endif

#ifdef BOOTLOADER_FUNCTION_TINYDMA_DESCRIPTOR_GET_CONFIG_DEFAULTS
	bf->tinydma_descriptor_get_config_defaults = tinydma_descriptor_get_config_defaults;
#endif

#ifdef BOOTLOADER_FUNCTION_TINYDMA_DESCRIPTOR_INIT
	bf->tinydma_descriptor_init = tinydma_descriptor_init;
#endif

#ifdef BOOTLOADER_FUNCTION_TINYDMA_CHANNEL_INIT
	bf->tinydma_channel_init = tinydma_channel_init;
#endif

#ifdef BOOTLOADER_FUNCTION_AEABI_IDIV
	bf->__aeabi_idiv = __aeabi_idiv;
#endif

#ifdef BOOTLOADER_FUNCTION_AEABI_UIDIV
	bf->__aeabi_uidiv = __aeabi_uidiv;
#endif

#ifdef BOOTLOADER_FUNCTION_AEABI_IDIVMOD
	bf->__aeabi_idivmod = __aeabi_idivmod;
#endif

#ifdef BOOTLOADER_FUNCTION_AEABI_UIDIVMOD
	bf->__aeabi_uidivmod = __aeabi_uidivmod;
#endif

#ifdef BOOTLOADER_FUNCTION_READ_PAGE
	bf->read_eeprom_page = tfp_common_read_eeprom_page;
#endif

#ifdef BOOTLOADER_FUNCTION_WRITE_PAGE
	bf->write_eeprom_page = tfp_common_write_eeprom_page;
#endif

	spitfp_init(&bs->st);
}

// Sets functions that can be used by firmware and initializes spitfp state machine
__attribute__ ((section(".firmware_entry_func"))) void firmware_entry(BootloaderFunctions *bf, BootloaderStatus *bs) {
	firmware_entry_handle(bf, bs);
}
