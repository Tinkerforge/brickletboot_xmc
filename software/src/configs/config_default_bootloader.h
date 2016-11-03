/* brickletboot
 * Copyright (C) 2016 Olaf Lüke <olaf@tinkerforge.com>
 *
 * config.c: Default bootloader configurations (for test compilation)
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

#ifndef CONFIG_DEFAULT_BOOTLOADER_H
#define CONFIG_DEFAULT_BOOTLOADER_H

// --- Basic defines ---
#define BOOTLOADER_DEVICE_IDENTIFIER   29999
#define BOOTLOADER_HW_VERSION_MAJOR    1
#define BOOTLOADER_HW_VERSION_MINOR    0
#define BOOTLOADER_HW_VERSION_REVISION 0

#define BOOTLOADER_STATUS_LED_PIN      P1_5

// --- SPITFP ---
#define SPITFP_RECEIVE_BUFFER_SIZE    1024
#define SPITFP_RECEIVE_BUFFER_MASK    (SPITFP_RECEIVE_BUFFER_SIZE-1) // If size is power of 2

// --- BOOTLOADER FUNCTIONS ---

// define bootloader functions to put them into the BootloaderFunctions struct
// These functions are in the bootloader code and can be used from the firmware

#define BOOTLOADER_FUNCTION_SPITFP_TICK
#define BOOTLOADER_FUNCTION_SEND_ACK_AND_MESSAGE
#define BOOTLOADER_FUNCTION_SPITFP_IS_SEND_POSSIBLE
//#define BOOTLOADER_FUNCTION_SPI_INIT
//#define BOOTLOADER_FUNCTION_TINYDMA_GET_CHANNEL_CONFIG_DEFAULTS
//#define BOOTLOADER_FUNCTION_TINYDMA_START_TRANSFER
//#define BOOTLOADER_FUNCTION_TINYDMA_DESCRIPTOR_GET_CONFIG_DEFAULTS
//#define BOOTLOADER_FUNCTION_TINYDMA_DESCRIPTOR_INIT
//#define BOOTLOADER_FUNCTION_TINYDMA_CHANNEL_INIT
#define BOOTLOADER_FUNCTION_AEABI_IDIV
#define BOOTLOADER_FUNCTION_AEABI_UIDIV
#define BOOTLOADER_FUNCTION_AEABI_IDIVMOD
#define BOOTLOADER_FUNCTION_AEABI_UIDIVMOD

#endif
