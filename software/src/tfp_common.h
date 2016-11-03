/* brickletboot
 * Copyright (C) 2016 Olaf Lüke <olaf@tinkerforge.com>
 *
 * tfp_common.h: Tinkerforge Protocol (TFP) common messages
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

#ifndef TFP_COMMON_H
#define TFP_COMMON_H

#include <stdbool.h>

#define TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_OK                          0
#define TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_INVALID_MODE                1
#define TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_NO_CHANGE                   2
#define TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_ENTRY_FUNCTION_NOT_PRESENT  3
#define TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_DEVICE_IDENTIFIER_INCORRECT 4
#define TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_CRC_MISMATCH                5

#include "bootloader_spitfp.h"
#include "bricklib2/protocols/tfp/tfp.h"

void tfp_common_handle_message(const void *message, const uint8_t length, BootloaderStatus *bs);
void tfp_common_handle_reset(BootloaderStatus *bs);

#endif
