/* brickletboot
 * Copyright (C) 2010 Olaf Lüke <olaf@tinkerforge.com>
 *
 * spitfp.h: Tinkerforge Protocol (TFP) over SPI implementation
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

#ifndef BOOTLOADER_SPITFP_H
#define BOOTLOADER_SPITFP_H

#include "configs/config.h"

#include "bricklib2/utility/ringbuffer.h"
#include "bricklib2/protocols/tfp/tfp.h"
#include "bricklib2/protocols/spitfp/spitfp.h"
#include "bricklib2/bootloader/bootloader.h"

void spitfp_init(SPITFP *st);
void spitfp_tick(BootloaderStatus *bootloader_status);
bool spitfp_is_send_possible(SPITFP *st);
void spitfp_send_ack_and_message(BootloaderStatus *bs, uint8_t *data, const uint8_t length);
void spitfp_send_ack(BootloaderStatus *bs);

#endif
