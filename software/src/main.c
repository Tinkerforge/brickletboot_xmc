/* brickletboot
 * Copyright (C) 2016 Olaf Lüke <olaf@tinkerforge.com>
 *
 * main.c: Bricklet bootloader
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



/*

For Bricklets with co-processor we use the following flash memory map
(in this case for 16kb of flash):

-- BOOTLOADER --------------------------------------------------------------
| brickletboot                                                             |
| 0000-2000                                                                |
----------------------------------------------------------------------------

-- FIRMWARE ----------------------------------------------------------------
| firmware      | firmware version      | device id    | firmware crc      |
| 2000-3ff4     | 3ff4-3ff8             | 3ff8-3ffc    | 3ffc-4000         |
----------------------------------------------------------------------------

*/

#include "xmc_gpio.h"
#include "xmc_scu.h"
#include "configs/config.h"

#include "bootloader_spitfp.h"
#include "tfp_common.h"
#include "boot.h"

#include "bricklib2/bootloader/bootloader.h"

#if UC_SERIES == XMC11  || UC_SERIES == XMC12 || UC_SERIES == XMC13
#define TICK_COUNT_FOR_1MS 24 // 24 ticks @32mhz = 1ms
#elif UC_SERIES == XMC14
#define TICK_COUNT_FOR_1MS 36 // 36 ticks @48mhz = 1ms
#endif

const uint32_t device_identifier __attribute__ ((section(".device_identifier"))) = BOOTLOADER_DEVICE_IDENTIFIER;
const uint32_t bootloader_version __attribute__ ((section(".bootloader_version"))) = (BOOTLOADER_VERSION_MAJOR << 16) | (BOOTLOADER_VERSION_MINOR << 8) | (BOOTLOADER_VERSION_REVISION << 0);

void main_led_init(void) {
	XMC_GPIO_CONFIG_t led;
	led.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
	led.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW;
	XMC_GPIO_Init(BOOTLOADER_STATUS_LED_PIN, &led);
}

BootloaderStatus bootloader_status;
int main(void) {
	// Enable LED and temperature measurement for
	// bootloader as well as firmware
	main_led_init();
	XMC_SCU_StartTempMeasurement();

	// Jump to firmware if we can
	const uint8_t can_jump_to_firmware = boot_can_jump_to_firmware();
	if(can_jump_to_firmware == TFP_COMMON_SET_BOOTLOADER_MODE_STATUS_OK) {
		XMC_GPIO_SetOutputLow(BOOTLOADER_STATUS_LED_PIN);
		boot_jump_to_firmware();
	}

	// By default we turn status LED off in bootloader mode
	XMC_GPIO_SetOutputHigh(BOOTLOADER_STATUS_LED_PIN);

	bootloader_status.boot_mode = BOOT_MODE_BOOTLOADER;
	bootloader_status.status_led_config = 0;
	bootloader_status.system_timer_tick = 0;
	bootloader_status.reboot_started_at = 0;

	spitfp_init(&bootloader_status.st);

	uint8_t tick_counter = 0;
	while(true) {
		tick_counter++;
		if(tick_counter >= TICK_COUNT_FOR_1MS) {
			bootloader_status.system_timer_tick++;
			tick_counter = 0;
		}

		spitfp_tick(&bootloader_status);
	}
}
