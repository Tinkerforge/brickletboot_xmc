/* brickletboot
 * Copyright (C) 2016-2018 Olaf Lüke <olaf@tinkerforge.com>
 *
 * spitfp.c: Tinkerforge Protocol (TFP) over SPI implementation
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

---- SPITFP (Tinkerforge Protocol over SPI) details ----

SPITFP uses four data lines (standard SPI):
* MISO
* MOSI
* CLK
* SELECT

The protocol has three different kinds of packets:
 NoData Packet (length 1):
  * Byte 0: 0

 ACK Packet (length 3):
  * Byte 0: Length: 3
  * Byte 1: Sequence Number: 0 | (last_sequence_number_seen << 4)
  * Byte 2: Checksum

 Data Packet (length 3 + payload length):
  * Byte 0: Length
  * Byte 1: Sequence Numbers: current_sequence_number | (last_sequence_number_seen << 4)
  * Bytes 2-(n-1): Payload
  * Byte n: Checksum

Protocol information:
* Master polls with NoData Packet (i.e. zero bytes) if no data is to be sent
* Slave answers with NoData packet if no data is to be sent
* Only send next packet if ACK was received. (i.e. only one packet can be "in flight" in each direction)
* Send timeout is 5ms (re-send if no ACK is received after 5ms)

* Increase sequence number if ACK was received
* Sequence number runs from 0x2 to 0xF (0 is for ACK Packet only; 1 is for the very first enumerate request, see below)
* Sequence number 1 is reserved, because the bricklets will always answer with a complete packet to this sequence number:
  If a brick resets without resetting the bricklets it could otherwise happen that the CoMCU enumerate request is ignored because
  the bricklet has just seen exactly this packet. The bricklet then sends the acknowledgement only, but no response packet.

* Checksum is a pearson hash over all bytes of the packet (except the checksum byte itself)
* see bricklib2/utility/pearson_hash.c

* SPI MISO/MOSI data is just written to ringbuffer (if possible directly through dma)
* Compared to the SPI stack protocol, this protocol is made for slow SPI clock speeds

Bricklet behaviour:
* Bricklet waits one second after being clocked per SPI for an enumerate request
* If request is not received (e.g. if the Bricklet was hot-plugged to an already running Master Brick) one is injected into the Bricklet's receive buffer (see spitfp_handle_hotplug)
* The bricklet then answers the request. This packet contains an acknowledgement for the enumerate request.
* This can result in the Bricklet acking a packet that was never sent (because it was injected by the Bricklet itself).
* When communicating with a Bricklet, either the enumerate request must be sent (or just wait for the self-injected one) and the response must be handled correctly.
* If other packets are sent to the bricklet beforehand, it is possible, that the injection overwrites parts of the other packet in the Bricklet's receive buffer.
* Note: The injected enumerate request is a co_mcu_enumerate (FID=252) not the "normal" enumerate (FID=254). This results in a enumerate callback with type CONNECTED instead of AVAILABLE.
* If the enumerate request is sent by the other side, it has to have the sequence number 1.
*/

#include "bootloader_spitfp.h"

#include <string.h>
#include <stdint.h>

#include "configs/config.h"

#include "tfp_common.h"
#include "xmc_spi.h"
#include "xmc_gpio.h"
#include "xmc_wdt.h"

#include "bricklib2/utility/pearson_hash.h"
#include "bricklib2/utility/led_flicker.h"
#include "bricklib2/logging/logging.h"

#include "bricklib2/protocols/spitfp/spitfp.h"

void spitfp_init(SPITFP *st) {
	st->last_sequence_number_seen = 0;
	st->current_sequence_number = 0; // Initialize sequence number as 0, so the first one will be written as 1.
	st->buffer_send_pointer = st->buffer_send;
	st->buffer_send_pointer_end = st->buffer_send;

	// Configure ring buffer
	memset(&st->buffer_recv, 0, SPITFP_RECEIVE_BUFFER_SIZE);
	ringbuffer_init(&st->ringbuffer_recv, SPITFP_RECEIVE_BUFFER_SIZE, st->buffer_recv);


	// USIC channel configuration
	const XMC_SPI_CH_CONFIG_t channel_config = {
	  .bus_mode     = XMC_SPI_CH_BUS_MODE_SLAVE,
	  .parity_mode   = XMC_USIC_CH_PARITY_MODE_NONE
	};

	// MISO pin configuration
	const XMC_GPIO_CONFIG_t miso_pin_config = {
	  .mode             = SPITFP_MISO_PIN_AF,
	  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};

	// MOSI pin configuration
	const XMC_GPIO_CONFIG_t mosi_pin_config = {
	  .mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
	  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
	  .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_LARGE
	};

	// SCLK pin configuration
	const XMC_GPIO_CONFIG_t sclk_pin_config = {
	  .mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
	  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
	  .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_LARGE
	};

	// SELIN pin configuration
	const XMC_GPIO_CONFIG_t slavesel_pin_config = {
	  .mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
	  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
	  .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_LARGE
	};

	// Configure SCLK pin
	XMC_GPIO_Init(SPITFP_SCLK_PIN, &sclk_pin_config);

	// Configure slave select pin
	XMC_GPIO_Init(SPITFP_SELECT_PIN, &slavesel_pin_config);

	//Configure MOSI pin
	XMC_GPIO_Init(SPITFP_MOSI_PIN, &mosi_pin_config);

	// Initialize USIC channel in SPI slave mode
	XMC_SPI_CH_Init(SPITFP_USIC, &channel_config);
	SPITFP_USIC->SCTR &= ~USIC_CH_SCTR_PDL_Msk; // Set passive data level to 0

	XMC_SPI_CH_SetBitOrderMsbFirst(SPITFP_USIC);

	XMC_SPI_CH_SetWordLength(SPITFP_USIC, (uint8_t)8U);
	XMC_SPI_CH_SetFrameLength(SPITFP_USIC, (uint8_t)64U);

	//Set input source path
	XMC_SPI_CH_SetInputSource(SPITFP_USIC, SPITFP_MOSI_INPUT,   SPITFP_MOSI_SOURCE);
	XMC_SPI_CH_SetInputSource(SPITFP_USIC, SPITFP_SCLK_INPUT,   SPITFP_SCLK_SOURCE);
	XMC_SPI_CH_SetInputSource(SPITFP_USIC, SPITFP_SELECT_INPUT, SPITFP_SELECT_SOURCE);
	XMC_SPI_CH_EnableInputInversion(SPITFP_USIC, XMC_SPI_CH_INPUT_SLAVE_SELIN);

	// SPI Mode 0: CPOL=0 and CPHA=0
	SPITFP_USIC_CHANNEL->DX1CR |= USIC_CH_DX1CR_DPOL_Msk;
	//SPITFP_USIC_CHANNEL->PCR |= USIC_CH_PCR_SSCMode_SLPHSEL_Msk; // This does not work, see errata USIC_AI.017

	// Configure transmit FIFO
	XMC_USIC_CH_TXFIFO_Configure(SPITFP_USIC, SPITFP_TX_DATA_POINTER, SPITFP_TX_SIZE, SPITFP_TX_LIMIT);

	// Configure receive FIFO
	XMC_USIC_CH_RXFIFO_Configure(SPITFP_USIC, SPITFP_RX_DATA_POINTER, SPITFP_RX_SIZE, SPITFP_RX_LIMIT);

	// Set service request for tx FIFO transmit interrupt
	XMC_USIC_CH_TXFIFO_SetInterruptNodePointer(SPITFP_USIC, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, SPITFP_SERVICE_REQUEST_TX);  // IRQ SPITFP_IRQ_TX

	// Set service request for rx FIFO receive interrupt
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(SPITFP_USIC, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, SPITFP_SERVICE_REQUEST_RX);  // IRQ SPITFP_IRQ_RX
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(SPITFP_USIC, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, SPITFP_SERVICE_REQUEST_RX); // IRQ SPITFP_IRQ_RX

	//Set priority and enable NVIC node for transmit interrupt
	NVIC_SetPriority((IRQn_Type)SPITFP_IRQ_TX, SPITFP_IRQ_TX_PRIORITY);
	NVIC_EnableIRQ((IRQn_Type)SPITFP_IRQ_TX);

	// Set priority and enable NVIC node for receive interrupt
	NVIC_SetPriority((IRQn_Type)SPITFP_IRQ_RX, SPITFP_IRQ_RX_PRIORITY);
	NVIC_EnableIRQ((IRQn_Type)SPITFP_IRQ_RX);

	// Start SPI
	XMC_SPI_CH_Start(SPITFP_USIC);

	// Initialize SPI Slave MISO pin
	XMC_GPIO_Init(SPITFP_MISO_PIN, &miso_pin_config);
	XMC_GPIO_SetHardwareControl(SPITFP_MISO_PIN, XMC_GPIO_HWCTRL_DISABLED);

	XMC_USIC_CH_EnableEvent(SPITFP_USIC, (uint32_t)((uint32_t)XMC_USIC_CH_EVENT_STANDARD_RECEIVE | (uint32_t)XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE));
}

uint8_t spitfp_get_sequence_byte(SPITFP *st, const bool increase) {
	if(increase) {
		st->current_sequence_number++;
		if(st->current_sequence_number > 0xF) {
			st->current_sequence_number = 2;
		}
	}

	return st->current_sequence_number | (st->last_sequence_number_seen << 4);
}

void spitfp_send_ack_and_message(BootloaderStatus *bs, uint8_t *data, const uint8_t length) {
	// Increase counter for outgoing message
	led_flicker_increase_counter(&bs->led_flicker_state);

	SPITFP *st = &bs->st;
	uint8_t checksum = 0;
	const uint8_t buffer_send_length = length + SPITFP_PROTOCOL_OVERHEAD;
#ifdef BOOTLOADER_FIX_POINTER_END
	st->buffer_send_pointer_end = st->buffer_send + buffer_send_length;
#else
	st->buffer_send_pointer_end = st->buffer_send + buffer_send_length - 1;
#endif
	st->buffer_send[0] = buffer_send_length;
	PEARSON(checksum, buffer_send_length);

	st->buffer_send[1] = spitfp_get_sequence_byte(st, true);
	PEARSON(checksum, st->buffer_send[1]);

	for(uint8_t i = 0; i < length; i++) {
		st->buffer_send[2+i] = data[i];
		PEARSON(checksum, st->buffer_send[2+i]);
	}

	st->buffer_send[length + SPITFP_PROTOCOL_OVERHEAD-1] = checksum;

	st->buffer_send_pointer = st->buffer_send;
	XMC_USIC_CH_TXFIFO_EnableEvent(SPITFP_USIC, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
	XMC_USIC_CH_TriggerServiceRequest(SPITFP_USIC, SPITFP_SERVICE_REQUEST_TX);

	st->last_send_started = bs->system_timer_tick;
}

void spitfp_send_ack(BootloaderStatus *bs) {
	SPITFP *st = &bs->st;
	// Set new sequence number and checksum for ACK
	st->buffer_send[0] = SPITFP_PROTOCOL_OVERHEAD;
	st->buffer_send[1] = st->last_sequence_number_seen << 4;
	st->buffer_send[2] = pearson_permutation[pearson_permutation[st->buffer_send[0]] ^ st->buffer_send[1]];

#ifdef BOOTLOADER_FIX_POINTER_END
	bs->st.buffer_send_pointer_end = st->buffer_send + SPITFP_PROTOCOL_OVERHEAD;
#else
	bs->st.buffer_send_pointer_end = st->buffer_send + SPITFP_PROTOCOL_OVERHEAD - 1;
#endif
	bs->st.buffer_send_pointer = st->buffer_send;

	XMC_USIC_CH_TXFIFO_EnableEvent(SPITFP_USIC, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
	XMC_USIC_CH_TriggerServiceRequest(SPITFP_USIC, SPITFP_SERVICE_REQUEST_TX);

	st->last_send_started = bs->system_timer_tick;
}

bool spitfp_is_send_possible(SPITFP *st) {
	return st->buffer_send_pointer_end == st->buffer_send;
}

void spitfp_check_message_send_timeout(BootloaderStatus *bs) {
	// If there is still data in the buffer (we did not receive an ack) and
	// we are currently not sending data and was longer then SPITFP_TIMEOUT ms
	// ago that we send data, we will try again!
	if((bs->st.buffer_send_pointer_end > bs->st.buffer_send) &&
	   (bs->st.buffer_send_pointer >= bs->st.buffer_send_pointer_end) &&
	   ((bs->system_timer_tick - bs->st.last_send_started) >= SPITFP_TIMEOUT)) {

		// We leave the old message the same and try again
		bs->st.buffer_send_pointer = bs->st.buffer_send;

		// Update sequence number of send buffer. We don't increase the current sequence
		// number, but if we have seen a new message from the master we insert
		// the updated "last seen sequence number".
		// If the number changed we also have to update the checksum.
		uint8_t new_sequence_byte = spitfp_get_sequence_byte(&bs->st, false);
		if(new_sequence_byte != bs->st.buffer_send[1]) {
			bs->st.buffer_send[1] = new_sequence_byte;
			uint8_t checksum = 0;
			for(uint8_t i = 0; i < bs->st.buffer_send[0]-1; i++) {
				PEARSON(checksum, bs->st.buffer_send[i]);
			}
			bs->st.buffer_send[bs->st.buffer_send[0]-1] = checksum;
		}

		bs->st.last_send_started = bs->system_timer_tick;
		XMC_USIC_CH_TXFIFO_EnableEvent(SPITFP_USIC, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
		XMC_USIC_CH_TriggerServiceRequest(SPITFP_USIC, SPITFP_SERVICE_REQUEST_TX);
	}
}

void spitfp_handle_protocol_error(SPITFP *st) {
	// In case of error we completely empty the ringbuffer
	uint8_t data;
	while(ringbuffer_get(&st->ringbuffer_recv, &data));
}

void spitfp_handle_hotplug(BootloaderStatus *bootloader_status) {
	if(bootloader_status->hotplug_time == UINT32_MAX) {
		// Startup already handled
		return;
	} else if(bootloader_status->hotplug_time == 0) {
		// We got the first data, start timer!
		bootloader_status->hotplug_time = bootloader_status->system_timer_tick;
	} else if((bootloader_status->system_timer_tick - bootloader_status->hotplug_time) >= SPITFP_HOTPLUG_TIMEOUT) {
		// Disable RX interrupts so we can't accidentally
		// receive data in between the ringbuffer_adds
		NVIC_DisableIRQ((IRQn_Type)SPITFP_IRQ_RX);
		__DSB();
		__ISB();

		// Lets fake a co mcu enumerate message
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x0B);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x01);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x00);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x00);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x00);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x00);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x08);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0xFC);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x00);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0x00);
		ringbuffer_add(&bootloader_status->st.ringbuffer_recv, 0xCC);

		// Enable interrupt again
		NVIC_EnableIRQ((IRQn_Type)SPITFP_IRQ_RX);

		bootloader_status->hotplug_time = bootloader_status->system_timer_tick;
	}
}

void spitfp_tick(BootloaderStatus *bootloader_status) {
	XMC_WDT_Service();
	SPITFP *st = &bootloader_status->st;

	tfp_common_handle_reset(bootloader_status);
	led_flicker_tick(&bootloader_status->led_flicker_state, bootloader_status->system_timer_tick, BOOTLOADER_STATUS_LED_PIN);

	// If the temporary buffer length is > 0 we still have a message to handle
	if(st->buffer_recv_tmp_length > 0) {
		if(spitfp_is_send_possible(st)) {
			if(st->buffer_recv_tmp_length == SPITFP_PROTOCOL_OVERHEAD) {
				// If the length is set to SPITFP_PROTOCOL_OVERHEAD we just have to answer with an ACK
				spitfp_send_ack(bootloader_status);
			} else {
				// Otherwise we send the whole message with the corresponding length
				tfp_common_handle_message(st->buffer_recv_tmp, st->buffer_recv_tmp_length, bootloader_status);
			}

			// In both cases the length is set to 0 afterwards and the temporary buffer can be written again.
			st->buffer_recv_tmp_length = 0;
		}
	}

	spitfp_check_message_send_timeout(bootloader_status);

	uint8_t message[TFP_MESSAGE_MAX_LENGTH] = {0};
	uint8_t message_position = 0;
	uint16_t num_to_remove_from_ringbuffer = 0;
	uint8_t checksum = 0;

	uint8_t data_sequence_number = 0;
	uint8_t data_length = 0;

	SPITFPState state = SPITFP_STATE_START;
	uint16_t used = ringbuffer_get_used(&st->ringbuffer_recv);
	uint16_t start = st->ringbuffer_recv.start;

	if(used > 0) {
		spitfp_handle_hotplug(bootloader_status);
		used = ringbuffer_get_used(&st->ringbuffer_recv);
	}

	for(uint16_t i = start; i < start+used; i++) {
		const uint16_t index = i & SPITFP_RECEIVE_BUFFER_MASK;
		const uint8_t data = st->buffer_recv[index];

		// Handle "standard case" first (we are sending data and Master has nothing to send)
		if(state == SPITFP_STATE_START && data == 0) {
			// equivalent (but faster) to "ringbuffer_remove(&st->ringbuffer_recv, 1);"
			st->ringbuffer_recv.start = (st->ringbuffer_recv.start + 1) & SPITFP_RECEIVE_BUFFER_MASK;
			continue;
		}
		num_to_remove_from_ringbuffer++;

		switch(state) {
			case SPITFP_STATE_START: {
				checksum = 0;
				message_position = 0;

				if(data == SPITFP_PROTOCOL_OVERHEAD) {
					state = SPITFP_STATE_ACK_SEQUENCE_NUMBER;
				} else if(data >= SPITFP_MIN_TFP_MESSAGE_LENGTH && data <= SPITFP_MAX_TFP_MESSAGE_LENGTH) {
					state = SPITFP_STATE_MESSAGE_SEQUENCE_NUMBER;
				} else if(data == 0) {
					// equivalent (but faster) to "ringbuffer_remove(&st->ringbuffer_recv, 1);"
					st->ringbuffer_recv.start = (st->ringbuffer_recv.start + 1) & SPITFP_RECEIVE_BUFFER_MASK;
					num_to_remove_from_ringbuffer--;
					break;
				} else {
					// If the length is not PROTOCOL_OVERHEAD or within [MIN_TFP_MESSAGE_LENGTH, MAX_TFP_MESSAGE_LENGTH]
					// or 0, something has gone wrong!
					bootloader_status->error_count.error_count_frame++;
					spitfp_handle_protocol_error(st);
					return;
				}

				data_length = data;
				if((start+used - i) < data_length) {
					// There can't be enough data for a whole message, we can return here.
					return;
				}
				PEARSON(checksum, data_length);

				break;
			}

			case SPITFP_STATE_ACK_SEQUENCE_NUMBER: {
				data_sequence_number = data;
				PEARSON(checksum, data_sequence_number);
				state = SPITFP_STATE_ACK_CHECKSUM;
				break;
			}

			case SPITFP_STATE_ACK_CHECKSUM: {
				// Whatever happens here, we will go to start again and remove
				// data from ringbuffer
				state = SPITFP_STATE_START;
				ringbuffer_remove(&st->ringbuffer_recv, num_to_remove_from_ringbuffer);
				num_to_remove_from_ringbuffer = 0;

				if(checksum != data) {
					bootloader_status->error_count.error_count_ack_checksum++;
					spitfp_handle_protocol_error(st);
					return;
				}

				uint8_t last_sequence_number_seen_by_master = (data_sequence_number & 0xF0) >> 4;
				if(last_sequence_number_seen_by_master == st->current_sequence_number) {
					// If we got a timeout and are now re-sending the message, it
					// is possible that we are currently sending this message again.
					// Check if it was send completely
					if(st->buffer_send_pointer == st->buffer_send_pointer_end) {
						st->buffer_send_pointer_end = st->buffer_send;
					}
				}

				break;
			}

			case SPITFP_STATE_MESSAGE_SEQUENCE_NUMBER: {
				data_sequence_number = data;
				PEARSON(checksum, data_sequence_number);
				state = SPITFP_STATE_MESSAGE_DATA;
				break;
			}

			case SPITFP_STATE_MESSAGE_DATA: {
				message[message_position] = data;
				message_position++;

				PEARSON(checksum, data);

				if(message_position == data_length - SPITFP_PROTOCOL_OVERHEAD) {
					state = SPITFP_STATE_MESSAGE_CHECKSUM;
				}
				break;
			}

			case SPITFP_STATE_MESSAGE_CHECKSUM: {
				// Whatever happens here, we will go to start again
				state = SPITFP_STATE_START;

				// Remove data from ringbuffer. If we can't send it we can't handle
				// it at the moment we will wait for the SPI master to re-send it.
				ringbuffer_remove(&st->ringbuffer_recv, num_to_remove_from_ringbuffer);
				num_to_remove_from_ringbuffer = 0;

				if(checksum != data) {
					bootloader_status->error_count.error_count_message_checksum++;
					spitfp_handle_protocol_error(st);
					return;
				}

				uint8_t last_sequence_number_seen_by_master = (data_sequence_number & 0xF0) >> 4;
				if(last_sequence_number_seen_by_master == st->current_sequence_number) {
					// If we got a timeout and are now re-sending the message, it
					// is possible that we are currently sending this message again.
					// Check if it was send completely
					if(st->buffer_send_pointer == st->buffer_send_pointer_end) {
						st->buffer_send_pointer_end = st->buffer_send;
					}
				}

				// If we already have one recv message in the temporary buffer,
				// we don't handle the newly received message and just throw it away.
				// The SPI master will send it again.
				if(st->buffer_recv_tmp_length == 0) {
					// If sequence number is new, we can handle the message.
					// Otherwise we only ACK the already handled message again.
					const uint8_t message_sequence_number = data_sequence_number & 0x0F;
					if((message_sequence_number != st->last_sequence_number_seen) || (message_sequence_number == 1)) {
						// For the special case that the sequence number is 1 (only used for the very first message)
						// we always send an answer, even if we havn't seen anything else in between.
						// Otherwise it is not possible to reset the Master Brick if no messages were exchanged before
						// the reset

						st->last_sequence_number_seen = message_sequence_number;
						// The handle message function will send an ACK for the message
						// if it can handle the message at the current moment.
						// Otherwise it will save the message and length for it it be send
						// later on.
						if(spitfp_is_send_possible(st)) {
							tfp_common_handle_message(message, message_position, bootloader_status);
						} else {
							st->buffer_recv_tmp_length = message_position;
							memcpy(st->buffer_recv_tmp, message, message_position);
						}
					} else {
						if(spitfp_is_send_possible(st)) {
							spitfp_send_ack(bootloader_status);
						} else {
							// If we want to send an ACK but currently can't, we set the
							// temporary recv buffer length to SPITFP_PROTOCOL_OVERHEAD.
							st->buffer_recv_tmp_length = SPITFP_PROTOCOL_OVERHEAD;
						}
					}
				}
				return;
			}
		}
	}
}
