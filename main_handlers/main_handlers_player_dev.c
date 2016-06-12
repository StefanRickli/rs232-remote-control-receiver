/*
 * main_handlers_player_dev.c
 *
 *  Created on: 25.05.2016
 *      Author: stefa
 */


#include "main_handlers_player_dev.h"
#include <ascii_comm_functions_player_dev.h>

//#include <stddef.h>
#include <string.h>
//#include <stdlib.h>

//#include <msp430.h>
#include <driverlib.h>

#include <config.h>
#include <event.h>
#include <misc.h>
#include <mac_layer.h>
#include <timers.h>
#include <ringbuffer.h>

//#include <states_remote_dev.h>


/*
 * Packet receive handler
 */
void lora_rx_packet_handler(void) {
	struct Packet_t* packet;
	while (!FIFO_EMPTY(mac_rx_fifo_first, mac_rx_fifo_last)) {
		// get first packet in MAC-FIFO
		packet = (struct Packet_t*) mac_rx_fifo[mac_rx_fifo_first].data;

		switch (packet->type) {
		case PLAYER_COMMAND: {
			debug_uart_sendstr("PLAYER_COMMAND rcvd\r\n");

			// could be just a status update
			break;
		}
		case CUSTOM_COMMAND: {
			// nothing to do on remote_dev side (yet)
			break;
		}
		case PLAYER_COMMAND_SUCC: {
			debug_uart_sendstr("PLAYER_COMMAND_SUCC rcvd\r\n");
			break;
		}
		case PLAYER_COMMAND_RESPONSE: {
			if (packet->command_no == 0xD5) { // TRACK No. RETURN
				debug_uart_sendstr("track number rcvd\r\n");
				// next two bytes are for EOM information
				int* tens_no = (int*)(&(packet->text));
				int* ones_no = tens_no++;
				int new_track_no = *tens_no * 10 + *ones_no;
				//update_track_no((unsigned int)new_track_no);
			}

			if (packet->command_no == 0xD9) {
				debug_uart_sendstr("track name rcvd\r\n");
				//update_track_name(&(packet->text));
			}
			break;
		}
		case CUSTOM_COMMAND_RESPONSE: {
			break;
		}
		default:
			panic("invalid packet type");
			break;
		}

		if (packet->player_status != EMPTY_STATUS) {
			//update_state(packet->player_status);
		}

		// Free this spot
		FIFO_INCR(mac_rx_fifo_first, MAC_RX_FIFO_SIZE);
	}
}

/**
 * DEBUG UART RX handler
 */
static char debug_uart_parser_buffer[DEBUG_UART_RX_BUFFER_SIZE + 1];
void debug_uart_cmdline_handler(void) {
	// read one line from RX buffer
	char c;
	int i = 0;
	rbd_t debug_rx_ringbuffer_id = debug_uart_get_rx_ringbuffer_id();

	while (ring_buffer_get(debug_rx_ringbuffer_id, &c) == 0 && c != '\r') {
		debug_uart_parser_buffer[i++] = c;
	}

	char* cmdline = debug_uart_parser_buffer;
	switch (debug_uart_parser_buffer[0]) {
	case 'L': // (L)ocal command is for this device
	case 'l':
		cmdline++;
	default: // assume local command
	{
		// parse the local command and take appropriate action
		if (strcmp(cmdline, "stop") == 0) {
			debug_uart_sendstr("<010\r");

			tascam_uart_sendstr("\n010\r");
#ifdef LORA_ENABLE
			struct Packet_t packet;
			packet.type = PLAYER_COMMAND;
			packet.command_no = 0x10;
			mac_lora_tx(BROADCAST_ADDRESS, (char*) (&packet),
					sizeof(struct Packet_t));
#endif

		} else if (strcmp(cmdline, "start") == 0
				|| strcmp(cmdline, "play") == 0) {
			debug_uart_sendstr("<012\r");

			tascam_uart_sendstr("\n012\r");
#ifdef LORA_ENABLE
			struct Packet_t packet;
			packet.type = PLAYER_COMMAND;
			packet.command_no = 0x12;
			mac_lora_tx(BROADCAST_ADDRESS, (char*) (&packet),
					sizeof(struct Packet_t));
#endif

		} else if (strcmp(cmdline, "pause") == 0
				|| strcmp(cmdline, "ready") == 0) {
			debug_uart_sendstr("<01401\r");

			tascam_uart_sendstr("\n01401\r");
#ifdef LORA_ENABLE
			struct Packet_t packet;
			packet.type = PLAYER_COMMAND;
			packet.command_no = 0x1401;
			mac_lora_tx(BROADCAST_ADDRESS, (char*) (&packet),
					sizeof(struct Packet_t));
#endif

		} else if (strcmp(cmdline, "prev") == 0) {
			debug_uart_sendstr("<01A01\r");

			tascam_uart_sendstr("\n01A01\r");
#ifdef LORA_ENABLE
			struct Packet_t packet;
			packet.type = PLAYER_COMMAND;
			packet.command_no = 0x1A01;
			mac_lora_tx(BROADCAST_ADDRESS, (char*) (&packet),
					sizeof(struct Packet_t));
#endif

		} else if (strcmp(cmdline, "next") == 0) {
			debug_uart_sendstr("<01A00\r");

			tascam_uart_sendstr("\n01A00\r");
#ifdef LORA_ENABLE
			struct Packet_t packet;
			packet.type = PLAYER_COMMAND;
			packet.command_no = 0x1A00;
			mac_lora_tx(BROADCAST_ADDRESS, (char*) (&packet),
					sizeof(struct Packet_t));
#endif

		} else {
			GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN6);
		}
		break;
	}
	case 'R': // (R)emote traffic is for the other device. Relay it via COMM_UART
	case 'r':
	{
		debug_uart_parser_buffer[0] = 'L';
		comm_uart_sendstr(debug_uart_parser_buffer);
		break;
	}
	}

	// reset parser buffer
	memset(debug_uart_parser_buffer, 0, sizeof(debug_uart_parser_buffer));
}

/**
 * TASCAM UART RX handler
 */
static char tascam_uart_parser_buffer[TASCAM_UART_RX_BUFFER_SIZE + 1];
void tascam_uart_cmdline_handler(void) {
	// read one line from RX buffer
	char c;
	int i = 0;
	rbd_t tascam_rx_ringbuffer_id = tascam_uart_get_rx_ringbuffer_id();

	while (ring_buffer_get(tascam_rx_ringbuffer_id, &c) == 0 && c != '\r') {
		tascam_uart_parser_buffer[i++] = c;
	}
	tascam_uart_parser_buffer[i] = c;

	char* cmdline = tascam_uart_parser_buffer + 1; // get rid of '\n'
	comm_uart_enqueue_char('T');
	comm_uart_sendstr(cmdline); // already has '\r' at end

	// reset parser buffer
	memset(tascam_uart_parser_buffer, 0, sizeof(tascam_uart_parser_buffer));
}
