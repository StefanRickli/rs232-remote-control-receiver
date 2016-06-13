/*
 * gpio_handler_player_dev.c
 *
 *  Created on: 22.05.2016
 *      Author: stefa
 */

#include <gpio_handler_player_dev.h>

#include <ascii_comm_functions_player_dev.h>
#include "config.h"
#include <transmit.h>

/**
 * SW1: STOP
 */
void player_dev_sw1_handler(void)
{
	debug_uart_sendstr("<SW1 STOP\n \r");
	debug_uart_sendstr("<10\n \r");

	tascam_uart_sendstr("\n010\r");

	struct Packet_t packet;
	packet.type = PLAYER_COMMAND;
	packet.command_no = 0x10;

	lora_send_pkt(&packet);
}

/**
 * SW2: START
 */
void player_dev_sw2_handler(void)
{
	debug_uart_sendstr("<SW2 START\r");
	debug_uart_sendstr("<12\r");

	tascam_uart_sendstr("\n012\r");

	struct Packet_t packet;
	packet.type = PLAYER_COMMAND;
	packet.command_no = 0x12;

	lora_send_pkt(&packet);
}
