/*
 * gpio_handler_player_dev.c
 *
 *  Created on: 22.05.2016
 *      Author: stefa
 */

#include <gpio_handler_player_dev.h>

#include <ascii_comm_functions_player_dev.h>
#include "config.h"
#include "mac_layer.h"

/**
 * SW1: STOP
 */
void player_dev_sw1_handler(void)
{
	debug_uart_sendstr("<SW1 STOP\r");
	debug_uart_sendstr("<10\r");

	tascam_uart_sendstr("\n010\r");

	struct Packet_t packet;
	packet.type = PLAYER_COMMAND;
	packet.command_no = 0x10;

	mac_lora_tx(BROADCAST_ADDRESS, (char*)(&packet), sizeof(struct Packet_t));
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

	mac_lora_tx(BROADCAST_ADDRESS, (char*)(&packet), sizeof(struct Packet_t));
}
