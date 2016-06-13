/*
 * transmit.h
 *
 *  Created on: 25.05.2016
 *      Author: Robin Kramer
 */

#ifndef TRANSMIT_TRANSMIT_H_
#define TRANSMIT_TRANSMIT_H_
#include <config.h>
enum Packet_type_t
{
	SENSING_DATA,			// Deprecated, Sensing data
	PLAYER_COMMAND,			// Command is in Tascam formatting
	CUSTOM_COMMAND,			// Command is in custom formatting (one int following)
	PLAYER_COMMAND_SUCC,	// Command has been executed successfully
	PLAYER_COMMAND_RESPONSE,// Response with one int for the command and data (char[])
	CUSTOM_COMMAND_RESPONSE	// Response with one int for the command and data (char[])
};

struct Packet_t
{
	enum Packet_type_t type;	// Type of the packet
	int command_no;				// Information: specific to each type
	enum Player_Mecha_Status player_status;
	char text[MAX_TRACK_NAME_LENGTH];
};


void lora_init();

void local_rx_handler(void); //required by the sxdriver


void lora_send_pkt(struct Packet_t *pkt);

void lora_rx_handler(void);

void crc_err_handler(void);

void timeout_handler(void);

#endif /* TRANSMIT_TRANSMIT_H_ */
