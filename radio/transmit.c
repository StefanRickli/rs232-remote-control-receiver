/*
 * transmit.c
 *
 *  Created on: 25.05.2016
 *      Author: Robin Kramer
 */
#include "transmit.h"
#include <msp430.h>
#include <config.h>
#include <sx1276_driver.h>
#include <event.h>
#include <ascii_comm_functions_player_dev.h>
#include <main_handlers_player_dev.h>


void lora_init()
{
	sx1276_init(local_rx_handler, lora_rx_packet_handler, timeout_handler, crc_err_handler);
	sx1276_sleep();

		// LORA MODE - DEFAULT
		// Transmission power = 10 dBm
		// Bandwidth = 500 kHz
		// Spreading factor = 4096 chips/symbol
		// Coding rate = 4/6 (2)
		// Preamble Len = 8 + 4.25 symbols
		// Variable length
		// CRC enabled
		// Timeout: NO, we use continous mode. The radio will stay receiving until a packet is received or the mode is changed to tx
		sx1276_set_tx_config(MODEM_LORA, 10, 0, 2, 12, 2, 8, false, true);
		sx1276_set_rx_config(MODEM_LORA, 2, 12, 2, 0, 8, 0, false, 0, true);

		sx1276_rx_single_pkt(); //go afterward in listening mode

}

void lora_send_pkt(struct Packet_t *pkt)
{
	sx1276_tx_pkt((char*) pkt, sizeof(struct Packet_t), TARGET_ADDRESS);
	sx1276_rx_single_pkt();
	debug_uart_sendstr("LoRa Packet sent \n\r");

}

// Called by the SX1276 driver when a local packet is received.
// SHOULD NOT HAPPEN IN THIS APPLICATION!
void local_rx_handler(void)
{
	// Nothing here, could be improved
}
/*
 *Moved to main_handlers.c code left here as an working example
void lora_rx_handler(void)
{
	struct packet_t received;	// Buffer for data rx
	debug_uart_sendstr("New LoRa Packet \n\r");


	// Copy the packet from the driver FIFO to the buffer
	while(!FIFO_EMPTY(sx1276_rx_fifo_first, sx1276_rx_fifo_last))
	{



		memcpy(&received, sx1276_rx_fifo[sx1276_rx_fifo_first].data, sx1276_rx_fifo[sx1276_rx_fifo_first].size); //get the 1st Packet from the FiFo
		FIFO_INCR(sx1276_rx_fifo_first, SX1276_RX_FIFO_SIZE);	// Free the spot in the SX1276 driver FIFO


		#ifdef PLAYER_DEV //TODO implement the real thingy
		led_blink();
		uart_send_str(received.data);
		uart_new_line();
		#elif defined(REMOTE_DEV)
		led2_blink();
		uart_send_str(received.data);
		uart_new_line();
		lora_send_pkt(&received);//ECHO functionality
		#endif
	}

	sx1276_rx_single_pkt();
}
*/
void crc_err_handler(void)
{
	sx1276_rx_single_pkt();
	// We do nothing for now
	// Can be improved.
}

// Called by the SX1276 driver when a timeout happens
void timeout_handler(void)
{
	sx1276_rx_single_pkt();

	// We do nothing for now
	// Can be improved.
}
