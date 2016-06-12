/*
 * mac_layer.h
 *
 *  Created on: 10 mars 2016
 *      Author: faitaoudia
 */

#ifndef MAC_MAC_LAYER_H_
#define MAC_MAC_LAYER_H_

#include <stdint.h>
#include <event.h>
#include <config.h>
#include <stdbool.h>

// Before each LoRa TX, a random backoff takes place
#define MAX_LORA_TX_BACKOFF	2047	// 500ms


/*
 * Initialize the MAC layer
 */
void mac_init(event_handler_t rx_pkt_handler, bool use_wurx, bool lora_ctn_rx);

/*
 * Send a packet using LoRa
 */
void mac_lora_tx(uint8_t dst_addr, char* data, uint8_t data_size);

/*
 * Send a packet locally. Uses WuRx/OOK/FSK
 */
void mac_local_tx(uint8_t dst_addr, char *data, uint8_t data_size);

/*
 * RX FIFO made from a circular buffer
 */
#define MAC_RX_FIFO_SIZE	5	// packets

struct mac_packet_t
{
	uint8_t address;	// address of the sender
	uint8_t size;	// 0 means nothing here (free spot)
	char data[MAX_PACKET_SIZE];
};
extern struct mac_packet_t mac_rx_fifo[MAC_RX_FIFO_SIZE];
extern volatile uint16_t mac_rx_fifo_first;
extern volatile uint16_t mac_rx_fifo_last;


#endif /* MAC_MAC_LAYER_H_ */
