/*
 * Driver for the SEMTECH SX1276 radio chip.
 *
 * Interface
 */

#ifndef SX1276_DRIVER_H
#define SX1276_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <event.h>
#include <config.h>

/*
 * Initialize the radio
 */
void sx1276_init(event_handler_t local_pkt_handler, event_handler_t lora_pkt_handler,
		event_handler_t timeout_handler, event_handler_t crc_err_handler);

/*
 * Receive a packet.
 * The size of the payload is assumed to be the one set using 'payload_length'
 */
void sx1276_rx_single_pkt(void);

/*
 * Transmit a packet.
 * address is the address of the recipient
 */
void sx1276_tx_pkt(char *data, uint8_t pkt_size, uint8_t address);

/*
 * Modem schemes
 */
typedef enum
{
	MODEM_NONE,
	MODEM_LORA,
	MODEM_FSK,
	MODEM_OOK
} Modem_t;

void sx1276_set_rx_config( Modem_t modem, uint32_t bandwidth,
        uint32_t datarate, uint8_t coderate,
        uint32_t bandwidthAfc, uint16_t preambleLen,
        uint16_t timeout, bool fixLen,
        uint8_t payloadLen,
        bool crcOn);

void sx1276_set_tx_config( Modem_t modem, int8_t power, uint32_t fdev,
        uint32_t bandwidth, uint32_t datarate,
        uint8_t coderate, uint16_t preambleLen,
        bool fixLen, bool crcOn);

/*
 * Enable/Disable Sync word generation and detection
 */
void sx1276_enable_sync_word(void);
void sx1276_disable_sync_word(void);

/*
 * Put the transceiver in the sleep state
 */
void sx1276_sleep(void);

/*
 * Get the currently used modulation scheme
 */
Modem_t sx1276_get_modem(void);

/*
 * Hard reset.
 *
 * After the reset, calibration and carrier frequency are set
 */
void sx1276_reset();

/*
 * RX packets FIFO
 */
#define SX1276_RX_FIFO_SIZE 5 // packets
struct sx1276_rx_packet_t
{
	char data[MAX_PACKET_SIZE];
	uint8_t size;	// Zero means no packet
	uint8_t address;	// Address of the sender
};
extern struct sx1276_rx_packet_t sx1276_rx_fifo[SX1276_RX_FIFO_SIZE];
extern volatile  uint16_t sx1276_rx_fifo_first;
extern volatile uint16_t sx1276_rx_fifo_last;

#endif
