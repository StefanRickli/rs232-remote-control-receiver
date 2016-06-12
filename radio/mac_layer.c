/*
 * mac_layer.c
 *
 *  Created on: 10 mars 2016
 *      Author: faitaoudia
 */

#include <mac_layer.h>
#include <sx1276_driver.h>
#include <string.h>
#include <misc.h>
#include <spi.h>
#include <wurx.h>
#include <timers.h>

// -----------------------------------------------------------------------------------
// Internals
// -----------------------------------------------------------------------------------
// ---------------------------------- Internal variables
// IDs of the events associated with a lora packet reception and local packet reception
static uint16_t _rx_pkt_ev;
// Are we in LoRa continous RX mode
static bool _lora_ctn_rx = false;

// ---------------------------------- Internal functions
static void _lora_rx_handler(void)
{
	// Copy the packet from the driver buffer to this module buffer
	while (!FIFO_EMPTY(sx1276_rx_fifo_first, sx1276_rx_fifo_last))
	{
		memcpy(mac_rx_fifo[mac_rx_fifo_last].data, sx1276_rx_fifo[sx1276_rx_fifo_first].data, sx1276_rx_fifo[sx1276_rx_fifo_first].size);
		mac_rx_fifo[mac_rx_fifo_last].address = sx1276_rx_fifo[sx1276_rx_fifo_first].address;
		mac_rx_fifo[mac_rx_fifo_last].size = sx1276_rx_fifo[sx1276_rx_fifo_first].size;
		FIFO_INCR(mac_rx_fifo_last, MAC_RX_FIFO_SIZE);
		FIFO_INCR(sx1276_rx_fifo_first, SX1276_RX_FIFO_SIZE);
	}

	// Signal the receiving pkt event
	event_signal(_rx_pkt_ev);

	if (!_lora_ctn_rx)
	{
		sx1276_sleep();
	}
	else
	{
		sx1276_rx_single_pkt();
	}
}

static void _timeout_handler(void)
{
	// If not in LoRa continuous listening mode, go to sleep.
	if (!_lora_ctn_rx)
	{
		sx1276_sleep();
	}
}

static void _crc_err_handler(void)
{
	if (!_lora_ctn_rx)
	{
		sx1276_sleep();
	}
	else
	{
		sx1276_rx_single_pkt();
	}
}

static void _lora_mode(void)
{
	if (sx1276_get_modem() == MODEM_LORA)
	{
		return;
	}

	// First go to sleep mode to change the mode
	sx1276_sleep();

    // LORA MODE - DEFAULT
    // Transmission power = 10 dBm
    // Bandwidth = 500 kHz
    // Spreading factor = 4096 chips/symbol
    // Coding rate = 4/6 (2)
    // Preamble Len = 8 + 4.25 symbols
    // Variable length
    // CRC enabled
    // Timeout: NO, we use continous mode. The radio will stay receiving until a packet is received.
	sx1276_set_tx_config(MODEM_LORA, 10, 0, 2, 12, 2, 8, false, true);
	sx1276_set_rx_config(MODEM_LORA, 2, 12, 2, 0, 8, 0, false, 0, true);
}

static void _fsk_mode(void)
{
	if (sx1276_get_modem() == MODEM_FSK)
	{
		return;
	}

	// First go to sleep mode to change the mode
	sx1276_sleep();

    // FSK mode
    // Transmission power = 0 dBm
    // Frequency deviation = 25 kHz
    // RX Bandwidth = 50 kHz
    // Bitrate = 20 kbps
    // AFC Bandwisth = 83333 Hz
    // Preamble Length = 5 bytes
    // Variable length
    // CRC enabled
    // Timeout: 1s
    sx1276_set_tx_config(MODEM_FSK, 0, 5000, 0, 20000, 0, 5, false, true);
    sx1276_set_rx_config(MODEM_FSK, 62500, 20000, 0, 83333, 5, 4096u, false, 0u, true);
}

static void _ook_mode(void)
{
	if (sx1276_get_modem() == MODEM_OOK)
	{
		return;
	}

	sx1276_sleep();

    // OOK MODE
    // Transmission power = 10 dBm
    // RX Bandwidth = 50 kHz
    // Bitrate = 1 kbps
    // AFC Bandwisth = 83.3 kHz
    // No preamble
    // No sync word
    // Length fixed to 1 byte
    // CRC disabled
    // Timeout: ~5ms
    sx1276_set_tx_config(MODEM_OOK, 10, 0, 0, 1000, 0, 0, true, false);
    sx1276_set_rx_config(MODEM_OOK, 50000, 1000, 0, 83333, 0, 21u, true, 1, false);
    sx1276_disable_sync_word();
}

static void _local_rx_handler(void)
{
	// Copy the packet from the driver buffer to this module buffer
	while (!FIFO_EMPTY(sx1276_rx_fifo_first, sx1276_rx_fifo_last))
	{
		memcpy(mac_rx_fifo[mac_rx_fifo_last].data, sx1276_rx_fifo[sx1276_rx_fifo_first].data, sx1276_rx_fifo[sx1276_rx_fifo_first].size);
		mac_rx_fifo[mac_rx_fifo_last].address = sx1276_rx_fifo[sx1276_rx_fifo_first].address;
		mac_rx_fifo[mac_rx_fifo_last].size = sx1276_rx_fifo[sx1276_rx_fifo_first].size;
		FIFO_INCR(mac_rx_fifo_last, MAC_RX_FIFO_SIZE);
		FIFO_INCR(sx1276_rx_fifo_first, SX1276_RX_FIFO_SIZE);
	}

	// Signal the receiving pkt event
	event_signal(_rx_pkt_ev);

	if (_lora_ctn_rx)
	{
		_lora_mode();
		sx1276_rx_single_pkt();
	}
	else
	{
		sx1276_sleep();
	}
}

void _wurx_handler(void)
{
	// Listening for the COMMAND arriving locally using FSK modulation
	_fsk_mode();
	// Delay to give time to the transmitting node to switch from OOK to FSK and to start transmitting
	//__delay_cycles(5000u);
	sx1276_rx_single_pkt();
}

// -----------------------------------------------------------------------------------
// Implementation of the interface
// -----------------------------------------------------------------------------------
// ------------------------------------------------- Public variables
// The RX fifo
struct mac_packet_t mac_rx_fifo[MAC_RX_FIFO_SIZE];
volatile uint16_t mac_rx_fifo_first = 0u;
volatile uint16_t mac_rx_fifo_last = 0u;

// ------------------------------------------------- Public function
/*
 * Initialize the MAC layer
 */
void mac_init(event_handler_t rx_pkt_handler, bool use_wurx, bool lora_ctn_rx)
{
	// Initializing the transceiver
    init_spi();
    sx1276_init(_local_rx_handler, _lora_rx_handler, _timeout_handler, _crc_err_handler);

    if (use_wurx)
    {
    	wurx_init(_wurx_handler);
    }

    _rx_pkt_ev = event_add(rx_pkt_handler);

    if (lora_ctn_rx)
    {
    	_lora_ctn_rx = true;
    	_lora_mode();
    	// Go in continous RX
    	// As we put no timeout, the radio will stay in RX until a packet is received
    	sx1276_rx_single_pkt();

    }
    else
    {
    	// If no continuous LoRa listening, put the radio in sleep
    	sx1276_sleep();
    }
}

void mac_lora_tx(uint8_t dst_addr, char* data, uint8_t data_size)
{
	// JUST FOR TESTING DEACTIVATED!
	return;

	_lora_mode();

	sx1276_tx_pkt(data, data_size, dst_addr);
	// At this point the radio is in standby-mode

	if (_lora_ctn_rx)
	{
		// If continuous LoRa listening enabled, put the radio is RX LoRa mode
		sx1276_rx_single_pkt();
	}
	else
	{
		// If no continuous LoRa listening enabled, put the radio in SLEEP
		sx1276_sleep();
	}
}

/*
 * Send a packet locally. Uses WuRx/OOK/FSK
 */
void mac_local_tx(uint8_t dst_addr, char *data, uint8_t data_size)
{
	_ook_mode();
	uint8_t wub[] = {WUB_PREAMBLE, dst_addr};
	sx1276_tx_pkt((char*)wub, 2u, 0u);	// Don't care about the third parameter when sending using OOK

	// Then we send the data
	_fsk_mode();
	sx1276_tx_pkt(data, data_size, dst_addr);

	if (_lora_ctn_rx)
	{
		// If continuous LoRa listening enabled, put the radio is RX LoRa mode
		_lora_mode();
		sx1276_rx_single_pkt();
	}
	else
	{
		// If no continuous LoRa listening enabled, put the radio in SLEEP
		sx1276_sleep();
	}
}
