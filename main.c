#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include <msp430.h>
#include <driverlib.h>

#include <config.h>
#include <event.h>
#include <mac_layer.h>
#include <ringbuffer.h>
#include <timers.h>

#include <main_handlers_player_dev.h>
#include <gpio_low_level_player_dev.h>
#include <gpio_handler_player_dev.h>
#include <ascii_comm_functions_player_dev.h>

/*
 * Initialize the clock
 *
 * DCO set at 1 MHz
 *
 * MCLK and SMCLK set at 8 MHz and sources from DCO without div /1, and thus run at 1 MHz
 * ACLK is sources from LFXT (32 kHz) with div / 1
 */
static void init_clock(void) {
	// Set the DCO at 1 MHz
	CSCTL0 = CSKEY;
	CSCTL1 = DCOFSEL_0;
	CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__LFXTCLK;
	CSCTL3 = DIVM__1 | DIVS__1 | DIVA__1;
}

#define MAX_CMD_ACK_BACKOFF 4095	// 1s - MAX random backoff for cmd ack



// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5;

	init_clock();
	event_init();
	timers_init();
	srand(NODE_ADDRESS);

	debug_comm_uart_init(debug_uart_cmdline_handler);
	tascam_uart_init(tascam_uart_cmdline_handler);

	gpio_init_player_dev(player_dev_sw1_handler, player_dev_sw2_handler);

	mac_init(lora_rx_packet_handler, false, true);

	// Enabling global interrupts
	_EINT();

	// Starting the event loop
	event_loop();
}
