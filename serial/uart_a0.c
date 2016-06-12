/*
 * \file uart.c
 * \author Stefan Rickli
 * \brief UART driver
 *
 *  Created on: 2016-05-20
 *
 *  With code from
 *  	simplyembedded.com
 *  	faitaoudia
 *
 */

#include "uart_a0.h"

#include <string.h>
#include <msp430.h>
#include "driverlib.h"

#include "config.h"
#include "defines.h"
#include "ringbuffer.h"
#include "event.h"
#include "misc.h"

#define UART_A0_TX_READY		(UCA0IFG & UCTXIFG)
#define UART_A0_RX_READY		(UCA0IFG & UCRXIFG)
#define UART_A0_TX_DONE			(UCA0IFG & UCTXCPTIFG)
#define UART_A0_RESET_TX_DONE	(UCA0IFG &= ~UCTXCPTIFG)

// IDs of the events associated with UART reception
static uint16_t _uart_a0_rx_ev;
static uint16_t _uart_a0_tx_start_ev;
static uint16_t _uart_a0_tx_rdy_ev;
static uint16_t _uart_a0_tx_interrupt_pending_ev;



/* RX ring bufer */
#define A0_RX_BUFFER_SIZE	DEBUG_UART_RX_BUFFER_SIZE
#define A0_TX_BUFFER_SIZE	DEBUG_UART_TX_BUFFER_SIZE

rbd_t _uart_a0_rx_ringbuffer_id;
char _uart_a0_rx_rbmem[A0_RX_BUFFER_SIZE];
rbd_t _uart_a0_tx_ringbuffer_id;
char _uart_a0_tx_rbmem[A0_TX_BUFFER_SIZE];

void uart_a0_tx_start_handler(void);
void uart_a0_tx_rdy_handler(void);
void uart_a0_tx_interrupt_pending(void);
void uart_a0_putchar_buffered(char);
void uart_a0_sendstr_buffered(char[]);

/**
 * RIC
 * Configure eUSCI A0 in UART mode with RX ringbuffer
 */
int uart_a0_init(event_handler_t uart_a0_rx_cmdline_handler) {
	/*
	 * Short:			A0_UART,1MHz-DCO-SMCLK/9600-N-1,normal
	 * Connection:		P2.0 (TX) - Isolation Block
	 * 					P2.1 (RX) - Isolation Block
	 *
	 * Clock Source:	SMCLK
	 * Clock Rate:		DCO 1MHz
	 *
	 * Baud Rate:		9600 baud/s
	 * Parity:			none
	 * Stop Bits:		1
	 * Flow Control:	none
	 * Bit Order:		LSB first
	 * UART Mode:		normal
	 *
	 */

	int status = -1;

	// eUSCI A0 as UART device on P2.0 (TX) and P2.1 (RX)
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0,
	GPIO_SECONDARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN1,
	GPIO_SECONDARY_MODULE_FUNCTION);

	// configure parameters
	EUSCI_A_UART_initParam eUSCI_initParam = {
	EUSCI_A_UART_CLOCKSOURCE_SMCLK, // selectClockSource
			6, // clockPrescalar UCBRx
			8, // firstModReg UCBRFx
			0x20, // secondModReg UCBRSx
			EUSCI_A_UART_NO_PARITY, EUSCI_A_UART_LSB_FIRST,
			EUSCI_A_UART_ONE_STOP_BIT, EUSCI_A_UART_MODE,
			EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION };

	/* Initialize the ring buffers */
	rb_attr_t rx_rbf_attr = { sizeof(_uart_a0_rx_rbmem[0]), ARRAY_SIZE(
			_uart_a0_rx_rbmem), _uart_a0_rx_rbmem };
	rb_attr_t tx_rbf_attr = { sizeof(_uart_a0_tx_rbmem[0]), ARRAY_SIZE(
			_uart_a0_tx_rbmem), _uart_a0_tx_rbmem };

	if (ring_buffer_init(&_uart_a0_rx_ringbuffer_id, &rx_rbf_attr) == 0
			&& ring_buffer_init(&_uart_a0_tx_ringbuffer_id, &tx_rbf_attr) == 0) {
		// ring buffers have been initialized successfully, continue to
		// register the events associated with UART RX and TX
		_uart_a0_rx_ev = event_add(uart_a0_rx_cmdline_handler);
		_uart_a0_tx_start_ev = event_add(uart_a0_tx_start_handler);
		_uart_a0_tx_rdy_ev = event_add(uart_a0_tx_rdy_handler);
		_uart_a0_tx_interrupt_pending_ev = event_add(
				uart_a0_tx_interrupt_pending);

		// write parameters to module and activate it
		EUSCI_A_UART_init(EUSCI_A0_BASE, &eUSCI_initParam);
		EUSCI_A_UART_enable(EUSCI_A0_BASE);
		EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
		EUSCI_A_UART_RECEIVE_INTERRUPT); // we need this order: module enable, then interrupt enable

		status = 0;
	}
	return status;
}

inline rbd_t uart_a0_get_rx_ringbuffer_id(void) {
	return _uart_a0_rx_ringbuffer_id;
}

// ---------------------------------------------------------------------------
// high level interface functions
// ---------------------------------------------------------------------------

void uart_a0_enqueue_char_buffered(char c) {
	ring_buffer_put(_uart_a0_tx_ringbuffer_id, &c);
}

void uart_a0_putchar_buffered(char c) {
	ring_buffer_put(_uart_a0_tx_ringbuffer_id, &c);
	event_signal(_uart_a0_tx_start_ev);
}

void uart_a0_enqueue_str_buffered(char str[]) {
	while (*str != '\0') {
		ring_buffer_put(_uart_a0_tx_ringbuffer_id, str);
		str++;
	}
}

void uart_a0_sendstr_buffered(char str[]) {
	while (*str != '\0') {
		ring_buffer_put(_uart_a0_tx_ringbuffer_id, str);
		str++;
	}
	event_signal(_uart_a0_tx_start_ev);
}


// ---------------------------------------------------------------------------
// event handler
// ---------------------------------------------------------------------------

void uart_a0_tx_start_handler(void) {
	if (EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE,
	EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)) {
		char c;
		if (ring_buffer_get(_uart_a0_tx_ringbuffer_id, &c) == 0) {
			UCA0TXBUF = c;
		} else {
			panic(
					"uart_a0_tx_start_handler: invoked but tx_rbf is empty");
		}
	}
	EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
	EUSCI_A_UART_TRANSMIT_INTERRUPT);
}

void uart_a0_tx_rdy_handler(void) {
	char c;
	if (ring_buffer_get(_uart_a0_tx_ringbuffer_id, &c) == 0) {
		UCA0TXBUF = c;
		EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
		EUSCI_A_UART_TRANSMIT_INTERRUPT);
	} else { // ringbuffer empty, nothing to send anymore...
		EUSCI_A_UART_disableInterrupt(EUSCI_A0_BASE,
		EUSCI_A_UART_TRANSMIT_INTERRUPT);
		UCA0IFG |= EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG;
	}
}

void uart_a0_tx_interrupt_pending(void) {
	UCA0IFG |= EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG;
}

// ---------------------------------------------------------------------------
// ISRs
// ---------------------------------------------------------------------------

/**
 * ISR for eUSCI A0
 * Stores RX data in its buffer and echoes it.
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
	switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
	case USCI_NONE:
		break;

	case USCI_UART_UCRXIFG: {

		const char c = UCA0RXBUF;
		ring_buffer_put(_uart_a0_rx_ringbuffer_id, &c);	// store rcvd char
		//uart_a0_putchar_buffered(c);					// echo rcvd char
		if (c == '\r') {								// line break: end of cmd
			event_signal(_uart_a0_rx_ev);
		}
		LPM1_EXIT;
		break;
	}

	case USCI_UART_UCTXIFG:
		if (event_signal(_uart_a0_tx_rdy_ev) == -1) {
			event_signal(_uart_a0_tx_interrupt_pending_ev);
		}
		LPM1_EXIT;
		break;

	case USCI_UART_UCSTTIFG:
		break;
	case USCI_UART_UCTXCPTIFG:
		break;
	}
}
