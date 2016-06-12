/*
 * \file uart.h
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

#ifndef UART_A0_H
#define UART_A0_H

#include <stdint.h>
#include "ringbuffer.h"
#include "event.h"

/**
 * \brief Initialize the UART peripheral
 * \param[in] config - the UART configuration
 * \return 0 on success, -1 otherwise
 */
int uart_a0_init(event_handler_t);

/**
 * returns the internal id of the RX ringbuffers
 */
rbd_t uart_a0_get_rx_ringbuffer_id(void);

/**
 * Enqueue a character in the TX buffer but don't send it yet
 */
void uart_a0_enqueue_char_buffered(char);

/*
 * Send a character
 */
void uart_a0_putchar_buffered(char);

/**
 * Enqueue a string in the TX buffer but don't send it yet
 */
void uart_a0_enqueue_str_buffered(char[]);

/*
 * Send a string
 */
void uart_a0_sendstr_buffered(char[]);

#endif /* UART_A0_H */
