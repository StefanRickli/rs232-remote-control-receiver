/*
 * debug_remote_dev.h
 *
 *  Created on: 22.05.2016
 *      Author: stefa
 */

#ifndef SERIAL_ASCII_COMM_FUNCTIONS_PLAYER_DEV_H_
#define SERIAL_ASCII_COMM_FUNCTIONS_PLAYER_DEV_H_

#include "config.h"
#include "event.h"
#include "ringbuffer.h"

void debug_comm_uart_init(event_handler_t);
rbd_t debug_uart_get_rx_ringbuffer_id(void);
void debug_uart_sendstr(char[]);
void debug_uart_newline(void);

rbd_t comm_uart_get_rx_ringbuffer_id(void);
void comm_uart_enqueue_char(char);
void comm_uart_sendstr(char str[]);
void comm_uart_newline(void);

void tascam_uart_init(event_handler_t);
rbd_t tascam_uart_get_rx_ringbuffer_id(void);
void tascam_uart_sendstr(char str[]);


#endif /* SERIAL_ASCII_COMM_FUNCTIONS_PLAYER_DEV_H_ */
