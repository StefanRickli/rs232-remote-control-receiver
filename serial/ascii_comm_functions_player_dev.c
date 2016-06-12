/*
 * debug_remote_dev.c
 *
 *  Created on: 22.05.2016
 *      Author: stefa
 */

#include <ascii_comm_functions_player_dev.h>
#include <uart_a0.h>
#include <uart_a1.h>
#include "config.h"
#include "event.h"



inline void debug_comm_uart_init(event_handler_t debug_uart_handler) {
	uart_a0_init(debug_uart_handler);
}

inline rbd_t debug_uart_get_rx_ringbuffer_id(void) {
	return uart_a0_get_rx_ringbuffer_id();
}

inline void debug_uart_sendstr(char str[]) {
	uart_a0_sendstr_buffered(str);
}

inline void debug_uart_newline(void) {
	uart_a0_putchar_buffered('\r');
}

inline rbd_t comm_uart_get_rx_ringbuffer_id(void) {
	return uart_a0_get_rx_ringbuffer_id();
}

inline void comm_uart_enqueue_char(char c) {
	uart_a0_enqueue_char_buffered(c);
}

inline void comm_uart_sendstr(char str[]) {
	uart_a0_sendstr_buffered(str);
}

inline void comm_uart_newline(void) {
	uart_a0_sendstr_buffered("\r");
}

inline void tascam_uart_init(event_handler_t tascam_uart_handler) {
	uart_a1_init(tascam_uart_handler);
}

inline rbd_t tascam_uart_get_rx_ringbuffer_id(void) {
	return uart_a1_get_rx_ringbuffer_id();
}

inline void tascam_uart_sendstr(char str[]) {
	uart_a1_sendstr_buffered(str);
}

