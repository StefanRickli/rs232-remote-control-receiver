/*
 * wurx.h
 *
 *  Created on: 9 mars 2016
 *      Author: faitaoudia
 */

#ifndef WURX_WURX_H_
#define WURX_WURX_H_

#include <event.h>
#include <stddef.h>

// extern needed for gpio_use
extern uint16_t _wurx_ev;

// Type of function used as WuRx IT handler
typedef void (*wurx_it_handler_t)(void);


/*
 * Initialize WuRx communication.
 */
void wurx_init(event_handler_t handler);

// Enable/Disable WuRx interrupts
void wurx_enable_it(void);
void wurx_disable_it(void);

#endif /* WURX_WURX_H_ */
