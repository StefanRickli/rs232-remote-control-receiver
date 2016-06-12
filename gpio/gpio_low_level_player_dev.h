/*
 * gpio_use.h
 *
 *  Created on: 20.05.2016
 *      Author: stefa
 */

#ifndef GPIO_GPIO_LOW_LEVEL_PLAYER_DEV_H_
#define GPIO_GPIO_LOW_LEVEL_PLAYER_DEV_H_

#include "event.h"

// -----------------------------------------------------------------------------
// GPIO init
// -----------------------------------------------------------------------------

void gpio_init_player_dev(event_handler_t, event_handler_t);


// -----------------------------------------------------------------------------
// LED functions
// -----------------------------------------------------------------------------

void led1_on(void);

void led1_off(void);

void led2_on(void);

void led2_off(void);

void led3_on(void);

void led3_off(void);


#endif /* GPIO_GPIO_LOW_LEVEL_PLAYER_DEV_H_ */
