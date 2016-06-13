/*
 * gpio_use.c
 *
 *  Created on: 20.05.2016
 *      Author: Stefan Rickli
 */

#include "gpio_low_level_player_dev.h"
#include <msp430.h>
#include "driverlib.h" //TODO get rid of the driverlib to remove unnecessary overhead when compiled. Unless it should be portable to some other msp430 RK
#include "event.h"



// -----------------------------------------------------------------------------
// GPIO init
// -----------------------------------------------------------------------------
// RIC: PINASSIGN
#define SW1_PORT	GPIO_PORT_P4
#define SW1_PIN		GPIO_PIN5
#define SW2_PORT	GPIO_PORT_P1
#define SW2_PIN		GPIO_PIN1

#define LED1_PORT	GPIO_PORT_P4
#define LED1_PIN	GPIO_PIN6
#define LED2_PORT	GPIO_PORT_P1
#define LED2_PIN	GPIO_PIN0
#define LED3_PORT	GPIO_PORT_P1
#define LED3_PIN	GPIO_PIN2

// Event IDs associated with the buttons
uint16_t _sw1_event;
uint16_t _sw2_event;

// -----------------------------------------------------------------------------
// GPIO init
// -----------------------------------------------------------------------------

void gpio_init_player_dev(event_handler_t sw1_handler, event_handler_t sw2_handler) { //, event_handler_t sw3_handler, event_handler_t sw4_handler, event_handler_t sw5_handler, event_handler_t sw6_handler, event_handler_t sw7_handler) {
	// Switch init
	GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
	GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
	GPIO_selectInterruptEdge(SW1_PORT, SW1_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
	GPIO_selectInterruptEdge(SW2_PORT, SW2_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
	GPIO_clearInterrupt(SW1_PORT, SW1_PIN);
	GPIO_clearInterrupt(SW2_PORT, SW2_PIN);
	GPIO_enableInterrupt(SW1_PORT, SW1_PIN);
	GPIO_enableInterrupt(SW2_PORT, SW2_PIN);

	_sw1_event = event_add(sw1_handler);
	_sw2_event = event_add(sw2_handler);

	// LED init
	GPIO_setAsOutputPin(LED1_PORT, LED1_PIN);
	GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
	GPIO_setAsOutputPin(LED3_PORT, LED3_PIN);

	GPIO_setOutputLowOnPin(LED1_PORT, LED1_PIN);
	GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
	GPIO_setOutputLowOnPin(LED3_PORT, LED3_PIN);
}

// -----------------------------------------------------------------------------
// LED functions
// -----------------------------------------------------------------------------

/**
 * LED1 on: STOP
 */
inline void led1_on(void) {
	GPIO_setOutputHighOnPin(LED1_PORT, LED1_PIN);
}

/**
 * LED1 off: STOP
 */
inline void led1_off(void) {
	GPIO_setOutputLowOnPin(LED1_PORT, LED1_PIN);
}

/**
 * LED1 on: START
 */
inline void led2_on(void) {
	GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);
}

/**
 * LED2 off: START
 */
inline void led2_off(void) {
	GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
}

/**
 * LED3 on: READY
 */
inline void led3_on(void) {
	GPIO_setOutputHighOnPin(LED3_PORT, LED3_PIN);
}

/**
 * LED3 off: READY
 */
inline void led3_off(void) {
	GPIO_setOutputLowOnPin(LED3_PORT, LED3_PIN);
}

// -----------------------------------------------------------------------------
// GPIO ISRs

// -----------------------------------------------------------------------------

//P1 moved to sx1276_driver.c in order to avoid many extern declared variables RK

#pragma vector=PORT3_VECTOR//in case P3 will be used anytime soon. RK
__interrupt void isr_gpio_p3(void)
{

	switch (__even_in_range(P3IV, P3IV_P3IFG7)) {
	case P3IV_NONE:
		break;
	case P3IV_P3IFG0:
		break;
	case P3IV_P3IFG1:
		break;
	case P3IV_P3IFG2:
		break;
	case P3IV_P3IFG3:
		break;
	case P3IV_P3IFG4:
		break;
	case P3IV_P3IFG5:
		break;
	case P3IV_P3IFG6:
		break;
	case P3IV_P3IFG7:
		break;
	}

}


#pragma vector=PORT4_VECTOR
__interrupt void isr_gpio_p4(void)
{
	led1_on();
	switch (__even_in_range(P4IV, P4IV_P4IFG7)) {
	case P4IV_NONE:
		break;
	case P4IV_P4IFG0:
		break;
	case P4IV_P4IFG1:
		break;
	case P4IV_P4IFG2:
		break;
	case P4IV_P4IFG3:
		break;
	case P4IV_P4IFG4:
		break;
	case P4IV_P4IFG5:
		__delay_cycles(200);
		while(!(P4IN & BIT5));
		__delay_cycles(200);
		P4IFG &= ~BIT5;
		EVENT_SIGNAL_ISR(_sw1_event);
		break;
	case P4IV_P4IFG6:
		break;
	case P4IV_P4IFG7:
		break;
	}
	__delay_cycles(20000);
	led1_off();
}
