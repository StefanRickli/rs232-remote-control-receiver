/*
 * wurx.c
 * RIC: Wake-Up on RX?
 *
 *  Created on: 9 mars 2016
 *      Author: faitaoudia
 */

#include <wurx.h>
#include <stddef.h>
#include <msp430.h>

/*
 * Event ID associated to a WuRx interrupt
 */
uint16_t _wurx_ev;

/*
 * Initialize WuRx communication.
 */
void wurx_init(event_handler_t handler)
{
	// RIC: PINASSIGN
	// The WuRx is assumed to be connected on P1.5
	P1DIR &= ~BIT5;
	P1OUT &= ~BIT5;
	P1IES &= ~BIT5;

	_wurx_ev = event_add(handler);

	wurx_enable_it();
}

// Enable/Disable WuRx interrupts
void wurx_enable_it(void)
{
	// RIC: PINASSIGN
	P1IE |= BIT5;
}

void wurx_disable_it(void)
{
	// RIC: PINASSIGN
	P1IE &= ~BIT5;
}


//RIC_ ISR now integrated in gpio_low_level
/*
#pragma vector=PORT1_VECTOR
__interrupt void port1_interrupt_handler(void)
{
	if (P1IFG & BIT5)
	{
		EVENT_SIGNAL_ISR(_wurx_ev);

		P1IFG &= ~BIT5;
	}
}
*/
