/*
 * misc.c
 *
 *  Created on: 10 mars 2016
 *      Author: faitaoudia
 */

#include <msp430.h>
#include "driverlib.h"

// -----------------------------------------------------------------------
// Implementation of some useful functions
// -----------------------------------------------------------------------
/*
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 *
 * Convert a int to a string.
 */
char* itoa(int value, char* result, int base)
{

        // check that the base if valid
        if (base < 2 || base > 36) { *result = '\0'; return result; }
        char* ptr = result, *ptr1 = result, tmp_char;
        int tmp_value, init_value = value;

        do {
                tmp_value = value;
                value /= base;
                *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
        } while ( value );

        if(init_value<10) {
			*ptr++ = '0';
		}

        // Apply negative sign
        if (tmp_value < 0) *ptr++ = '-';
        *ptr-- = '\0';
        while(ptr1 < ptr) {
                tmp_char = *ptr;
                *ptr--= *ptr1;
                *ptr1++ = tmp_char;
        }

        return result;
}

#define UART_A0_TX_READY		(UCA0IFG & UCTXIFG)

/**
 * sends a last debug line, then traps the microprocessor,
 * blinking the LED
 */
void panic(char panic_reason[]) {
	// TODO send it on debug channel
	__disable_interrupt();
	while (*panic_reason != '\0') {
		while (!(UART_A0_TX_READY))
			;
		UCA0TXBUF = *panic_reason;
		panic_reason++;
	}

	for (;;) {
		GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN6);
		__delay_cycles(100000);
	}
}

