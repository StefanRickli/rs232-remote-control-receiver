/*
 * spi.c
 *
 *  Created on: 29 févr. 2016
 *      Author: faitaoudia
 */

#include <msp430.h>
#include "spi.h"

// RIC: PINASSIGN
#define BIT_SPI_CS	BIT4
#define BIT_RESET	BIT3

/* RIC: unused
#define BIT_DIO_0	BIT5
#define BIT_DIO_1	BIT6
#define BIT_DIO_2	BIT2
#define BIT_DIO_3	BIT4
#define BIT_DIO_4	BIT3
#define BIT_DIO_5	BIT6
*/

void init_spi(void) {
//                   MSP430FR5969
//                 -----------------
//                |                 |
//                |             P1.6|-> Data Out (UCB0SIMO)
//                |                 |
//                |             P1.7|<- Data In (UCB0SOMI)
//                |                 |
//                |             P2.2|-> Serial Clock Out (UCB0CLK)
//                |                 |
//                |             P1.4|-> CS
//                |                 |
//                |             P1.3|-> RESET
//                |                 |
//                |             P2.5|<-> DIO_0
//                |                 |
//                |             P2.6|<-> DIO_1
//                |                 |
//                |             P4.2|<-> DIO_2
//                |                 |
//                |             P2.4|<-> DIO_3
//                |                 |
//                |             P4.3|<-> DIO_4
//                |                 |
//                |             P3.6|<-> DIO_5

	// RIC: PINASSIGN
	// Configure GPIO
	P1DIR |= (BIT_SPI_CS | BIT_RESET);
	P1OUT &= ~BIT_RESET;	// RIC: disable sx1276 from the beginning?, BACKUP P1OUT &= BIT_RESET
	P2SEL0 &= ~BIT2;
	P1SEL1 &= ~(BIT6 | BIT7);
	P2SEL1 |= BIT2;
	P1SEL1 |= BIT6 | BIT7;

	// Configure SPI
	// Reset mode
	UCB0CTLW0 = UCSWRST;

	// 3 pin
	// SPI master
	// High clock polarity
	// MSB first
	UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB;
	// Sourcing SPI clock from SMCLK (should be at 1 MHz)
	UCB0CTLW0 |= UCSSEL__SMCLK;
	UCB0BR0 = 2;
	UCB0BR1 = 0;
	// Initializing USCI
	UCB0CTLW0 &= ~UCSWRST;

	P1OUT |= BIT_SPI_CS;

	//UCB0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

/*
 * Set a register over the SPI interface
 */
void spi_snd_data(uint8_t reg_add, uint8_t data)
{
	P1OUT &= ~BIT_SPI_CS;

	__delay_cycles(20);

	// But the MSB bit at 1 for write access
	UCB0TXBUF = 0x80u | reg_add;
	while (UCB0STATW & UCBUSY);

	UCB0TXBUF = data;
	while (UCB0STATW & UCBUSY);

	P1OUT |= BIT_SPI_CS;
}

/*
 * Read a register value over the SPI interface
 */
uint8_t spi_rcv_data(uint8_t reg_add)
{
	P1OUT &= ~BIT_SPI_CS;

	__delay_cycles(20);

	UCB0TXBUF = reg_add;
	while (UCB0STATW & UCBUSY);

	UCB0TXBUF = 0x00;
	while (UCB0STATW & UCBUSY);

	P1OUT |= BIT_SPI_CS;
	return UCB0RXBUF;
}
