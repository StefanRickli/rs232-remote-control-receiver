/*
 * spi.c
 *
 *  Created on: 29 févr. 2016
 *      Author: faitaoudia
 */

#include <msp430.h>
#include "spi.h"

#define BIT_SPI_CS	BIT0
#define BIT_RESET	BIT4
#define BIT_DIO_0	BIT3
#define BIT_DIO_1	BIT2
#define BIT_DIO_2	BIT3
#define BIT_DIO_3	BIT5
#define BIT_DIO_4	BIT6
#define BIT_DIO_5	BIT3

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
//                |             P3.0|-> CS
//                |                 |
//                |             P1.4|-> RESET
//                |                 |
//                |             P1.3|<-> DIO_0
//                |                 |
//                |             P1.2|<-> DIO_1
//                |                 |
//                |             P4.3|<-> DIO_2
//                |                 |
//                |             P1.5|<-> DIO_3
//                |                 |
//                |             P3.6|<-> DIO_4
//                |                 |
//                |             P3.5|<-> DIO_5

	// Configure GPIO
	P1DIR |= (BIT_RESET);
	P3DIR |= (BIT_SPI_CS);
	P1OUT &= BIT_RESET;
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
	//UCA0CTLW0 |= UCSSEL__ACLK;                // ACLK, added JASKI
	UCB0BR0 = 2;
	UCB0BR1 = 0;
	// Initializing USCI
	UCB0CTLW0 &= ~UCSWRST;

	P3OUT |= BIT_SPI_CS;

	//UCB0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

/*
 * Set a register over the SPI interface
 */
void spi_snd_data(uint8_t reg_add, uint8_t data)
{
	P3OUT &= ~BIT_SPI_CS;

	__delay_cycles(20);

	// But the MSB bit at 1 for write access
	UCB0TXBUF = 0x80u | reg_add;
	while (UCB0STATW & UCBUSY);

	UCB0TXBUF = data;
	while (UCB0STATW & UCBUSY);

	P3OUT |= BIT_SPI_CS;
}

/*
 * Read a register value over the SPI interface
 */
uint8_t spi_rcv_data(uint8_t reg_add)
{
	P3OUT &= ~BIT_SPI_CS;

	__delay_cycles(20);

	UCB0TXBUF = reg_add;
	while (UCB0STATW & UCBUSY);

	UCB0TXBUF = 0x00;
	while (UCB0STATW & UCBUSY);

	P3OUT |= BIT_SPI_CS;
	return UCB0RXBUF;
}
