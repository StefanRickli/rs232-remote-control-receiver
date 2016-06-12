/*
 * spi.h
 *
 *  Created on: 29 févr. 2016
 *      Author: faitaoudia
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

/*
 * Initialize the hardware
 */
void init_spi(void);

/*
 * Set a register over the SPI interface
 */
void spi_snd_data(uint8_t reg_add, uint8_t data);

/*
 * Read a register value over the SPI interface
 */
uint8_t spi_rcv_data(uint8_t reg_add);

#endif /* SPI_H_ */
