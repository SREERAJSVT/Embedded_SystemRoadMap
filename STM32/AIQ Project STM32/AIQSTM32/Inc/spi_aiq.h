/*
 * spi_aiq.h
 *
 *  Created on: 24-Dec-2025
 *      Author: sreer
 */

#ifndef SPI_AIQ_H_
#define SPI_AIQ_H_
#include <stdint.h>
void spi1_cs_low(void);
void spi1_cs_high(void);
uint8_t spi1_txrx(uint8_t v);


#endif /* SPI_AIQ_H_ */
