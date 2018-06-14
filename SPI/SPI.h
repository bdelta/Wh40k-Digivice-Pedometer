//Written by Brian Do

#ifndef _SPI_H
#define _SPI_H

#include <stdint.h> 
#include <avr/io.h>

extern void spi_init(void);

extern void spi_transcieve(uint8_t data);

#endif