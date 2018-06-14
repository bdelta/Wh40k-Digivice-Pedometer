#include "SPI.h"

//SPI with interrupts
void spi_init(void)
{
	//Set MOSI, SCK and !SS as outputs
	DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2);

	//Set MISO as input
	DDRB &= ~(1<<PB4);

	//Enable SPI with interrupts as master in MSB at 4Mhz
	SPCR0 = 0xD0;

	//Double speed for f_osc/2
	SPSR0 |= 0x01;
}

void spi_transceive(uint8_t data)
{
	SPDR0 = data;
	//Recieve data through interrupt routine
}