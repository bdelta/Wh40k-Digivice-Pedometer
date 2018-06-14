//Written by Brian Do
#include "SPI.h"
void spi_init(void)
{
	//Set MOSI, SCK and !SS as outputs
	DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2);

	//Set MISO as input
	DDRB &= ~(1<<PB4);

	//Enable SPI without interrupts as master in MSB at 4Mhz
	SPCR = 0x50; //Use 0xD0 for interrupts

	//Double speed for f_osc/2
	SPSR |= 0x01;
}

void spi_transceive(uint8_t data)
{
	SPDR = data;
	//Wait for transmission end
	//Recieve data through interrupt routine
}