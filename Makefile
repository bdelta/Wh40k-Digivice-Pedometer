CC=avr-g++
LD=$(CC)
OBJCOPY=avr-objcopy
CHIP_NAME=atmega328
CFLAGS=-Os -Wall -D F_CPU=8E6
LDFLAGS=
CHIP_ID=m328p
PORT=COM1
BAUD_RATE=19200

PROJ_NAME=lcd_test

$(PROJ_NAME).hex:

$(PROJ_NAME).o: $(PROJ_NAME).c
	$(CC) -c $(CFLAGS) -mmcu=$(CHIP_NAME) $< -o $@

%.elf: %.o
	$(LD) $(LDFLAGS) -mmcu=$(CHIP_NAME) $< -o $@ 

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

upload:
	avrdude -p $(CHIP_ID) -P $(PORT) -c avrisp -b $(BAUD_RATE) -U flash:w:$(PROJ_NAME).hex

clean:
	rm -f *.hex *.elf *.o *.lst *.map
