# Wh40k-Digivice

A digivice clone utilizing the atmega328p programmed in avr-c. Also includes some handy productivity timers such as pomodoro and a lunch timer.

Processor: atmega328p running at 8 Mhz External Crystal Oscillator


Screen: Nokia5110 with PCD8544


Accelerometer/Pedometer: ADXL345


Power Supply: MAX756 Boost Converter w/ 2 x AAA batteries


Memory: 24LC256EEPROM

Require avrdude with avr-g++ and avr-gcc

Edit required COM port in makefile

Designed to run at 8Mhz, can use internal oscillator but disable prescale using fuse bytes and comment out change_clk(0x01).

To compile use makefile "make" and to upload "make upload" in command line.
