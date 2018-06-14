/*
 * Nokia 3110 / 5110 unbuffered driver
 * (C)2013 Radu Motisan
 * www.pocketmagic.net
 * Based on a work by Tony Myatt - 2007
 */
//Modified by Brian Do

#pragma once

#include "../timeout.h"
#include "../SPI/SPI.h"
#include "../SPI/SPI.c"

// define  Lcd screen size 
#define LCD_WIDTH 84
#define LCD_HEIGHT 48
#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8
#define MENU_WIDTH 9

class LCD_5110 {
	/* Command type sent to the lcd */

public:
	typedef enum { LCD_CMD  = 0, LCD_DATA = 1 } LcdCmdData;
	
	uint8_t  m_dq_SCE, m_dq_RST, m_dq_DC, m_dq_DATA;
	volatile uint8_t  *m_port_SCE, *m_port_RST, *m_port_DC;
	
	int lcdCacheIdx;

	volatile uint8_t* Port2DDR(volatile uint8_t *port) {
		return port - 1;
	}
	
	void lcd_init(
		volatile uint8_t  *port_SCE, uint8_t  dq_SCE,
		volatile uint8_t  *port_RST, uint8_t  dq_RST,
		volatile uint8_t  *port_DC, uint8_t  dq_DC
	);
	void lcd_contrast(unsigned char contrast);
	void lcd_clear(void);
	void lcd_clear_area(unsigned char line, unsigned char startX, unsigned char endX);
	void lcd_clear_line(unsigned char line);
	void lcd_goto_xy(unsigned char x, unsigned char y);
	void lcd_goto_xy_exact(unsigned char x, unsigned char y);
	void lcd_chr(char chr);
	void lcd_menu(unsigned char menu);
	void lcd_str(char* str);
	void lcd_string_format(char *szFormat, ...);
	void lcd_send(unsigned char data, LcdCmdData cd);
	void lcd_base_addr(unsigned int addr) ;
	void lcd_col(char chr);
	void lcd_pixelBack(void);
	void printPictureOnLCD ( const unsigned char *data);
	void printPictureOnLCD_nonpgm (unsigned char *data);
	void drawPixel(unsigned char  x, unsigned char  y, int color);
};



