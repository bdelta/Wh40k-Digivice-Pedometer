// WH40K Digivice Written by Brian Do

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>       // needed for uint8_t
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//i2c library
#include "i2cmaster/i2cmaster.h"
#include "i2cmaster/twimaster.c"

//LCD libraries
#include "5110/5110.h" 
#include "5110/5110.cpp"

//Eeprom mapping
#include "../Assets/eepmap.h"

#define DEV24LC256 0xA0

#define ADXL345 0xA6

#define ADXL345_REG_DATAX0	(0x32)    // X-axis data 0

//Step detection parameters
#define DYN_SAMPLES 50
#define PRECISION 20 //Larger means less sensitive
#define WIN_MIN 300 
#define WIN_MAX 2000 //2 seconds

LCD_5110 lcd;

//Macros
#define low_byte(addr) (addr & 0xFF) 
#define high_byte(addr) (addr >> 8)

#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

#define DISPLAY_SIZE 504
#define PAGE_SIZE 64

//Calibrate by hand, changes due to overheads
#define NUM_TIMERS 3
#define SECOND 1000

#define POMODORO 25
#define MX200 11
#define LUNCH 30


//1 if timer running, 0 if timer stopped
volatile uint8_t mode_vect = 0x00;

volatile unsigned long milis = 0;    //This is our shared volatile variable that will be used as the milis count
volatile unsigned int interval = 0;
//Debouncing by cooldown
volatile uint8_t btnPC0_cd = 0;
volatile uint8_t btnPC1_cd = 0;
volatile uint8_t btnPD0_cd = 0;
volatile uint8_t btn_vect = 0x00;

//Buzzer counters
uint8_t buzz_1 = 0;
uint8_t buzz_2 = 0;

volatile uint8_t buzz_vect = 0x00;

//Beep patterns
volatile uint8_t beep_rpt = 0x00;

//Display
//Default mode display timers
volatile uint8_t disp_vect = 0x40; //8th bit for interrupts, 7th-2nd bits for mode, 1st bit for refresh rate


//ADXL345
uint8_t max_axis = 0;

//Read from EEPROM to display on LCD
unsigned char display_buffer[DISPLAY_SIZE] = {0};

//Function Prototypes
void change_clk(int clk);
//Timers
void timer0_init(void);
void timer2_init(void); 
void end_time(int type);

//LCD Functions
void sleep_lcd(void);
void wake_lcd(void);
void update_time(uint8_t seconds, uint8_t minutes, uint8_t type);
void animate(void);

//Buzzer
void buzz_setup(void);
void buzz_on(void);
void buzz_off(void);
void btn_setup(void);
void check_end(uint8_t * timer_arr);
void buzz_routine(void);

//ADXL345
void adxl_init(void);
void readAccelDat(int * data);
int detectStep(int data[], int * min, int * max, int * shift, int counter); //Return 1 if step detected otherwise 0
int abs(int x);

//EEPROM interfacing
void write_frame_eep(const unsigned char *p, uint16_t str_addr);
void read_frame_eep(unsigned char * display_buffer, uint16_t str_addr);

//Button Polling
void btnPC0_poll(void);
void btnPC1_poll(void);
void btnPD0_poll(void);



int main()
{
	change_clk(0x01);
	//Communication protocols
	i2c_init();
	spi_init();
	sei(); //Enable global interrupts
	//Setup
	btn_setup();
	buzz_setup();
	lcd.lcd_init(&PORTB, PB2, &PORTB, PB0, &PORTD, PD5);
	read_frame_eep(display_buffer, LOAD_SCREEN);
	lcd.printPictureOnLCD_nonpgm(display_buffer);
    _delay_ms(1000);
    read_frame_eep(display_buffer, MAIN_MENU);
	lcd.printPictureOnLCD_nonpgm(display_buffer);
    timer0_init();
    timer2_init();
    volatile unsigned long pre_milis = 0;
    int excess = 0;
    unsigned long temp = 0;

    //ADXL345
    int counter = 0;
	int data[6] = {0};
	int max[3] = {0};
	int min[3] = {0};
	int shift[6] = {0}; //0 and 1, new and old etc
	unsigned int steps = 0; //Max 65,536
	adxl_init();
	
	//Timers
	uint8_t timer_arr[NUM_TIMERS*2] = {0}; //Array that keeps track of minutes and seconds
	timer_arr[0] = POMODORO;
	timer_arr[2] = MX200;
	timer_arr[4] = LUNCH;
	uint8_t *timer_p = timer_arr;

    while(1)
    {
    	//Button Polling
    	if(btn_vect&0x80)
    	{
    		btnPD0_poll();
    		btnPC1_poll();
    		btnPC0_poll();
    		btn_vect &= ~(0x80);
    	}

    	//Buzzer
    	if(buzz_vect&(0x80))
    	{
    		buzz_routine();
    		buzz_vect &= ~(0x80);
    	}
    	
    	//ADXL345
		if(milis - temp > 200)
		{
			temp = milis;
			readAccelDat(data);
		
			if(detectStep(data, min, max, shift, counter))
			{
				if(interval > WIN_MIN && interval < WIN_MAX)
				{
					steps++;
				}
				interval = 0;
			}
			counter++;
			if(counter >= DYN_SAMPLES)
			{
				counter = 0;
			}
		}

		//Update display
		if(disp_vect&(0x80))
		{
			disp_vect ^= 0x01;
			if(disp_vect&(0x01)) //50Hz refresh rate
			{
				lcd.lcd_goto_xy_exact(3, 0);
				lcd.lcd_string_format("%d", steps);
				lcd.lcd_goto_xy_exact(42, 0);
				lcd.lcd_string_format("       ", steps);
				lcd.lcd_goto_xy_exact(51, 5);
				if((mode_vect>>4) == 0)
				{
					update_time(timer_arr[1], timer_arr[0], 0);
					lcd.lcd_goto_xy_exact(43, 0);
					lcd.lcd_menu(0);
				}
				else if((mode_vect>>4) == 1)
				{
					update_time(timer_arr[3], timer_arr[2], 1);
					lcd.lcd_goto_xy_exact(54, 0);
					lcd.lcd_menu(1);
				}
				else if((mode_vect>>4) == 2)
				{
					update_time(timer_arr[5], timer_arr[4], 2);
					lcd.lcd_goto_xy_exact(65, 0);
					lcd.lcd_menu(2);
				}
				else if((mode_vect>>4) == 3)
				{
					update_time(timer_arr[5], timer_arr[4], 2);
					lcd.lcd_goto_xy_exact(75, 0);
					lcd.lcd_menu(3);
				}
			}
			
			disp_vect &= ~(0x80);
		}

		//Timer routine
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			excess = milis - pre_milis;
		}
		
    	if(excess >= SECOND)
    	{

    		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    		{
    			pre_milis = milis;
    		}
    		
	    	if(mode_vect&(0x01))
			{
				if(timer_arr[1] <= 0)
		    	{
		    		timer_arr[1] = 60;
		    		timer_arr[0]--;
		    	} 
		    	timer_arr[1]--;
			}
			if(mode_vect&(0x02))
			{
				if(timer_arr[3] <= 0)
		    	{
		    		timer_arr[3] = 60;
		    		timer_arr[2]--;
		    	} 
		    	timer_arr[3]--;
			}
			if(mode_vect&(0x04))
			{
				if(timer_arr[5] <= 0)
		    	{
		    		timer_arr[5] = 60;
		    		timer_arr[4]--;
		    	} 
		    	timer_arr[5]--;
			}
	    	check_end(timer_p);
    	}
    }
}

void timer0_init(void){
 
   	TCCR0A = (1<<WGM01);        //Timer in CTC mode
   	TCCR0B = ((1<<CS01)|(1<<CS00));    //1:64 prescaler
  	OCR0A = 124;            //Value to have a compare at every 1ms
   	TIMSK0 = (1<<OCIE0A);        //Enable timer interrupts
}

void timer2_init(void)
{
	TCCR2A = (1<<WGM21);        //Timer in CTC mode
   	TCCR2B = ((1<<CS22)|(1<<CS21)|(1<<CS20));    //1:1024 prescaler
   	OCR2A = 77;            //Value to have a compare at every 10ms
   	TIMSK2 = (1<<OCIE2A);        //Enable timer interrupts
}

void sleep_lcd(void)
{
	lcd.lcd_clear();
	lcd.lcd_send(0x24, LCD_5110::LCD_CMD);
}

void wake_lcd(void)
{
	lcd.lcd_send(0x20, LCD_5110::LCD_CMD);
}

void buzz_setup(void)
{
	DDRB |= (1 << DDB1);// PB1 as output
  	// set PWM for 50% duty cycle at 10bit
  	TCCR1A |= (1<<WGM11)|(1<<WGM10);
	TCCR1B |= (1 << CS11)|(1<<WGM12); //Prescale by 8
	OCR1A = 128;
}

void buzz_on(void)
{
	TCCR1A |= (1<<COM1A1);
}

void buzz_off(void)
{
	TCCR1A &= ~(1<<COM1A1);
}

void change_clk(int clk)
{
	CLKPR = (1 << CLKPCE);
	CLKPR = clk;
}


void btn_setup(void)
{
	set_input(DDRD,0x00); //Misc
	set_input(DDRC,0x00); //Start and stop timer button
	set_input(DDRC,0x01); //Change timer type button
	output_high(PORTD, PD0); //Set internal pullups
	output_high(PORTC,PC0); //Set internal pullups
	output_high(PORTC,PC1);	//Set internal pullups
}

void update_time(uint8_t seconds, uint8_t minutes, uint8_t type)
{

	switch (type)
	{
		case 0:
			lcd.lcd_chr('P');
			break;

		case 1:
			lcd.lcd_chr('M');
			break;

		case 2:
			lcd.lcd_chr('L');
			break;

		default:
			break;
	}
	lcd.lcd_goto_xy_exact(59,5);
	lcd.lcd_chr((minutes)/10 + 48);
	lcd.lcd_chr((minutes)%10 + 48);
	lcd.lcd_goto_xy_exact(73,5);
	if(seconds/10 == 6)
		lcd.lcd_chr(48);
	else
		lcd.lcd_chr(seconds/10 + 48);
	lcd.lcd_chr(seconds%10 + 48);
}

void check_end(uint8_t * timer_arr)
{
	if(timer_arr[0] <= 0 && timer_arr[1] <= 0 && (mode_vect & (0x01)))
	{
		mode_vect &= ~(0x01);
		timer_arr[0] = POMODORO;
		timer_arr[1] = 0;
		end_time(0);
	}
	if(timer_arr[2] <= 0 && timer_arr[3] <= 0 && (mode_vect & (0x02)))
	{
		mode_vect &= ~(0x02);
		timer_arr[2] = MX200;
		timer_arr[3] = 0;
		end_time(1);
	}
	if(timer_arr[4] <= 0 && timer_arr[5] <= 0 && (mode_vect & (0x04)))
	{
		mode_vect &= ~(0x04);
		timer_arr[4] = LUNCH;
		timer_arr[5] = 0;
		end_time(2);
	}
}

void end_time(int type)
{
	buzz_1 = 0;
	buzz_2 = 0; 
	switch (type)
	{
		case 0:
			buzz_vect |= 0x01;
			beep_rpt = 4;
	    	break;

	    case 1:
	    	buzz_vect |= 0x02;
	    	beep_rpt = 2;
			break;

		case 2:
			buzz_vect |= 0x04;
			beep_rpt = 5;
			break;

		default:
			break;
	}
	buzz_vect |= 0x40;
}

void write_frame_eep(const unsigned char *p, uint16_t str_addr)
{
	int j = 0;
	uint16_t addr = str_addr;
	unsigned char ret;
	ret = i2c_start(DEV24LC256+I2C_WRITE);
	if(ret)
	{
		i2c_stop();
	}
	else
	{
		i2c_write(high_byte(addr));
		i2c_write(low_byte(addr));
		for(int i = 0; i < 504; i++)
		{
			if(j == 64)
			{
				j = 0;
				i2c_stop();
				i2c_start_wait(DEV24LC256+I2C_WRITE);
				addr += 64;
				i2c_write(high_byte(addr));
				i2c_write(low_byte(addr));
			}
			i2c_write(pgm_read_byte(p++));
			j++;
		}
		i2c_stop();
	}
}
void read_frame_eep(unsigned char * display_buffer, uint16_t str_addr)
{
	unsigned char ret;
	uint16_t addr = str_addr;
	ret = i2c_start(DEV24LC256+I2C_WRITE);
	if(ret)
	{
		i2c_stop();
	}
	else
	{
		i2c_write(high_byte(addr));
		i2c_write(low_byte(addr));
		i2c_rep_start(DEV24LC256+I2C_READ);
		for(int i = 0; i < 503; i++)
		{
			display_buffer[i] = i2c_readAck();
		}
		display_buffer[503] = i2c_readNak();
		i2c_stop();
	}
}

void animate(void)
{
	
}

void adxl_init(void)
{
	unsigned char ret;
	ret = i2c_start(ADXL345+I2C_WRITE);
	if(ret)
	{
		i2c_stop();
	}
	else
	{
		i2c_write(0x2C); 
		i2c_write(0x0C); //Set 400Hz Data rate
		i2c_write(0x08); //Set measuring mode
		i2c_stop();
	}
}

void readAccelDat(int * data)
{

	int values[6];
	int temp[3];
	unsigned char ret;
	ret = i2c_start(ADXL345+I2C_WRITE);
	if(ret)
	{
		i2c_stop();
	}
	else
	{
		i2c_write(ADXL345_REG_DATAX0);
		i2c_rep_start(ADXL345+I2C_READ);
		for(int i = 0; i < 5; i++)
		{
			values[i] = i2c_readAck();
		}
		values[5] = i2c_readNak();
		i2c_stop();
		//Record change in acceleration
		temp[0] = (((int)values[1]) << 8) | values[0]; 
	  	temp[1] = (((int)values[3])<< 8) | values[2]; 
	  	temp[2] = (((int)values[5]) << 8) | values[4];
		data[3] = abs(data[0] - temp[0]); // x-axis
		data[4]	= abs(data[1] - temp[1]); // y-axis
		data[5]	= abs(data[2] - temp[2]); // z-axis
		data[0] = temp[0];
		data[1] = temp[1];
		data[2] = temp[2];
		
	}
}

int detectStep(int data[], int * min, int * max, int * shift, int counter)
{
	int max_delta_acc = data[3];
	for(int i = 0; i < 3; i++)
	{
		if(data[i + 3] > max_delta_acc)
		{
			max_delta_acc = data[i+3];
			max_axis = i;
		}
		if(counter != 0)
		{
			min[i] = (data[i] < min[i]) ? data[i] : min[i];
			max[i] = (data[i] > max[i]) ? data[i] : max[i];
		}
		else
		{
			min[i] = 512;
			max[i] = -512;
		}
		shift[i*2 + 1] = shift[i*2];
		if(abs(data[i] - shift[i*2]) > PRECISION)
		{
			shift[i*2] = data[i];
		}
		
	}
	int dc = min[max_axis] +(max[max_axis] - min[max_axis])/2;
	if((shift[max_axis*2 + 1] < dc) && (dc < shift[max_axis*2]))
	{
		return 1;
	}
	return 0;
}

void btnPC0_poll(void)
{
	if(!(PINC & (1<<PC0)))
	{
   		btnPC0_cd++;
   		if(btnPC0_cd >= 5 && !(btn_vect&(0x01)))
   		{
   			btn_vect |= 0x01;
   			btnPC0_cd = 0;
   			mode_vect += 0x10;
			if(mode_vect & (0x40))
			{
				mode_vect &= 0x0F;
			}
   		}
	}
	else
	{
		btn_vect &= ~(0x01);
		btnPC0_cd = 0;
	}
}
	
void btnPC1_poll(void)
{

	if(!(PINC & (1<<PC1)))
	{
   		btnPC1_cd++;
   		if(btnPC1_cd >= 5 && !(btn_vect&(0x02)))
   		{
   			btn_vect |= (0x02);
   			btnPC1_cd = 0;
   			if(!(mode_vect&(0xF0)))
   			{
   				mode_vect ^= 0x01;
   			}
   			else if(mode_vect&(0x10))
   			{
   				mode_vect ^= 0x02;
   			}
   			else if(mode_vect&(0x20))
   			{
   				mode_vect ^= 0x04;
   			}
   		}
	}
	else
	{
		btn_vect &= ~(0x02);
		btnPC1_cd = 0;
	}
}
void btnPD0_poll(void)
{
	if(!(PIND & (1<<PD0)))
	{
   		btnPD0_cd++;
   		if(btnPD0_cd >= 5 && !(btn_vect&(0x04)))
   		{
   			btn_vect |= (0x04);
   			btnPD0_cd = 0;
   			//Code here
   			end_time((mode_vect>>4));
   		}
	}
	else
	{
		btn_vect &= ~(0x04);
		btnPD0_cd = 0;
	}
}

void buzz_routine(void)
{
	//Varying beep lengths
	uint8_t on = 10;
	if(buzz_vect&(0x02))
	{
		on = 20;
	}
	if(buzz_vect&(0x04))
	{
		on = 5;
	}
	//Beep sequence
	if(beep_rpt)
	{
		//Beep on
		if(buzz_vect & (0x40))
		{
			buzz_on();
			buzz_2++;
			if(buzz_2 > on)
			{
				buzz_vect &= ~(0x40);
				beep_rpt -= 1;
				buzz_2 = 0;
			}
		}

		//Beep off
		if(!(buzz_vect & (0x40)))
		{
			buzz_off();
			buzz_1++;
			if(buzz_1 > 10)
			{
				buzz_vect |= 0x40;
				buzz_1 = 0;
			}
		}
	}
	else
	{
		buzz_off();
		buzz_vect = 0;
	}
}
 
ISR(TIMER0_COMPA_vect)
{
  	 milis++;    //Increase milis count by one millisecond
  	 interval++;
}

//Polling buttons and buzzers
//Refreshing Display
ISR(TIMER2_COMPA_vect)
{
	disp_vect |= 0x80;
	btn_vect |= 0x80;
	buzz_vect |= 0x80;
}


