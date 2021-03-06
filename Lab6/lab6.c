// ECE473 Lab6 - Alarm Clock Radio using Encoders, Bargraph, Buttons & 7-Seg - Using SPI, TWI, Timers and Interrupts
// John-Paul Molden
// 12.09.16

// Includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"

#include "twi_master.h"
#include "lm73_functions.h"
#include "uart_functions.h"
#include "si4734.h"
//Radio Driver


//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

//  PORTB0-3 are used for SPI to the encoder and bargraph boards
//  PORTB.1 = SCLK, PORTB.2 = MOSI, PORTB.3 = MISO

//  PORTE.5 = Bar graph ~OE, PORTE.6 = Encoder SR ~SCLK_enable
//  PORTE.6 = Encoder SR Shift/~Load

//

//#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0

#define SEG_OFF 0xFF
#define BUTTONS_OFF 0x00
#define BUTTONS_ON 0xFF

#define DDRA_OUTPUT 0xFF
#define DDRA_INPUT 0x00

#define Clock_mode 0
#define Alarm_mode 1
#define Clock_set_mode 2
#define Alarm_set_mode 3


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={
        0xC0,   //0
        0xF9,   //1
        0xA4,   //2
        0xB0,   //3
        0x99,   //4
        0x92,   //5
        0x82,   //6
        0xF8,   //7
        0x80,   //8
        0x98,   //9
        0xFF,   //OFF
        0xFC,   //Colon
	};


// Bargraph display values
uint8_t incdec_to_bargraph[5]={
        0xC0,   //0
        0x01,   //1
        0x83,   //2
        0xFF,   //Not used
	0x4F,	//4
        };

// Buttons to inc_dec mode lookup table
uint8_t buttons_to_incdec[4]={
        0x01,   //0
        0x02,   //1
        0x04,   //2
        0x00,
        };

// State machine look up table
uint8_t encoder_lookup[16]={0,2,1,0,1,0,0,2,2,0,0,1,0,1,2,0};
//uint8_t encoder_lookup[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


char lcd_display[32];
char lcd_string_array[32] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
				 'I', 'N', ':', ' ', ' ', 'O', 'U', 'T', ':', ' ', ' ', ' ', ' ', ' ', ' ', ' '}; //holds a string to refresh the LCD

// Temp Sensor
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp; //a place to assemble the temperature from the lm73



// Mode Variables
volatile uint8_t incdec_mode = 0;
volatile uint8_t clockmode = Clock_mode;
volatile uint8_t alarm_armed = 1;


// Current and new encoder states
volatile uint8_t encoder = 0;
volatile uint8_t old_encoder = 0;


// Time Variables
volatile uint8_t hours = 11;
volatile uint8_t mins = 55;
volatile uint8_t seconds = 40;
volatile uint8_t alarm_hours = 11;
volatile uint8_t alarm_mins = 56;
volatile uint8_t alarm_seconds = 0;
volatile uint8_t alarm_buzz = 0;


volatile uint8_t i; //dummy counter
volatile uint8_t j;
volatile uint8_t firstbyte; //Used to determine if UART byte from temp sensor is byte 1 or 2



//USART ATMega 48
char tempsensor_string[2]; //holds value of sequence number
uint8_t first_byte = TRUE;


//Radio
// Radio Variables
enum radio_band{FM, AM, SW};
volatile enum radio_band current_radio_band;
																//uint8_t freq_disp_flag = FALSE;
																//uint8_t freq_disp_counter = 0;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t eeprom_volume;

uint16_t current_fm_freq;
uint16_t LCD_current_fm_freq;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t current_volume;
extern uint8_t STC_interrupt;
uint8_t radio_onoff_toggle = TRUE;
char digits[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };



//Function Declarations
void spi_init();
uint8_t spi_rw8(uint8_t write8);
void segsum(uint8_t xmode);

void init_tcnt2();
uint16_t hours_mins_to_7segsum(uint8_t xhrs, uint8_t xmins);
void check_user_input();
void encoders();

// Disable Timers
void disable_tcnt1();
void disable_timer2();

// Init Timers
void init_tcnt0(); // initalize TIMER/COUNTER0 - Real Time Clock
void init_tcnt1(); // initalize TIMER/COUNTER1 - Alarm Tone PWM
void init_tcnt2(); // initalize TIMER/COUNTER2 - 7-Seg Brigtness PWM
void init_tcnt3(); // initalize TIMER/COUNTER3 - Audio Volume PWM

void init_DDRs();
void init_ADC();
void check_alarm(); // Checks to see if the alarm should sound on a min tick
void snooze(); // Adds 10 second to alarm upto 50 seconds
void check_ADCs(); // Checks the ADCs and changes the PWM cycle for brightness


//Radio
void radio_reset();
uint16_t current_fm_freq;


//***********************************************************************
//                            main
//**********************************************************************
int main(){


	spi_init();  //initalize SPI port, also initializes DDB
	init_tcnt0(); // initalize TIMER/COUNTER0 - Real Time Clock 8-Bit
	init_tcnt1(); // initalize TIMER/COUNTER1 - Alarm Tone PWM 16-Bit
	disable_tcnt1(); // stop the clock
	init_tcnt2(); // initalize TIMER/COUNTER2 - 7-Seg Brigtness PWM 8-bit
	init_tcnt3(); // initalize TIMER/COUNTER3 - Audio Volume PWM 16-bit
	
	init_twi();   // initialize TWI(I2C) interface - Temp Sensor
	lm73_wr_buf[0] = 0x00; //Loads the buffer with the read only temperature pointer addr
			       //The ADDR Pin is left floating for addr 0x90
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2); //start the TWI write process (twi_start_wr())
	
	//ATMega48 Functions
	//USART
	uart_init();
	
	init_DDRs(); // initalize DDRs for the display, encoders bargraph
	init_ADC();
	lcd_init(); // initialize the lcd screen

	//Radio code
	EIMSK |= 0x80; //Enable int 7 mask
	EICRB |= (1<<ISC71) | (1<<ISC70); //Set external interupt control reg B

	//radio_reset();
	radio_reset();
	_delay_ms(100);


	sei(); // enable global interrupts


	fm_pwr_up(); //powerup the radio as appropriate
	_delay_ms(100);
	current_fm_freq = 10630; //arg2, arg3: 99.9Mhz, 200khz steps
	//current_fm_freq = 10150; //arg2, arg3: 99.9Mhz, 200khz steps
	fm_tune_freq(); //tune radio to frequency in current_fm_freq
	_delay_ms(100);

	while(1){                             //main while loop
	// Send the Digits to the Display
	  //break up the disp_value to 4, BCD digits in the array: call (segsum)
		segsum(clockmode);
	  //bound a counter (0-4) to keep track of digit to display
		i = 0;
		j = 0; //Refresh the seg data less frequently
	  //send 7 segment code to LED segments
		for(;j<10;j++){
		for(;i<5;i++){
                	PORTB = i<<4 | 0<<PB7; // Select a segment
			PORTA = segment_data[i]; // Send data to the segment
			_delay_us(10); // Hold
			PORTA = 0XFF; //Seg off to reduce flicker
			_delay_us(1); // Hold
		}

	}// End while

 } //while(1)
} //main





//***********************************************************************
//                            spi_init
//**********************************************************************
void spi_init(){
  DDRB   = 0xF7; //output mode for SS, MOSI, SCLK & Pins 4-7 (7Seg & Bar Graph)

  SPCR   = (1<<SPE) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA); // Enable SPI, master mode, clk low on idle, leading edge sample

  SPSR   = (0<<SPI2X); //No double speed operation
 }//spi_init
//**********************************************************************


//***********************************************************************
//                            timer/counter0_init
//**********************************************************************
void init_tcnt0(){
// Triggers a 1 sec TOV Interrupt.
//
// Timer counter 0 initializeed to overflow every 1 second using a 32768Hz
// External clock and a 128 prescaler. This way every overflow(256) is 1sec

  ASSR  |=  (1<<AS0);                //run off external 32khz osc (TOSC)
  //enable interrupts for output compare match 0
  TIMSK |= (1<<TOIE0);  //TimerOverflow Interrupt Enable
  TCCR0 =  (1<<CS02) | (0<<CS01) | (1<<CS00);  //Normal mode, 128 prescale, OC0 Disconnected
  //OCR0  |=  0xFF;                   //compare at 256
}
//**********************************************************************




//***********************************************************************
//                            timer/counter1_init
//**********************************************************************
void init_tcnt1(){
// Annoying Frequency - Alarm
//
// setup TCNT1 in pwm mode
// set OC1A (PB5) as pwm output
// pwm frequency:  (16,000,000)/(1 * (61440 + 1)) = 260h
	//fast pwm, OC1A/B/C Disconnected (Used for 7Seg), ICR1 holds TOP
	TCCR1A |= (0<<COM1A1) | (0<<COM1A0) | (1<<WGM11);
	//use ICR1 as source for TOP, use clk/1
	TCCR1B |= (1<<WGM13) | (1<< WGM12) | (1<<CS10);
	//no forced compare
	TCCR1C = 0x00;
	//20% duty cycle, LED is a bit dimmer
	OCR1A = 0xC000; //set   at 0xC000
	ICR1  = 0xF000; //clear at 0xF000

	//enable interrupts for output compare match 0
	TIMSK |= (1<<OCIE1A);  // Timer/Counter1, Output Compare A Match Interrupt Enable
}
//**********************************************************************



//***********************************************************************
//                            timer/counter2_init
//**********************************************************************
void init_tcnt2(){
// 7-Seg Brigtness PWM

  //enable interrupts for Timer/Counter2 overflow
  TIMSK |= (1<<TOIE2) | (0<<OCIE2);  //TimerOverflow Interrupt Enable
  TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (0<<CS22) | (1<<CS21) | (1<<CS20);
	//Fast-PWM mode, Inverting PWM Mode, 64 prescale, OC2(PB7)(PWM) Connected
	//0-256 takes 1ms (16k CLKIO cycles)
  OCR2 =  0xF0;                   //compare at 128(50%)
}
//**********************************************************************


//***********************************************************************
//                            timer/counter3_init
//**********************************************************************
void init_tcnt3(){
// Volume PWM
// setup TCNT3 in pwm mode
	TCCR3A |= (1<<COM3A1) | (0<<COM3A0) | (0<<WGM31) | (1<<WGM30);
	// Fast PWM, 8-Bit mode top 0x00FF
	// Clear OC3A on compare match, Fast PWM (non-inverting)
	//use ICR1 as source for TOP, use clk/1
	TCCR3B |= (0<<WGM33) | (1<< WGM32) | (1<<CS30);
	// No prescaling
	//no forced compare
	TCCR3C = 0x00;
	//20% duty cycle, LED is a bit dimmer
	OCR3A = 0x00CD; // Initally at 50% Duty Cycle

}
//**********************************************************************



//***********************************************************************
//                            timer/counter0_init
//**********************************************************************
void init_DDRs(){
// Add HERE
	// Set the DDR for Ports
	DDRA = DDRA_OUTPUT; // 7-Seg Data out
	DDRE = (1<<PE3) | (1<<PE5) | (1<<PE6) | (1<<PE7);
	// Control for Encoders and Bargraph & Volume PWM
	DDRD = (1<<PD2); // Control for Encoders and Bargraph
	DDRC = (1<<PC0) | (1<<PC1); // Alarm PWM Tone
	incdec_mode = 0x01;
	// Read the starting encoder positions
	old_encoder = spi_rw8(0x55);

}
//**********************************************************************




//***********************************************************************
//                            init_adc
//**********************************************************************
void init_ADC(){
// ADC used to read brightness levels using a photoresistor
	//Initalize ADC and its ports
	DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input
	PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off

	ADMUX = (1<<ADLAR) | (1<<REFS0) | (1<<MUX0) | (1<<MUX1) | (1<<MUX2);
	//ADMUX = (1<<MUX0) | (1<<MUX1) | (1<<MUX2)
	//single-ended, input PORTF bit 7, right adjusted, 10 bits

	ADCSRA = (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
	//ADC enabled, don't start yet, single shot mode
}
//**********************************************************************



//***********************************************************************
//                            timer/counter1_init
//**********************************************************************
void disable_tcnt1(){
// Stop ingomkiquency - Alarm
//
	//use ICR1 as source for TOP, use clk/0 (No Clock)
	TCCR1B = (1<<WGM13) | (1<< WGM12) | (0<CS10);
	TIMSK &= ~(1<<OCIE1A); //Disable OCR1A Interrupt
}
//**********************************************************************



//***********************************************************************
//                            timer/counter2_Disable
//			      Sets timer Clk to 0
//**********************************************************************
void disable_timer2(){
// Add HERE
// Timer counter 0 initializeed to overflow every 1 second using a 32768Hz
// External clock and a 128 prescaler. This way every overflow(256) is 1sec

  //enable interrupts for output compare match 0
  TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (0<<CS22) | (0<<CS21) | (0<<CS20);
        //Fast-PWM mode, Inverting PWM Mode, 0 prescale, OC2(PB7)(PWM) Connected
        //0-256 takes 1ms (16k CLKIO cycles)
}
//**********************************************************************






//***********************************************************************
//                            spi_read_write_8bit
//**********************************************************************
uint8_t spi_rw8(uint8_t write8){
// Add HERE
	uint8_t data = 0x00;
	SPDR = write8;				// Write to the Serial Port Data Reg
	while(bit_is_clear(SPSR,SPIF)){}
		// Wait untill Status Reg interrupt flag raised
	data = SPDR;
	return(data);
}
//**********************************************************************




//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//
uint8_t chk_buttons(uint8_t button) {
        static uint16_t state[8] = {0,0,0,0,0,0,0,0};
        state[button] = (state[button] << 1) | (! bit_is_clear(PINA,button)) | 0xE000;
        if(state[button] == 0xF000) return 1;
        return 0;

//******************************************************************************
}




//***********************************************************************
//                            volume_up
//**********************************************************************
void volume_up(){
	// Todo - Change OCR3

}




//***********************************************************************
//                            volume_down
//**********************************************************************
void volume_down(){
	// Todo - Change OCR3

}




//***********************************************************************************
//                                   segment_tsum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint8_t xmode){
  //determine how many digits there are
        //output to segment_data[5]
        //unsigned int no_digits = 0;
        //if(sum >= 1)(no_digits = 1);
        //if(sum >= 10)(no_digits = 2);
        //if(sum >= 100)(no_digits = 3);
        //if(sum >= 1000)(no_digits = 4);

	switch(xmode){
		case Clock_mode:
		  	//break up decimal sum into 4 digit-segment
			//The digits (0-9) are used as the index for the seven segment representation
			segment_data[0] = dec_to_7seg[(mins/1) %10];
			segment_data[1] = dec_to_7seg[(mins/10) %10];
			if((xmode == Clock_mode)){
				segment_data[2] = dec_to_7seg[10 + (seconds % 2)] & (~(alarm_armed<<2));
				//in dec_to_7seg index 11 = OFF, index 12 = Colon
				// Blinky Colon
			}
			segment_data[3] = dec_to_7seg[((hours)/1) %10];
			segment_data[4] = dec_to_7seg[((hours)/10) %10];
			break;
		case Alarm_mode:
			segment_data[0] = dec_to_7seg[(alarm_mins/1) %10];
			segment_data[1] = dec_to_7seg[(alarm_mins/10) %10];
			segment_data[2] = dec_to_7seg[11] & (~(alarm_armed<<2));
			// in dec_to_7seg index 11 = OFF, index 12 = Colon
			// Alarm armed bit into same position as L3 on 7Seg
			segment_data[3] = dec_to_7seg[((alarm_hours)/1) %10];
			segment_data[4] = dec_to_7seg[((alarm_hours)/10) %10];
			break;
		case Clock_set_mode:
			if((seconds % 2) == 1){
				segment_data[0] = dec_to_7seg[(mins/1) %10];
				segment_data[1] = dec_to_7seg[(mins/10) %10];
				segment_data[2] = dec_to_7seg[11] & (~(alarm_armed<<2));
				//in dec_to_7seg index 11 = OFF, index 12 = Colon
				// Blinky Colon
				segment_data[3] = dec_to_7seg[((hours)/1) %10];
				segment_data[4] = dec_to_7seg[((hours)/10) %10];
				break;
			}else{
				segment_data[0] = SEG_OFF;
				segment_data[1] = SEG_OFF;
				segment_data[2] = SEG_OFF;
				segment_data[3] = SEG_OFF;
				segment_data[4] = SEG_OFF;
			break;
			}
		case Alarm_set_mode:
			if((seconds % 2) == 1){
				segment_data[0] = dec_to_7seg[(alarm_mins/1) %10];
				segment_data[1] = dec_to_7seg[(alarm_mins/10) %10];
				segment_data[2] = dec_to_7seg[11] & (~(alarm_armed<<2));
				//in dec_to_7seg index 11 = OFF, index 12 = Colon
				// Blinky Colon
				segment_data[3] = dec_to_7seg[((alarm_hours)/1) %10];
				segment_data[4] = dec_to_7seg[((alarm_hours)/10) %10];
				break;
			}else{
				segment_data[0] = SEG_OFF;
				segment_data[1] = SEG_OFF;
				segment_data[2] = SEG_OFF;
				segment_data[3] = SEG_OFF;
				segment_data[4] = SEG_OFF;
				break;
			}
		default:
			//Do nothing
			break;
	}
}//segment_sum
//***********************************************************************************





//***********************************************************************
//                            Check Buttons/Encoders
//**********************************************************************
void check_user_input(){
	//Checks the state of the buttons and encoders
	//Output
  //Read the buttons
        PORTB = PORTB | (1<<PB4) | (1<<PB5) | (1<<PB6) | (1<<PB7);
	// Select 7 - Enable Tristates on Button Board

	DDRA = 0x00; // PortA as an input from buttons
	PORTA = 0xFF; // PortA enable Pull Ups

	_delay_us(1); 				//Test Wait
        if(chk_buttons(0)){
		clockmode = (clockmode ^ 0X01) & 0x01; // Toggle bit 0
		// Toggles between clock and alarm views
	}

	if(chk_buttons(1)){
		if(radio_onoff_toggle == TRUE){
			set_property(RX_HARD_MUTE, 0x0000);
			radio_onoff_toggle = FALSE;
		}else{
			set_property(RX_HARD_MUTE, 0x0003);
			radio_onoff_toggle = TRUE;
		}



		// Add 24-12 hr functionality here
        }

	if(chk_buttons(2)){
                volume_up();
        }

	if(chk_buttons(3)){
                volume_down();
        }

	if(chk_buttons(4)){
                clockmode = Clock_set_mode;
        }

	if(chk_buttons(5)){
                clockmode = Alarm_set_mode;
		alarm_armed ^= 0x01;
		alarm_seconds = 0;
		// Toggle the arming of the alarm
        }

	if(chk_buttons(6)){
                if((alarm_buzz == 0x01)){
			snooze();
		}
        }
	// Turn off the button board PWM high
	PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6) | (0<<PB7));

	DDRA = 0xFF; //DDRA Output
	PORTA = 0xFF; //Turn Off The 7Seg

  // Send info to the bargraph (Sending info will read in encoders)
	PORTD &= ~(1<<PD2); //Storage Reg for HC595 low
	PORTE &= ~((1<<PE6) |  (1<<PE5)); //Encoder Shift Reg Clk en Low, Load Mode
	PORTC &= ~(1<<PC1);
	PORTC |= (1<<PC1);//Shift Mode

	//PORTE |= (1<<PE7);
	encoder = spi_rw8(incdec_to_bargraph[clockmode]); // Send SPI_8bit
	//spi_rw8(0xF0); 			//Test line

  // Check the encoders
	if(encoder != old_encoder){
		// Change in the encoder position
		encoders();
	}
  // Return the to original states
	PORTD |= (1<<PD2); //SS_Bar Low
	PORTE |= (1<<PE6) | (0<<PE5); //Clk enable high, Shift mode
	PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6) | (1<<PB7)); // Sel 0
  // Disable the button board tristates
}




//***********************************************************************
//                            encoder
//**********************************************************************
void encoders(){
	// The direction is determined by a state machine look up table 0=no change
	// 1=CCW, 2=CW
	//The old encoder value is place in posistion in b2 b3, 0x03 masks out other 1's

	uint8_t direction = 0;
	switch(clockmode){
		case Clock_mode:
					// Do Nothing
					//Check encoder 1
					direction = encoder_lookup[((old_encoder & 0x03)<<2) | (encoder & 0x03)];
					switch(direction){
						case 0:
							break;
						case 1:
							if(mins > 0){
								current_fm_freq = current_fm_freq - 20;
								fm_tune_freq();
								lcd_string_array[10] = digits[(current_fm_freq/10) %10];
								lcd_string_array[9] = '.';
								lcd_string_array[8] = digits[(current_fm_freq/100) %10];
								lcd_string_array[7] = digits[(current_fm_freq/1000) %10];
								lcd_string_array[6] = digits[(current_fm_freq/10000) %10];
							}else{
								current_fm_freq = 10790;
								fm_tune_freq();
								LCD_current_fm_freq = current_fm_freq/10;
								lcd_string_array[10] = digits[(current_fm_freq/10) %10];
								lcd_string_array[9] = '.';
								lcd_string_array[8] = digits[(current_fm_freq/100) %10];
								lcd_string_array[7] = digits[(current_fm_freq/1000) %10];
								lcd_string_array[6] = digits[(current_fm_freq/10000) %10];
							}
							break;
						case 2:
							if(current_fm_freq < 10790){
								current_fm_freq = current_fm_freq + 20;
								fm_tune_freq();
								lcd_string_array[10] = digits[(current_fm_freq/10) %10];
								lcd_string_array[9] = '.';
								lcd_string_array[8] = digits[(current_fm_freq/100) %10];
								lcd_string_array[7] = digits[(current_fm_freq/1000) %10];
								lcd_string_array[6] = digits[(current_fm_freq/10000) %10];
							}else{
								current_fm_freq = 8890;
								fm_tune_freq();
								lcd_string_array[10] = digits[(current_fm_freq/10) %10];
								lcd_string_array[9] = '.';
								lcd_string_array[8] = digits[(current_fm_freq/100) %10];
								lcd_string_array[7] = digits[(current_fm_freq/1000) %10];
								lcd_string_array[6] = digits[(current_fm_freq/10000) %10];
							}
							break;
						default:
							break;
					}

					//Check encoder 2
					direction = encoder_lookup[(old_encoder & 0x0C) | ((encoder & 0x0C)>>2)];
					switch(direction){
						case 0:
							break;
						case 1:
							if(hours > 0){
								hours = hours - 1;
							}else{
								hours = 23;
							}
							break;
						case 2:
							if(hours < 23){
								hours = hours + 1;
							}else{
								hours = 0;
							}
							break;
						default:
							break;
					}
					break;
			break;

		case Alarm_mode:
			// Do Nothing
			break;
		case Clock_set_mode:
			//Check encoder 1
			direction = encoder_lookup[((old_encoder & 0x03)<<2) | (encoder & 0x03)];
			switch(direction){
				case 0:
					break;
				case 1:
					if(mins > 0){
						mins = mins - 1;
					}else{
						mins = 59;
					}
					break;
				case 2:
					if(mins < 59){
						mins = mins + 1;
					}else{
						mins = 0;
					}
					break;
				default:
					break;
			}


			//Check encoder 2
			direction = encoder_lookup[(old_encoder & 0x0C) | ((encoder & 0x0C)>>2)];
			switch(direction){
				case 0:
					break;
				case 1:
					if(hours > 0){
						hours = hours - 1;
					}else{
						hours = 23;
					}
					break;
				case 2:
					if(hours < 23){
						hours = hours + 1;
					}else{
						hours = 0;
					}
					break;
				default:
					break;
			}
			break;
		case Alarm_set_mode:
			//Check encoder 1
			direction = encoder_lookup[((old_encoder & 0x03)<<2) | (encoder & 0x03)];
			switch(direction){
				case 0:
					break;
				case 1:
					if(alarm_mins > 0){
						alarm_mins = alarm_mins - 1;
					}else{
						alarm_mins = 59;
					}
					break;
				case 2:
					if(alarm_mins < 59){
						alarm_mins = alarm_mins + 1;
					}else{
						alarm_mins = 0;
					}
					break;
				default:
					break;
			}

			//Check encoder 2
			direction = encoder_lookup[(old_encoder & 0x0C) | ((encoder & 0x0C)>>2)];
			switch(direction){
				case 0:
					break;
				case 1:
					if(alarm_hours > 0){
						alarm_hours = alarm_hours - 1;
					}else{
						alarm_hours = 23;
					}
					break;
				case 2:
					if(alarm_hours < 23){
						alarm_hours = alarm_hours + 1;
					}else{
						alarm_hours = 0;
					}
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	//Replace the old encoder value
	old_encoder = encoder;
}// End encoders
//**********************************************************************




//***********************************************************************
//                            check_alarm
//**********************************************************************
void check_alarm(){
	if((alarm_armed == 0x01) && (hours == alarm_hours) && (mins == alarm_mins)){
		if((seconds == alarm_seconds)){
			init_tcnt1();
			alarm_buzz = 0x01;
			//send_lcd(0x00, 0x0C);
			lcd_string_array[0] = 'A';
			lcd_string_array[1] = 'L';
			lcd_string_array[2] = 'A';
			lcd_string_array[3] = 'R';
			lcd_string_array[4] = 'M';
		}
	}

	// Keeps alarm on while the alarm is armed
	if((alarm_armed == 0x00)){
		disable_tcnt1();
		alarm_buzz = 0x00;
		//send_lcd(0x00, 0x08); //Turn off LCD
		lcd_string_array[0] = ' ';
		lcd_string_array[1] = ' ';
		lcd_string_array[2] = ' ';
		lcd_string_array[3] = ' ';
		lcd_string_array[4] = ' ';
	}
}
//**********************************************************************


//***********************************************************************
//                            snooze_alarm
//**********************************************************************
void snooze(){
	//Turn off the alarm
	disable_tcnt1();
	alarm_buzz = 0x00;

	if(alarm_seconds < 50){
		alarm_hours = hours;
		alarm_mins = mins;
		alarm_seconds  = seconds;

		if(alarm_seconds < 50){
			alarm_seconds = alarm_seconds + 10;
			return;
		}else{
			alarm_seconds = 60-alarm_seconds;
			alarm_mins++;
		}

		if(alarm_mins > 59){
			alarm_mins = 0;
			alarm_hours++;
		}
	}
}
//**********************************************************************



//***********************************************************************
//                            check_ADCs
//**********************************************************************
void check_ADCs(){
	// Reads the ADC
	uint8_t adc_result = 0;
	ADCSRA |= (1<<ADSC); //poke ADSC and start conversion

	while(bit_is_clear(ADCSRA,ADIF)); //spin while interrupt flag not set

	ADCSRA |= (1<<ADIF); //its done, clear flag by writing a one

	adc_result = ADCH; //read the ADC output as 16 bits
	OCR2 = adc_result; //0x10; //adc_result; //adc_result;
//return(adc_data);

}
//**********************************************************************




//***********************************************************************
//                            Timer0_overflow_interrupt
//**********************************************************************
ISR(TIMER0_OVF_vect){
	//This intterupt should occur every second
	//static uint8_t seconds = 0; //Holds the seconds between interupts
	check_alarm();
	seconds++;
	if((seconds % 60) == 0){
		mins++;
		seconds = 0;
		//check_alarm();
	}
	if(((mins % 60) == 0) & ((seconds % 60) == 0)){
		hours++;
		mins = 0;
	}
	if((hours % 24) == 0){
		hours = 0;
	}
	
    //Request the temp
	twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);
	//Format the local temp data
    lm73_temp = (lm73_rd_buf[0] << 8) | (lm73_rd_buf[1]);
    lm73_temp = lm73_temp >> 7;
    itoa(lm73_temp, tempsensor_string, 10);
    //Send the local data to the LCD array
    lcd_string_array[19] = tempsensor_string[0];
    lcd_string_array[20] = tempsensor_string[1];


    //Request the ATmega48 data
    while(!(UCSR0A & (1 << UDRE0)));
    UDR0 = 0xF0;

	
}
//**********************************************************************



//***********************************************************************
//                            Timer1_OCR1A_Match
//**********************************************************************
ISR(TIMER1_COMPA_vect){
	// Port C must be used for Alarm PWM as OCR1X is used for 7Seg
	PORTC ^= 0X03; //Toggle Bit 0 & 1

}
//**********************************************************************



//***********************************************************************
//                            Timer2_overflow_interrupt
//**********************************************************************
ISR(TIMER2_OVF_vect){
	//TO DO
	//
	static uint8_t timer_tick;
	timer_tick++;
		if((timer_tick > 10)){
		timer_tick = 0;
		check_user_input();
		check_ADCs();
		
		//Refresh one LCD character
		refresh_lcd(lcd_string_array); 
	}
}



ISR(TIMER2_COMP_vect){
        //TO DO
}




//***********************************************************************
//                            USART0_recieve_interrupt
//**********************************************************************
//Get the temp from the ATMega48
ISR(USART0_RX_vect) {

	//Process the recieved ATmega48 data to the LCD
    if(first_byte == TRUE){
    	lcd_string_array[25] = uart_getc();
    	first_byte = FALSE;
    }else{
    	lcd_string_array[26] = uart_getc();
    	first_byte = TRUE;
    }
}


void radio_reset(){
	//Code given by rodger
	DDRE  |= 0x04; //Port E bit 2 is active high reset for radio
	PORTE |= 0x04; //radio reset is on at powerup (active high)

	//hardware reset of Si4734
	 PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	 DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	 PORTE |=  (1<<PE2); //hardware reset Si4734
	 _delay_us(200);     //hold for 200us, 100us by spec
	 PORTE &= ~(1<<PE2); //release reset
	 _delay_us(30);      //5us required because of my slow I2C translators I suspect
							//Si code in "low" has 30us delay...no explaination
	 DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
}



//******************************************************************************
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){STC_interrupt = TRUE;}
/***********************************************************************/







