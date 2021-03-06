// ECE473 Lab3 - Encoders, Bargraph, Buttons & 7-Seg - Using SPI, Timers and Interrupts
// John-Paul Molden
// 10.26.16

// Includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

//  PORTB0-3 are used for SPI to the encoder and bargraph boards
//  PORTB.1 = SCLK, PORTB.2 = MOSI, PORTB.3 = MISO

//  PORTE.5 = Bar graph ~OE, PORTE.6 = Encoder SR ~SCLK_enable
//  PORTE.6 = Encoder SR Shift/~Load

//  PORTD.2 = Bar Graph Storage Reg CLK

//#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0

#define SEG_OFF 0xFF
#define BUTTONS_OFF 0x00
#define BUTTONS_ON 0xFF

#define DDRA_OUTPUT 0xFF
#define DDRA_INPUT 0x00


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
        0x7F,   //Colon
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



volatile uint8_t incdec_mode = 0;
volatile uint8_t button1 = 0;
volatile uint8_t button2 = 0;
volatile uint8_t encoder = 0;
volatile uint8_t old_encoder = 0;
volatile uint8_t display_count = 0x00; //holds count for 7seg display 




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
//***********************************************************************************
//                                   segment_tsum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //determine how many digits there are 
        //output to segment_data[5]
        //unsigned int no_digits = 0;
        //if(sum >= 1)(no_digits = 1);
        //if(sum >= 10)(no_digits = 2);
        //if(sum >= 100)(no_digits = 3);
        //if(sum >= 1000)(no_digits = 4);

  //break up decimal sum into 4 digit-segment   
        //The digits (0-9) are used as the index for the seven segment representation
        segment_data[0] = dec_to_7seg[(sum/1) %10];
        segment_data[1] = dec_to_7seg[(sum/10) %10];
        segment_data[2] = SEG_OFF;
        segment_data[3] = dec_to_7seg[(sum/100) %10];
        segment_data[4] = dec_to_7seg[(sum/1000) %10];

  //blank out leading zero digits
        if(sum < 1000){
                segment_data[4] = SEG_OFF;
        }
        if(sum < 100){
                 segment_data[3] = SEG_OFF;
        }
        if(sum < 10){
                 segment_data[1] = SEG_OFF;
        }
        if(sum == 0){
                 //segment_data[0] = SEG_OFF;
        }
}//segment_sum
//***********************************************************************************




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
// Add HERE

  ASSR  |=  (1<<AS0);                //run off external 32khz osc (TOSC)
  //enable interrupts for output compare match 0
  TIMSK |= (1<<OCIE0);
  TCCR0 |= (1<<WGM01) | (1<<CS00);  //CTC mode, no prescale
  OCR0  |=  0xFF;                   //compare at 256
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




//***********************************************************************
//                            encoder                               
//**********************************************************************
void encoders(uint8_t encoder_in, uint8_t old_encoder_in){
	// The direction is determined by a state machine look up table 0=no change
	// 1=CCW, 2=CW
	//The old encoder value is place in posistion in b2 b3, 0x03 masks out other 1's	

	//Check encoder 1		
	uint8_t direction = encoder_lookup[((old_encoder & 0x03)<<2) | (encoder & 0x03)];
	switch(direction){
		case 0:
			break;
		case 1:
			display_count = display_count - incdec_mode;
			break;
		case 2:
			display_count = display_count + incdec_mode;
			break;
		default:
			display_count = display_count;
	}
	
	//Check encoder 2
        direction = encoder_lookup[(old_encoder & 0x0C) | ((encoder & 0x0C)>>2)];
        switch(direction){
                case 0:
                        break;
                case 1:
                        display_count = display_count - incdec_mode;
                        break;
                case 2:
                        display_count = display_count + incdec_mode;
                        break;
                default:
                        display_count = display_count;
        }
	//Replace the old encoder value
	old_encoder = encoder;
	
}
//**********************************************************************





//***********************************************************************
//                            Interrupts
//**********************************************************************
ISR(TIMER0_COMP_vect){
  //Read the buttons
        PORTB = PORTB | (1<<PB4) | (1<<PB5) | (1<<PB6) | (1<<PB7);

	DDRA = 0x00; // PortA as an input
	PORTA = 0xFF; // PortA enable Pull Ups

        if(chk_buttons(1)){
                button2 = (button2^0x02)&0x02;
                (incdec_mode = buttons_to_incdec[((button2) | (button1))&0x03]);
        }

        if(chk_buttons(0)){
                button1 = (button1^0x01)&0x01;
		(incdec_mode = buttons_to_incdec[((button2) | (button1))&0x03]);
	}

	// The state of button2 is flipped ORd with button1 state and sets incdec mode 

	// Turn off the button board PWM high	
	PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6) | (0<<PB7));

	DDRA = 0xFF; //DDRA Output
	PORTA = 0xFF; //Turn Off The 7Seg
	
  // Send info to the bargraph (Sending info will read in encoders)
	PORTD &= ~(1<<PD2); //Storage Reg for HC595 low
	PORTE &= ~((1<<PE6) | (1<<PE7) | (1<<PE5)); //Encoder Shift Reg Clk en Low, Load Mode
	PORTE |= (1<<PE7); //Shift Mode
	encoder = spi_rw8(incdec_to_bargraph[incdec_mode]); // Send SPI_8bit
	
  // Check the encoders
	if(encoder != old_encoder){
		// Change in the encoder position
		encoders(encoder, old_encoder);
	}	
  // Return the to original states
	PORTD |= (1<<PD2); //SS_Bar Low
	PORTE |= (1<<PE6) | (1<<PE7) | (0<<PE5); //Clk enable high, Shift mode
	PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6) | (1<<PB7)); // Sel 0
  // Disable the button board tristates


}



//***********************************************************************
//                            main                               
//**********************************************************************
int main(){

uint8_t i; //dummy counter

spi_init();  //initalize SPI port
init_tcnt0(); // initalize TIMER/COUNTER0

// Set the DDR for Ports
DDRA = DDRA_OUTPUT;
DDRE = (1<<PE5) | (1<<PE6) | (1<<PE7);
DDRD = (1<<PD2);
incdec_mode = 0x01;
// Read the starting encoder positions
old_encoder = spi_rw8(0x55);
sei(); // enable global interrupts


while(1){                             //main while loop

// Decode the Display Digits
// Send the Digits to the Display
  //bound the count to 0 - 1023
        if(display_count > 1023)(display_count = display_count - 1023);
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
        segsum(display_count);
  //bound a counter (0-4) to keep track of digit to display 
        i = 0;
  //send 7 segment code to LED segments
        for(;i<5;i++){
                PORTA = segment_data[i];
                PORTB = i<<4 | 0<<PB7;
                _delay_ms(0.25);
        }

 } //while(1)
} //main
