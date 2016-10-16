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

#define F_CPU 16000000 // cpu speed in hertz 
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

uint8_t incdec_to_bargraph[4]={
        0x00,   //0
        0x01,   //1
        0x03,   //2
        0x0F,   //4
        };

uint8_t buttons_to_incdec[4]={
        0x01,   //0
        0x02,   //1
        0x04,   //2
        0x00,   //4
        };





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
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //determine how many digits there are 
        //output to segment_data[5]
        unsigned int no_digits = 0;
        if(sum >= 1)(no_digits = 1);
        if(sum >= 10)(no_digits = 2);
        if(sum >= 100)(no_digits = 3);
        if(sum >= 1000)(no_digits = 4);

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

        //for(i=1; i>no_digits; i--){
        //      segment_data[i] = SEG_OFF;
        //}
  //now move data to right place for misplaced colon position
  // Data is placed into the correct posistion
}//segment_sum
//***********************************************************************************





//***********************************************************************
//                            spi_init                               
//**********************************************************************
void spi_init(void){
  DDRB   = 0x07; //output mode for SS, MOSI, SCLK

  SPCR   = (1<<SPE) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA); // Enable SPI, master mode, clk low on idle, leading edge sample

  SPSR   = (1<<SPI2X); //choose double speed operation
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
	SPDR = write8;				// Write to the Serial Port Data Reg
	while(bit_is_clear(SPSR,SPIF)){}	
		// Wait untill Status Reg interrupt flag raised
	return(SPDR);
}
//**********************************************************************









//***********************************************************************
//                            Interrupts
//**********************************************************************
ISR(TIMER0_OVF_vect){
  //Read the buttons
	DDRA = 0x00; // PortA as an input
	PORTA = 0xFF; // PortA enable Pull Ups

  //enable the tristate buffer for the pushbuttons 
        DDRB = (1<<PB4 | 1<<PB5 | 1<<PB6 | 1<<PB7);
        PORTB = PORTB | BUTTONS_ON;

  // Set the inc/dec mode by reading button board
	if(chk_buttons(0))(incdec_mode = buttons_to_incdec[(button2 | (button1^0x01))];
	// The state of button1 is flipped ORd with button2 state and sets incdec mode 

        if(chk_buttons(1))(incdec_mode = buttons_to_incdec[((button2^0x02) | (button1))];
        // The state of button2 is flipped ORd with button1 state and sets incdec mode 

// Send info to the bargraph (Sending info will read in encoders)
	SPDR = incdec_to_bargraph[incdec_mode];
	
// Read the encoders

}



//***********************************************************************
//                            main                               
//**********************************************************************
int main(){

uint8_t display_count = 0x01; //holds count for display 
uint8_t i; //dummy counter

volatile uint8_t incdec_mode = 0;
volatile uint8_t button1 = 0;
volatile uint8_t button2 = 0;


spi_init();  //initalize SPI port
init_tcnt0(); // initalize TIMER/COUNTER0
sei(); // enable global interrupts

// Set the DDR for Ports





while(1){                             //main while loop

// Decode the Display Digits

// Send the Digits to the Display


// Send the mode to the 




    SPDR = display_count;//send display_count to the display 

    while (bit_is_clear(SPSR,SPIF)){} //wait till SPI data has been sent out

    PORTB |=  0x01;                   //send rising edge to regclk on HC595 
    PORTB &= ~0x01;                   //send falling edge to regclk on HC595

    display_count = display_count << 1;//shift display_count for next time 

    if(display_count==0x00){display_count=0x01;} //set display_count back to 1st positon
    for(i=0; i<=4; i++){_delay_ms(100);}         //0.5 sec delay
 


 } //while(1)
} //main







 
