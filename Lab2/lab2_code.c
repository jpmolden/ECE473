// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0

#define SEG_OFF 0xFF
#define BUTTONS_OFF 0x0E
#define BUTTONS_ON 0x00

#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={
	0xC0,   //1
        0xAA,   //2
        0xAA,   //3
        0xAA,   //4
        0xAA,   //5
        0xAA,   //6
        0xAA,   //7
        0xAA,   //8
        0xAA,   //9
        0xAA,   //OFF 
	0xAA,	//Colon
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

  //break up decimal sum into 4 digit-segments
	segment_data[0] = (sum/1) %10;		
	segment_data[1] = (sum/10) %10;
	segment_data[2] = SEG_OFF;			
	segment_data[3] = (sum/100) %10;			
	segment_data[4] = (sum/1000) %10;			

  //blank out leading zero digits
	int i = 4;	
	for(i=4; i>no_digits; i--){
		segment_data[i] = SEG_OFF;
	}
  //now move data to right place for misplaced colon position
  // Data is placed into the correct posistion
}//segment_sum
//***********************************************************************************


//***********************************************************************************
int main()
{
//set port bits 4-7 B as outputs
while(1){
	uint8_t sum;
  //insert loop delay for debounce
	_delay_ms(2);
  //make PORTA an input port with pullups 
	DDRA = 0x00; //Enable Input on Data Direction Reg
	PORTA = 0xFF; //Enable Pull Ups on PortA

  //enable tristate buffer for pushbutton switches
	DDRB = (1<<PB4 | 1<<PB5 | 1<<PB6 | 1<<PB7);	
	PORTB = PORTB | BUTTONS_ON;
	
  //now check each button and increment the count as needed
	int i = 0;
	for( ; i<8; i++){
		if(chk_buttons(i))(sum += (2^i));
	}	
  //disable tristate buffer for pushbutton switches
	PORTB |= (0<<PB4) | (0<<PB5) | (0<<PB6) | (1<<PB7);
		
  //bound the count to 0 - 1023
	if(sum > 1023)(sum = 0);
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
  	segsum(sum);
  //bound a counter (0-4) to keep track of digit to display 
	int counter = 0;
  //make PORTA an output
	DDRA = 0xFF;
  //send 7 segment code to LED segments
	for(;counter<5;counter++){
		PORTA = 0x00;
		PORTB = 0x7F;
		_delay_ms(2);
	}
  //send PORTB the digit to display
  //update digit to display 
}//while
return 0x00;
}//main
