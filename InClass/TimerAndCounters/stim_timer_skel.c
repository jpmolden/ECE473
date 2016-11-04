/*
 * StimulusReactionTimer.c
 *
 * Created: 10/14/2014 12:19:20 PM
 * Author: Justin Goins
 * Skeletonized for class use by R. Traylor 11.3.2014
 */ 

//Gives practice in using both 8 and 16 bit timer counters as well as 
//the use of a state machine to keep track of how button resources
//are used. Interrupts are not used but interrupt flags are manually
//manipulated.

//TCNT0 is used to count out a random time to wait for a users input.
//TCNT1 is used to count the time before the user hits a button

//Lines with code to be supplied by student are marked with a "*" in col 1.

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "LCDDriver.h"

enum states {SR_WELCOME, SR_RANDOM_DELAY, SR_TIMING_USER, SR_RESULTS};

int main(void)
{
  // Variables
  uint16_t numticks = 0;
	
  // set default state
  enum states state = SR_WELCOME;
 
  // PORTB is all outputs for LEDs except when used by SPI
  DDRB = 0xFF; PORTB = 0x00; 
  DDRD = 0x00; // Switches are inputs
  //PORTD = 0xFF;
  while(1) {
    switch (state) {
      case SR_WELCOME: {
        LCD_Init();
       	LCD_PutStr("REFLEX TESTER");
       	LCD_MovCursorLn2();
       	LCD_PutStr("Press any button");
       	SPCR = 0x00;    //disable SPI, PORTB used for LEDs
       	_delay_ms(100); //force minimum 100ms display time
       	while (PIND == 0xFF) {};  //spin till button pressed
	state = SR_RANDOM_DELAY;  //progress to RANDOM_DELAY state
	break;
     }

      case SR_RANDOM_DELAY: {
        LCD_Init();
        LCD_PutStr("Press any button");
        LCD_MovCursorLn2();
        LCD_PutStr("after LEDs light");
        SPCR = 0x00; // disable SPI so that PORTB can be used for LEDs

        // Set up Timer Counter 0   
        TCCR0 = 0x00;                     //normal mode, clock disabled
        TIMSK = 0x00;                     //disable _all_ TC0 interrupts	
        TIFR  = (1<<TOV0);               //manually clear overflow flag

        // start the timer with a 1024 prescaler;  16MHz/1024 = 15.625 KHz
        TCCR0 = 0x07;
        //Need to randomly wait between 2-10 seconds. Since it takes 
        //1.64 ms for the 8 bit timer to overflow, we need to loop between 
        //122-610 times.
        
        //following code is supposedly more random than other methods 
        //pick number between (0 - 487)
        uint16_t numIterations = rand() / (RAND_MAX / 488 + 1); 
        numIterations += 122; //numIterations should now be between (122 - 610)
    
        do {
    	 while(bit_is_set(TIFR, TOV0) ) {}; //spin till TC0 OVF flag set
    	 TIFR = (1<<TOV0);                                     //manually clear TC0 OVF flag
    	 //note: the counter will automatically keep counting upward again
    	 numIterations--; // decrement number of iterations
           } while (numIterations > 0);

        TCCR0 &= ~(1<<CS00 | 1<<CS01 | 1<<CS02);          //disable the TC0 timer

        state = SR_TIMING_USER;  //progress to TIMING_USER state
        break;
    }
    case SR_TIMING_USER: {
      //Set up TCNT1 to measure the user's reaction time
      TCCR1B = (0<<WGM12) | (0<<WGM13) | (0<<ICNC1);    //disable noise canceler, set WGM1{3,2} to 0, disable clock
      TCCR1A = 0x00;                  //disable all output compare pins and set WGM1{1,0} to 0

      TIMSK  &= ~(1<<TICIE1 | 1<<OCIE1A | 1<<OCIE1B | 1<<TOIE1);         //disable all TCNT1 interrupts in here
      ETIMSK = 0x00;                                    //and in here
      TIFR   = (1<<TOV1);                                    // manually clear the TC1 overflow flag
      TCNT1  = 0x0000;                                    // initialize TCNT counter to 0

      //Count the number of ticks until a button is pressed. Start the timer with a 1024 prescaler.
      //16MHz / 1024 = 15.625 KHz
      TCCR1B |= (1<<CS10) | (1<<CS11) | (1<<CS12);      // start TC1 counter
      PORTB = 0xFF; // light all LEDs so the user knows to push the button

      _delay_ms(1);	

      while ( ((TIFR & (1 << TOV1)) == 0) && (PIND == 0xFF) ) {}; // wait until button pressed or TC1 OVF set
      numticks = TCNT1;
      TCCR1B &= ~((1<<CS10) | (1<<CS11) | (1<<CS12));   // stop the TC1 counter
      // note that the count is now stored in TCNT1
      state = SR_RESULTS; // progress to RESULTS state
      break;
    }
			
    case SR_RESULTS: {
      // Now we compute the results without using floating point arithmetic. The timer runs at 15.625KHz so there 
      // are 15.625 ticks in a millisecond. We can use this information to determine the user's reaction time.
      PORTB = 0x00; //disable LEDs

      //To compute milliseconds, we multiply by 8/125. Since we are multiplying a 16 bit number,
      //be sure to perform the math using a 32 bit number.
      uint32_t numMilliseconds = ((uint32_t)numticks * 8) / 125;
	
      LCD_Init();
      if (((TIFR & (1 << TOV1)) == 0) && (numMilliseconds == 0)) {
      // overflow wasn't triggered but numMilliseconds = 0, the user held down the button
        LCD_PutStr("No cheating!!");
        LCD_MovCursorLn2();
        LCD_PutStr("Retry?");
        _delay_ms(1000);
      } else if ((TIFR & (1 << TOV1)) == 0) {
        // overflow wasn't triggered
        // display the time
        LCD_PutStr("Your time:");
        LCD_MovCursorLn2();
        LCD_PutDec16((uint16_t)numMilliseconds);
        LCD_PutStr(" ms");
      } else {
        // overflow was triggered
        // user took too long
        LCD_PutStr("Timer expired.");
        LCD_MovCursorLn2();
        LCD_PutStr("Press btn to rst");
    }
	
    SPCR = 0x00; // disable SPI so that PORTB can be used for LEDs
    _delay_ms(300); // add delay to avoid switch bouncing issues
    while (PIND == 0xFF) {}; // wait until a button is pressed
    state = SR_RANDOM_DELAY; // move back to the random delay state
    break;
    }
  }
 }
}
