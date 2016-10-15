// ECE473 Lab3 - Encoders, Bargraph, Buttons & 7-Seg - Using SPI, Timers and Interrupts
// John-Paul Molden
// 10.26.16



// Includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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
//  ASSR  |=  (1<<AS0);                //run off external 32khz osc (TOSC)
  //enable interrupts for output compare match 0
//  TIMSK |= (1<<OCIE0);
//  TCCR0 |=  (1<<WGM01) | (1<<CS00);  //CTC mode, no prescale
//  OCR0  |=  0x07f;                   //compare at 128
}
//**********************************************************************





//***********************************************************************
//                            Interrupts
//**********************************************************************
ISR(TIMER0_OVF_vect){
// Read the buttons


// Set the inc/dec mode


// Read the encoders

// Possible send the display count to the display (may be too slow)

}



//***********************************************************************
//                            main                               
//**********************************************************************
int main(){

uint8_t display_count = 0x01; //holds count for display 
uint8_t i; //dummy counter

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







 
