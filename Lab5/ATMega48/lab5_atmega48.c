// ECE473 Lab4 - Alarm Clock using Encoders, Bargraph, Buttons & 7-Seg - Using SPI, Timers and Interrupts
// ATMega48 USART
// John-Paul Molden
// 10.26.16


// Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "mega48_uart_functions.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"

//Defines
#define TRUE 1
#define FALSE 0


//External Variable
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];

//Variable Defns
uint8_t rx_m128_command;
uint16_t lm73_temp; //a place to assemble the temperature from the lm73
uint8_t new_data_needed;
char temp_string_array[3] = {' ', ' ', ' '};




int main() {

    init_twi();
    uart_init();
    sei();
  
    //Initial Temparature Data
    lm73_wr_buf[0] = 0x00;
    twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2);
    lm73_temp = (lm73_rd_buf[0] << 8) | (lm73_rd_buf[1]);
    lm73_temp = lm73_temp >> 7;
    //Populate the local temparature data
    itoa(lm73_temp, temp_string_array, 10);  
  
    
    while(1) {
      //Get new data once the USART Intterupt Sends
      if(new_data_needed == TRUE){
          //Read the temp over TWI
          twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);
          _delay_ms(2);
          lm73_temp = (lm73_rd_buf[0] << 8) | (lm73_rd_buf[1]);
          lm73_temp = lm73_temp >> 7;
          //Populate the local temparature data
          itoa(lm73_temp, temp_string_array, 10);
          new_data_needed = FALSE;
      }//End if  
      //Do Nothing unless new data needed or interrupt  
    }//End While
}//main



//***********************************************************************
//                            USART0_recieve_interrupt
//**********************************************************************
//Get the temp command from the ATMega128
ISR(USART_RX_vect) {
    rx_m128_command = uart_getc();
    if(rx_m128_command == 0xAB){
        //If the recieved command is valid send the temparature
        uart_puts(temp_string_array);
    }
    new_data_needed = TRUE;
}



