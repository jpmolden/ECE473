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


fm_pwr_up(); //powerup the radio as appropriate
volatile uint16_t  current_fm_freq = 9990; //arg2, arg3: 99.9Mhz, 200khz steps
fm_tune_freq(); //tune radio to frequency in current_fm_freq

//to tune down or up in frequency in 200khz steps
current_fm_freq -= 20;
current_fm_freq += 20;


//to read signal strength...
while(twi_busy()){} //make sure TWI is not busy 
fm_rsq_status()
//save tune status 
rssi =  si4734_tune_status_buf[4];


   


//******************************************************************************
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){STC_interrupt = TRUE;}
/***********************************************************************/

