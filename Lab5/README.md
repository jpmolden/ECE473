# ECE473 - Lab 5

Purpose:
  *Using another mega family uC.
  *Modification of the makefile
  *Fuse bit modification
  *Start of using TWI interface.
  *Use of the UART module.

Project Specifications:
  *Using the supplied mega48 board, implement a remote temperature sensor 
   that sends back info to the alarm clock base via RS-232 signaling.
  *The mega128 board will send the temperature sensor a command to read 
   the temperature once per second. The remote board interprets the command 
   and responds.
  *The local mega128 board will also have a temperature sensor attached to it.  
  *Use the LCD to keep a real-time display of remote temperature as well as 
   local temperature.
  *Display the temperature on the LCD in a suitable format.
  *Temperature displayed to the whole digit.
  *Everything else should still work! 

Before Lab:
  *Sketch out the circuit that will connect the two boards.

Assembly:
  *The remote temperature sensor can be connected to the main base assembly with a 
   foot or so of ribbon cable. You could use up to 20ft or so at slower baud rates.
     
Software:
  *Add to your alarm clock code the local and remote temperature sensing 
   function.  Get the local temperature sensor working first. This is mostly
   just layering on the TWI code on top of your alarm clock code. 
  *You will need to modify the Makefile for the mega48 board.
  *ATMega48 chips come programmed at the factory to run at 1MHz. Thus you may
   need to set a "slow clock" jumpers if it exists to program it the first
   time until you set the clock speed faster with the fuse bits.
  *It will probably be a lot easier to interface the local temperature
   sensor to the mega128 board to get it working first, before having
   to code remote sensor.
  *Perhaps the best way to start this code is to have it write out a fixed 
   value of temperature to both LCD and UART.  When that is working, then 
   implement the I2C code. You may decide to merge this code with your existing 
   alarm clock code now or merge later.
  *I also suggest putting your UART functions in a separate C module to avoid making 
   your existing code any more confusing.

Test: 
  *See that the temperature sensor data is displayed and is correct.

Extra Credit:
  *Have the temperature display temperatures with one or two decimal points.  
  *Have the temperature display temperatures alternating in degrees F and C.   

Hint:
  For easy programming of the mega48 you must set the fuses to allow it to 
  run at 8mhz. (this may or may not be true with the newer blue programmers)
  Here is how:
  1. Put the "slow clock" jumper on the programmer
  2. Program the fuse bits as follows:
        EFUSE : don't change
        HFUSE : 0xDF
        LFUSE : 0xE2
  3. After this, you don't need the "slow clock" jumper

  Also, the best way to keep track of the different processor speeds is from
  within the makefile.  For your mega48 board, you would do the following in your 
  makefile...

#define CPU speed
  F_CPU          = 8000000UL

Further down in the makefile for the compiler flags make sure you have the compiler flag,
"DF_CPU" ...
#set compiler flags
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -DF_CPU=$(F_CPU)

This allows you to never have to specify the cpu speed in any file you are
using that is being compiled by that make file.  If you put the speed in a
file used by both mega128 and mega48 one or the other will be wrong.
  
What to show for your work:
  *Schematic diagram you used to wire up the remote board.
  *Pseudocode or flow chart for your code if used.
  *Printout of your code.  Use a2ps or format to make easily readable.  
  *Demonstrate working system to TA by before next lab.

Lab Weighting = 0.20

