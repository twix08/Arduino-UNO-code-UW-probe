/************************************************************************************/
// Programmer:      Kim Nykvist
// Project:         Underwater probe for deep sea exploration
// Ordered by:	    The Royal Institute of Technology, KTH
// Version:         v.04.1 (b)
// Date:	        08-10-2019
// Updated:         10-12-2019
/************************************************************************************/

// Include .h-files
#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <avr/io.h>   
#include <stdio.h>
#include <AD9850.h>
#include <rotary.h>
#include <EEPROM.h>

// LCD configuration/initiation
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);

// DDS - constants & variables
#define W_CLK  8
#define FQ_UD  9
#define DATA  10
#define RESET 11
#define pulseHigh( pin ) { digitalWrite( pin, HIGH );  digitalWrite( pin, LOW ); }
	
// Rotaries setups
Rotary rotary_1						= Rotary(2,3);			// sets the pins the rotary encoder uses (must be interrupt pins).
const int pinA2						= 4;
const int pinB2						= 5; 
const int pinA1						= 6;
const int pinB1						= 7;

// Variables
int_fast32_t choice_frequency		= 2500;
int_fast32_t frequency_for_trim		= 124999500;	
int_fast32_t phase					= 0;
int_fast32_t frequency_1			= 0;		
int_fast32_t frequency_2			= 0;		
int_fast32_t guard					= 72;		
int_fast32_t guard2					= 1;		
int_fast32_t period					= 72;		
int_fast32_t period2				= 1;		
int_fast32_t mark					= 1;
int_fast32_t mark2					= 1;
int_fast32_t space					= 1;
int_fast32_t space2					= 1;
int_fast32_t down_time				= 1;
int_fast32_t down_time2				= 1;
int_fast32_t send_function			= 1;
int_fast32_t send_function2			= 1;
int_fast32_t increment_size			= 1;		
int_fast32_t increment_guard		= 1;
int_fast32_t increment_period		= 1;
int_fast32_t increment_mark			= 1;
int_fast32_t increment_space		= 1;
int_fast32_t increment_down_time	= 1;
int_fast32_t increment_send_funcion = 1;
int_fast32_t increment_repetitions  = 1;
int_fast32_t increment_shift		= 1;
int_fast32_t shift					= 1;
int_fast32_t shift2					= 1;
int_fast32_t frequency				= 0;
int_fast32_t time_gone_by			= millis();

int send_function_nr				= 1;
int increment_choice				= 2;		
int increment_step_function			= 1;
int repetitions						= 1;
int repetitions2					= 1;
int buttonstate						= 0;
int buttonstate1					= 0;
int buttonstate2					= 0;        
int buttonstate3					= 0;		
int buttonstate4					= 0;
int freq_button_1					= 0;		
int guard_button					= 0;
int period_button					= 0;
int send_button						= 0;
int hertzPosition_shift				= 5;
int hertz_position_0				= 5;
int hertz_position_1				= 5;
int hertz_position_2				= 5;
int position_guard					= 5;
int position_period					= 5;
int choice_position					= 1;
int step_choice_position			= 1;
int position_down_time				= 1;
int position_mark					= 1;
int position_space					= 1;
int position_send_funcion			= 1;
int position_repetitions			= 1;
int memory_status_1					= 1;		// memory stats holder: 1 = current and 0 = previous.
int force_frequency					= 0;		

unsigned long current_time_1;
unsigned long loop_time_1;
unsigned int send_time_1			= 8;	
unsigned int guard_time_1			= 10;	
	
String hertz0						= " 1 Hz";
String hertz1						= " 1 Hz";
String hertz2						= " 1 Hz";
String hertz4						= " 1 Hz";
String time_guard					= " 1 us";
String time_period					= " 1 us";
String time_mark					= " 1 us";
String time_space					= " 1 us";
String time_down_time				= " 1 us";
String choice						= "Set frequency";
String step_choice					= "Set frequency step";
String send_funcion_choice			= "Send function 1 ";
String nr_of_repetitions			= " 1 st";
String frequency_string;			

// Placeholders
byte ones1, tens1, hundreds1, thousands1, tenthousands1, hundredthousands1, millions1; 
byte ones2, tens2, hundreds2, thousands2, tenthousands2, hundredthousands2, millions2;
byte ones3, tens3, hundreds3, thousands3, tenthousands3, hundredthousands3, millions3;
byte ones4, tens4, hundreds4, thousands4, tenthousands4, hundredthousands4, millions4; 
byte ones5, tens5, hundreds5, thousands5, tenthousands5, hundredthousands5, millions5;
byte ones6, tens6, hundreds6, thousands6, tenthousands6, hundredthousands6, millions6;
byte ones7, tens7, hundreds7, thousands7, tenthousands7, hundredthousands7, millions7;
byte ones8, tens8, hundreds8, thousands8, tenthousands8, hundredthousands8, millions8;

// Routine (Interrupt) for catching the rotary/encoder
ISR(PCINT2_vect)
{
	unsigned char result = rotary_1.process(); 
	
	// Frequency Change
	if (increment_choice == 1)
	{
		// Control/see if the rotary/encoder is turned
		if (result) 
		{	
			// If freq_button is pushed (low): change frequency
			if (freq_button_1 == LOW)
			{
				// ClockWise Direction
				if (result == DIR_CW)
				{ 
					frequency_1 = frequency_1 + increment_size;
				}  
				
				// Backward Direction
				else 
				{
					frequency_1 = frequency_1 - increment_size;
				};
						
				// upper limit for frequency
				if (frequency_1 >= 20000000)		
				{
					frequency_1 = frequency_2;
				};
				
				// lower limit for frequency
				if (frequency_1 <= 0.1)			
				{
					frequency_1 = frequency_2;
				};	
			}
		}
	}
	
	// Guard Time Change
	if (increment_choice == 2){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW) {guard = guard + increment_guard;}
					else {guard = guard - increment_guard;};
				if (guard >= 20000000) {guard = guard2;};		 
				if (guard <= 0.1) {guard = guard2;};			
			}
		}
	}

	// Period Time Change
	if (increment_choice == 3){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW) {period = period + increment_period;} 
					else {period = period - increment_period;};
				if (period >= 20000000){period = period2;}; 
				if (period <= 0.1){ period = period2;};
			}
		}
	}
	
	// Mark Time Change - the high frequency in FSK modulation
	if (increment_choice == 4){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW){ mark = mark + increment_mark;} 
				else {mark = mark - increment_mark;};
				if (mark >= 20000000) { mark = mark2;};
				if (mark <= 0.1) { mark = mark2;};
			}
		}
	}
	
	// Space Time Change - the low frequency in FSK modulation
	if (increment_choice == 5){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW) { space = space + increment_space;}  
				else { space = space - increment_space;};
				if (space >= 20000000) { space = space2;}; 
				if (space <= 0.1) { space = space2;};
			}
		}
	}

	// Down Time Change
	if (increment_choice == 6){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW) { down_time = down_time + increment_down_time;} 
				else { down_time = down_time - increment_down_time;};
				if (down_time >= 20000000) { down_time = down_time2;}; 
				if (down_time <= 0.1) { down_time = down_time2;};
			}
		}
	}
	
	// Repetitions Change
	if (increment_choice == 7){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW) { repetitions = repetitions + increment_repetitions;} 
				else { repetitions = repetitions - increment_repetitions;};
				if (repetitions >= 20000000) { repetitions = repetitions2;}; 
				if (repetitions <= 0.1) { repetitions = repetitions2;};
			}
		}
	}
	
	// Shift Change
	if (increment_choice == 8){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW) { shift = shift + increment_shift;} 
				else { shift = shift - increment_shift;};
				if (shift >= 20000000) { shift = shift2;};
				if (shift <= 0.1) { shift = shift2;};
			}
		}
	}
	
	// Send function Change
	if (increment_choice == 9){
		if (result) {
			if (freq_button_1 == LOW) {
				if (result == DIR_CW) { send_function = send_function + increment_send_funcion;} 
				else { send_function = send_function - increment_send_funcion;};
				if (send_function >= 20000000) { send_function = send_function;};
				if (send_function <= 0.1) { send_function = send_function2;};
			}
		}
	}
}

//****************************************************************************//
//          FUNCTIONS														  //
//                    Sine wave functions                                     //
//				     (i.e., send functions)                  			      //
//****************************************************************************//

// First Send Function:
//	(Try/test if getting anything at all to propagate through)
//	(Use a three quarter wavelength frequency, as it's the "strongest TVR" (transmission voltage response))
void sendFunction_1(int send_freq, unsigned int send_time, unsigned int send_guard) {  
	while(1)    // using while-loops as are having to use the reset-button to break the functions
				// otherwise the AD9850 do not behave accordingly
	{
		// set frequency and phase
		DDS.setfreq(send_freq, 90);
		delayMicroseconds(send_time);	

		// down time
		DDS.down();
		delay(send_guard); 
	}
}

// Function number 2: aluminum pipe, quarter wavelength (3710 Hz according to simulations)
void sendFunction_2(int send_freq, unsigned int send_time, unsigned int send_guard) {  // perhaps use "uint_fast32_t"
	while(1)
	{
		DDS.setfreq(3710, 90);
		delayMicroseconds(send_time);	
		
		DDS.down();						
		delay(send_guard);  // OBS! delay(); använder ms istället för us som i delayMicroseconds(); 	
	}
}

// Function number 3: stainless steel pipe, quarter wavelength (4360 Hz according to simulations)
void sendFunction_3(int send_freq, unsigned int send_time, unsigned int send_guard) {
	while(1)
	{
		DDS.setfreq(4360, 90);
		delayMicroseconds(send_time);
		
		DDS.down();
		delay(send_guard); 
	}
}

// Function number 4: aluminum pipe, three quarter wavelength (15260 Hz according to simulations)
void sendFunction_4(int send_freq, unsigned int send_time, unsigned int send_guard) {
	while(1)
	{
		DDS.setfreq(15260, 90);
		delayMicroseconds(send_time);
		
		DDS.down();
		delay(send_guard);  // OBS! delay(); använder ms istället för us som i delayMicroseconds();
	}
}

// Function number 5: stainless steel pipe, three quarter wavelength (13570 Hz according to simulations)
void sendFunction_5(int send_freq, unsigned int send_time, unsigned int send_guard) {
	while(1)
	{
		DDS.setfreq(13570, 90);
		delayMicroseconds(send_time);
		
		DDS.down();
		delay(send_guard);  // OBS! delay(); använder ms istället för us som i delayMicroseconds();
	}
}

// Function number 6: Frequency sweep
void sendFunction_6(int send_freq, unsigned int send_time, unsigned int send_guard, int mark_as_limit, int space_as_increment_size) {
	while(1)
	{
		// frequency to increment; to creat the sweep 
		int sweep_freq = send_freq;
		
		while(sweep_freq < mark_as_limit)
		{
			// send_freq set as starting frequency
			DDS.setfreq(sweep_freq, 90);
			delayMicroseconds(send_time);

			DDS.down();					
			delay(send_guard);			
	
			// increment frequency
			sweep_freq = sweep_freq + space_as_increment_size;
		}
		
		// delay in between sweeps
		delay(10000);
	}
}

// Function number 7: Modulated message with "5-bit Baudot Code".
// Sending the message: "HELLO_WORLD".
// Using the "ITA2 Standard", i.e., "5-bit Baudot Code".
// Using send_time to set Mark, Space and Release.
// Using 2 x send_time to set the Latch (i.e., latch_time)
void sendFunction_7(int send_freq, unsigned int send_time, unsigned int send_guard, int shift_baudot, unsigned int down_time_baudot, int mark_baudot, int space_baudot) {

	// Initialize the Latch
	unsigned int latch_time = send_time + send_time;
	
	// The Actual Send Sequence
	while(1) 
	{
		/* First symbol */
		/*     "H"      */
		// Release
		DDS.setfreq(space_baudot, 90);		// Release 
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);		
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch   
		delayMicroseconds(latch_time);
	
		// Guard time
		DDS.down();
		delay(send_guard); 
	
	
		/* Second symbol */	
		/*     "E"       */		
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch   
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard); 
		
		
		/* Third symbol */
		/*     "L"      */	
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch   
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard); 
		
		
		/* Forth symbol */
		/*     "L"      */
			
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch 
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard); 
		
		
		/* Fifth symbol */
		/*     "O"      */
		
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard);  
		
		
		/* Sixth symbol */
		/*   "Space"    */	
			
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch  
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard); 
		
		
		/* Seventh symbol */
		/*      "W "      */
				
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
				
		// five bits of message below
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
				
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
				
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
				
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
				
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
				
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch   
		delayMicroseconds(latch_time);
				
		// Guard time
		DDS.down();
		delay(send_guard); 
		
		
		/* Eight symbol */
		/*      "O"     */
		
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch 
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard); 
		
		
		/* Ninth symbol */
		/*      "R"     */
		
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard); 
				
		
		/* Ninth symbol */
		/*      "L"     */
		
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);		// Latch  
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard);
				
		
		/* Ninth symbol */
		/*      "D"     */
		
		// Release
		DDS.setfreq(space_baudot, 90);		// Release
		delayMicroseconds(send_time);
		
		// five bits of message below
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_baudot, 90);		// 0
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_baudot, 90);		// 1
		delayMicroseconds(send_time);
		
		// Latch
		DDS.setfreq(mark_baudot, 90);	
		delayMicroseconds(latch_time);
		
		// Guard time
		DDS.down();
		delay(send_guard);  
				
		// the END of CODE/Message
		DDS.down();
		delay(10000);   // 10 sec. delay	
	}
}

// Test av optimalt "shift" för att kunna få så bra FSK som möjligt
void sendFunction_8(int send_freq, unsigned int send_time, unsigned int send_guard, int shift_size) {
	
	int mark_frequency  = send_freq + shift_size / 2;
	int space_frequency = send_freq - shift_size / 2; 
	
	while(1)
	{
		DDS.setfreq(mark_frequency, 90);		// 1
		delayMicroseconds(send_time);
		
		DDS.setfreq(space_frequency, 90);		// 0 
		delayMicroseconds(send_time);
		
		DDS.setfreq(mark_frequency, 90);		// 1
		delayMicroseconds(send_time);

		DDS.setfreq(mark_frequency, 90);		// 1
		delayMicroseconds(send_time);

		DDS.setfreq(space_frequency, 90);		// 0
		delayMicroseconds(send_time);

		DDS.setfreq(mark_frequency, 90);		// 1
		delayMicroseconds(send_time);

		delay(send_guard);
	}
}

void sendFunction_9(int send_freq, unsigned int send_time, unsigned int send_guard) {
	while(1)
	{
		DDS.setfreq(4000, 90);
		delayMicroseconds(send_time);
		
		DDS.setfreq(6000, 90);
		delayMicroseconds(send_time);
	}
}

void sendFunction_10(int send_freq, unsigned int send_time, unsigned int send_guard) {
	while(1)
	{
		DDS.setfreq(4000, 90);
		delayMicroseconds(send_time);
		
		DDS.setfreq(6000, 90);
		delayMicroseconds(send_time);
	}
}

void sendFunction_11(int send_freq, unsigned int send_time, unsigned int send_guard) {
	while(1)
	{
		DDS.setfreq(4000, 90);
		delayMicroseconds(send_time);
		
		DDS.setfreq(6000, 90);
		delayMicroseconds(send_time);
	}
}

void setincrement(){   
	if(increment_size == 1)
	{
		increment_size = 10; hertz0 = "10 Hz"; hertz_position_0=5;
	}
	else if (increment_size == 10)
	{
		increment_size = 50;  hertz0 = "50 Hz"; hertz_position_0=5;
	}
	else if (increment_size == 50)
	{
		increment_size = 100;  hertz0 = "100 Hz"; hertz_position_0=4;
	}
	else if (increment_size == 100)
	{
		increment_size = 500; hertz0="500 Hz"; hertz_position_0=4;
	}
	else if (increment_size == 500)
	{
		increment_size = 1000; hertz0="1 KHz"; hertz_position_0=6;
	}
	else if (increment_size == 1000)
	{
		increment_size = 2500; hertz0="2.5 KHz"; hertz_position_0=4;
	}
	else if (increment_size == 2500)
	{
		increment_size = 5000; hertz0="5 KHz"; hertz_position_0=6;
	}
	else if (increment_size == 5000)
	{
		increment_size = 10000; hertz0="10 KHz"; hertz_position_0=5;
	}
	else if (increment_size == 10000)
	{
		increment_size = 100000; hertz0="100 KHz"; hertz_position_0=4;
	}
	else if (increment_size == 100000)
	{
		increment_size = 1000000; hertz0="1 MHz"; hertz_position_0=6;
	}
	else
	{
		increment_size = 1; hertz0 = "1 Hz"; hertz_position_0=6;
	};  
	
	lcd.setCursor(0,0);
	lcd.print("                    ");
	lcd.setCursor(hertz_position_0,0);
	lcd.print(hertz0);
	
	delay(250); // Delay for the "scroll speed"
};

void setincrement_shift(){
	if(increment_shift == 1){
		increment_shift = 10; hertz4 = "10 Hz"; hertzPosition_shift=5;} 
	else if (increment_shift == 10){
		increment_shift = 50;  hertz4 = "50 Hz"; hertzPosition_shift=5;}
	else if (increment_shift == 50){
		increment_shift = 100;  hertz4 = "100 Hz"; hertzPosition_shift=4;}
	else if (increment_shift == 100){
		increment_shift = 500; hertz4= "500 Hz"; hertzPosition_shift=4;}
	else if (increment_shift == 500){
		increment_shift = 1000; hertz4= "1 KHz"; hertzPosition_shift=6;}
	else if (increment_shift == 1000){
		increment_shift = 2500; hertz4= "2.5 KHz"; hertzPosition_shift=4;}
	else if (increment_shift == 2500){
		increment_shift = 5000; hertz4= "5 KHz"; hertzPosition_shift=6;}
	else if (increment_shift == 5000){
		increment_shift = 10000; hertz4= "10 KHz"; hertzPosition_shift=5;}
	else if (increment_shift == 10000){
		increment_shift = 100000; hertz4= "100 KHz"; hertzPosition_shift=4;}
	else if (increment_shift == 100000){
		increment_shift = 1000000; hertz4= "1 MHz"; hertzPosition_shift=6;}
	else{
		increment_shift = 1; hertz4 = "1 Hz"; hertzPosition_shift=6;}; 
	lcd.setCursor(0,0);
	lcd.print("                    ");
	lcd.setCursor(hertzPosition_shift,0);
	lcd.print(hertz4);
	delay(250); // Delay for the "scroll speed"
};

// Increments in micro-seconds
void setincrement_mark(){
	if(increment_mark == 1){
		increment_mark = 10; hertz1 = "10 Hz"; hertz_position_1=5;} 
	else if (increment_mark == 10){
		increment_mark = 50;  hertz1 = "50 Hz"; hertz_position_1=5;}
	else if (increment_mark == 50){
		increment_mark = 100;  hertz1 = "100 Hz"; hertz_position_1=4;}
	else if (increment_mark == 100){
		increment_mark = 500; hertz1= "500 Hz"; hertz_position_1=4;}
	else if (increment_mark == 500){
		increment_mark = 1000; hertz1= "1 KHz"; hertz_position_1=6;}
	else if (increment_mark == 1000){
		increment_mark = 2500; hertz1= "2.5 KHz"; hertz_position_1=4;}
	else if (increment_mark == 2500){
		increment_mark = 5000; hertz1= "5 KHz"; hertz_position_1=6;}
	else if (increment_mark == 5000){
		increment_mark = 10000; hertz1= "10 KHz"; hertz_position_1=5;}
	else if (increment_mark == 10000){
		increment_mark = 100000; hertz1= "100 KHz"; hertz_position_1=4;}
	else if (increment_mark == 100000){
		increment_mark = 1000000; hertz1= "1 MHz"; hertz_position_1=6;}
	else{
		increment_mark = 1; hertz1 = "1 Hz"; hertz_position_1=6;};
	lcd.setCursor(0,0);
	lcd.print("                    ");
	lcd.setCursor(hertz_position_1,0);
	lcd.print(hertz1);
	delay(250); // Delay for the "scroll speed"	
};

// Increments in micro-seconds
void setincrement_space(){
	if(increment_space == 1){
		increment_space = 10; hertz2 = "10 Hz"; hertz_position_2=5;} 
	else if (increment_space == 10){
		increment_space = 50;  hertz2 = "50 Hz"; hertz_position_2=5;}
	else if (increment_space == 50){
		increment_space = 100;  hertz2 = "100 Hz"; hertz_position_2=4;}
	else if (increment_space == 100){
		increment_space = 500; hertz2= "500 Hz"; hertz_position_2=4;}
	else if (increment_space == 500){
		increment_space = 1000; hertz2= "1 KHz"; hertz_position_2=6;}
	else if (increment_space == 1000){
		increment_space = 2500; hertz2= "2.5 KHz"; hertz_position_2=4;}
	else if (increment_space == 2500){
		increment_space = 5000; hertz2= "5 KHz"; hertz_position_2=6;}
	else if (increment_space == 5000){
		increment_space = 10000; hertz2= "10 KHz"; hertz_position_2=5;}
	else if (increment_space == 10000){
		increment_space = 100000; hertz2= "100 KHz"; hertz_position_2=4;}
	else if (increment_space == 100000){
		increment_space = 1000000; hertz2= "1 MHz"; hertz_position_2=6;}
	else{
		increment_space = 1; hertz2 = "1 Hz"; hertz_position_2=6;}; 
	lcd.setCursor(0,0);
	lcd.print("                    ");
	lcd.setCursor(hertz_position_2,0);
	lcd.print(hertz2);
	delay(250); // Delay for the "scroll speed"
};

// Increments in micro-seconds
void setincrement_guard(){
	if(increment_guard == 1){
		increment_guard = 10; time_guard = "10 us"; position_guard=5;} 
	else if (increment_guard == 10){
		increment_guard = 50;  time_guard = "50 us"; position_guard=5;}
	else if (increment_guard == 50){
		increment_guard = 100;  time_guard = "100 us"; position_guard=4;}
	else if (increment_guard == 100){
		increment_guard = 500; time_guard= "500 us"; position_guard=4;}
	else if (increment_guard == 500){
		increment_guard = 1000; time_guard= "1K us"; position_guard=6;}
	else if (increment_guard == 1000){
		increment_guard = 2500; time_guard= "2.5K us"; position_guard=4;}
	else if (increment_guard == 2500){
		increment_guard = 5000; time_guard= "5K us"; position_guard=6;}
	else if (increment_guard == 5000){
		increment_guard = 10000; time_guard= "10K us"; position_guard=5;}
	else if (increment_guard == 10000){
		increment_guard = 100000; time_guard= "100K us"; position_guard=4;}
	else if (increment_guard == 100000){
		increment_guard = 1000000; time_guard= "1M us"; position_guard=6;}
	else{
		increment_guard = 1; time_guard = "1 us"; position_guard=6;};
	lcd.setCursor(0,2);
	lcd.print("                    ");
	lcd.setCursor(position_guard,2);    
	lcd.print(time_guard);
	delay(250); // Delay for the "scroll speed"
};

// Increments in micro-seconds
void setincrement_period(){
	if(increment_period == 1){
		increment_period = 10; time_period = "10 us"; position_period=5;}
	else if (increment_period == 10){
		increment_period = 50;  time_period = "50 us"; position_period=5;}
	else if (increment_period == 50){
		increment_period = 100;  time_period = "100 us"; position_period=4;}
	else if (increment_period == 100){
		increment_period = 500; time_period= "500 us"; position_period=4;}
	else if (increment_period == 500){
		increment_period = 1000; time_period= "1K us"; position_period=6;}
	else if (increment_period == 1000){
		increment_period = 2500; time_period= "2.5K us"; position_period=4;}
	else if (increment_period == 2500){
		increment_period = 5000; time_period= "5K us"; position_period=6;}
	else if (increment_period == 5000){
		increment_period = 10000; time_period= "10K us"; position_period=5;}
	else if (increment_period == 10000){
		increment_period = 100000; time_period= "100K us"; position_period=4;}
	else if (increment_period == 100000){
		increment_period = 1000000; time_period= "1M us"; position_period=6;}
	else{
		increment_period = 1; time_period = "1 us"; position_period=6;}; 
	lcd.setCursor(0,3);
	lcd.print("                    ");
	lcd.setCursor(position_period,3);  
	lcd.print(time_period);
	delay(250); // Delay for the "scroll speed"
};

// Increments in micro-seconds
void setincrement_down_time(){
	if(increment_down_time == 1){
		increment_down_time = 10; time_down_time = "10 us"; position_down_time=5;}
	else if (increment_down_time == 10){
		increment_down_time = 50;  time_down_time = "50 us"; position_down_time=5;}
	else if (increment_down_time == 50){
		increment_down_time = 100;  time_down_time = "100 us"; position_down_time=4;}
	else if (increment_down_time == 100){
		increment_down_time = 500; time_down_time= "500 us"; position_down_time=4;}
	else if (increment_down_time == 500){
		increment_down_time = 1000; time_down_time= "1K us"; position_down_time=6;}
	else if (increment_down_time == 1000){
		increment_down_time = 2500; time_down_time= "2.5K us"; position_down_time=4;}
	else if (increment_down_time == 2500){
		increment_down_time = 5000; time_down_time= "5K us"; position_down_time=6;}
	else if (increment_down_time == 5000){
		increment_down_time = 10000; time_down_time= "10K us"; position_down_time=5;}
	else if (increment_down_time == 10000){
		increment_down_time = 100000; time_down_time= "100K us"; position_down_time=4;}
	else if (increment_down_time == 100000){
		increment_down_time = 1000000; time_down_time= "1M us"; position_down_time=6;}
	else{
		increment_down_time = 1; time_down_time = "1 us"; position_down_time=6;};  
	lcd.setCursor(0,3);
	lcd.print("                    ");
	lcd.setCursor(position_down_time,3);
	lcd.print(time_down_time);
	delay(250); // Delay for the "scroll speed"
};

// Increments/changes send function of choice
void setincrement_send_function(){
	if(increment_send_funcion == 1){
		send_function_nr = 2; increment_send_funcion = 10; send_funcion_choice = "Send function 2"; position_send_funcion=1;}  
	else if (increment_send_funcion == 10){
		send_function_nr = 3; increment_send_funcion = 50;  send_funcion_choice = "Send function 3"; position_send_funcion=1;}
	else if (increment_send_funcion == 50){
		send_function_nr = 4; increment_send_funcion = 100;  send_funcion_choice = "Send function 4"; position_send_funcion=1;}
	else if (increment_send_funcion == 100){
		send_function_nr = 5; increment_send_funcion = 500; send_funcion_choice="Send function 5"; position_send_funcion=1;}
	else if (increment_send_funcion == 500){
		send_function_nr = 6; increment_send_funcion = 1000; send_funcion_choice="Send function 6"; position_send_funcion=1;}
	else if (increment_send_funcion == 1000){
		send_function_nr = 7; increment_send_funcion = 2500; send_funcion_choice="Send function 7"; position_send_funcion=1;}
	else if (increment_send_funcion == 2500){
		send_function_nr = 8; increment_send_funcion = 5000; send_funcion_choice="Send function 8"; position_send_funcion=1;}
	else if (increment_send_funcion == 5000){
		send_function_nr = 9; increment_send_funcion = 10000; send_funcion_choice="Send function 9"; position_send_funcion=1;}
	else if (increment_send_funcion == 10000){
		send_function_nr = 10; increment_send_funcion = 100000; send_funcion_choice="Send function 10"; position_send_funcion=1;}
	else if (increment_send_funcion == 100000){
		send_function_nr = 11; increment_send_funcion = 1000000; send_funcion_choice="Send function 11"; position_send_funcion=1;}
	else{
		send_function_nr = 1; increment_send_funcion = 1; send_funcion_choice = "Send function 1"; position_send_funcion=1;}; 
	lcd.setCursor(0,3);
	lcd.print("                    ");
	lcd.setCursor(position_send_funcion,3);  
	lcd.print(send_funcion_choice);
	delay(250); // Delay for the "scroll speed"
};

// Increments in micro-seconds
void set_increment_repetitions(){
	if(increment_repetitions == 1){
		increment_repetitions = 10; nr_of_repetitions = "10 st"; position_repetitions=5;} 
	else if (increment_repetitions == 10){
		increment_repetitions = 50;  nr_of_repetitions = "50 st"; position_repetitions=5;}
	else if (increment_repetitions == 50){
		increment_repetitions = 100;  nr_of_repetitions = "100 st"; position_repetitions=4;}
	else if (increment_repetitions == 100){
		increment_repetitions = 500; nr_of_repetitions="500 st"; position_repetitions=4;}
	else if (increment_repetitions == 500){
		increment_repetitions = 1000; nr_of_repetitions="1K st"; position_repetitions=6;}
	else if (increment_repetitions == 1000){
		increment_repetitions = 2500; nr_of_repetitions="2.5K st"; position_repetitions=4;}
	else if (increment_repetitions == 2500){
		increment_repetitions = 5000; nr_of_repetitions="5K st"; position_repetitions=6;}
	else if (increment_repetitions == 5000){
		increment_repetitions = 10000; nr_of_repetitions="10K st"; position_repetitions=5;}
	else if (increment_repetitions == 10000){
		increment_repetitions = 100000; nr_of_repetitions="100K st"; position_repetitions=4;}
	else if (increment_repetitions == 100000){
		increment_repetitions = 1000000; nr_of_repetitions="1M st"; position_repetitions=6;}
	else{
		increment_repetitions = 1; nr_of_repetitions = "1 st"; position_repetitions=6;}; 
	lcd.setCursor(0,3);
	lcd.print("                    ");
	lcd.setCursor(position_repetitions,3);   
	lcd.print(nr_of_repetitions);
	delay(250); // Delay for the "scroll speed"
};

// Function to select/set which encoder function to use when turning the rotary
void set_increment_choice(){
	if(increment_choice == 1){
		increment_choice = 2; choice = "Set guard time"; choice_position=1;}  
	else if (increment_choice == 2){
		increment_choice = 3;  choice = "Set send time"; choice_position=1;}
	else if (increment_choice == 3){
		increment_choice = 4;  choice = "Set mark"; choice_position=1;}
	else if (increment_choice == 4){
		increment_choice = 5;  choice = "Set space"; choice_position=1;}
	else if (increment_choice == 5){
		increment_choice = 6;  choice = "Set down time"; choice_position=1;}
	else if (increment_choice == 6){
		increment_choice = 7;  choice = "Set repetitions"; choice_position=1;}
	else if (increment_choice == 7){
		increment_choice = 8;  choice = "Set shift"; choice_position=1;}
	else{
		increment_choice = 1; choice = "Set frequency"; choice_position=1;};  
	lcd.setCursor(0,1);
	lcd.print("                    ");
	lcd.setCursor(choice_position,1);
	lcd.print(choice);
	delay(500); // Delay for the "scroll speed"
};

// Function to select/set with increment button
void set_increment_step_function(){
	if(increment_step_function == 1){
		increment_step_function = 2; step_choice = "Set guard step"; step_choice_position=1;}  
	else if (increment_step_function == 2){
		increment_step_function = 3;  step_choice = "Set send step"; step_choice_position=1;}
	else if (increment_step_function == 3){
		increment_step_function = 4;  step_choice = "Set mark step"; step_choice_position=1;}
	else if (increment_step_function == 4){
		increment_step_function = 5;  step_choice = "Set space step"; step_choice_position=1;}
	else if (increment_step_function == 5){
		increment_step_function = 6;  step_choice = "Set downtime step"; step_choice_position=1;}
	else if (increment_step_function == 6){
		increment_step_function = 7;  step_choice = "Set repetition step"; step_choice_position=1;}
	else if (increment_step_function == 7){
		increment_step_function = 8;  step_choice = "Set shift step"; step_choice_position=1;}
	else if (increment_step_function == 8){
		increment_step_function = 9;  step_choice = "Set send function"; step_choice_position=1;}
	else{
		increment_step_function = 1; step_choice = "Set frequency step"; step_choice_position=1;};  
	lcd.setCursor(0,1);
	lcd.print("                    ");
	lcd.setCursor(step_choice_position,1);
	lcd.print(step_choice);
	delay(500); // Delay for the "scroll speed"
};

void show_frequency()
{
	millions1 = int(frequency_1/1000000);
	hundredthousands1 = ((frequency_1/100000)%10);
	tenthousands1 = ((frequency_1/10000)%10);
	thousands1 = ((frequency_1/1000)%10);
	hundreds1 = ((frequency_1/100)%10);
	tens1 = ((frequency_1/10)%10);
	ones1 = ((frequency_1/1)%10); 
	   
	lcd.setCursor(0,0);
	lcd.print("                    ");
	
	if (millions1 > 9){
		lcd.setCursor(1,0);}
	else{
		lcd.setCursor(2,0);}
		
	lcd.print(millions1);
	lcd.print(" ");
	lcd.print(hundredthousands1);
	lcd.print(tenthousands1);
	lcd.print(thousands1);
	lcd.print(" ");
	lcd.print(hundreds1);
	lcd.print(tens1);
	lcd.print(ones1);
	lcd.print(" Hz  ");
	
	time_gone_by = millis();
	memory_status_1 = 0;
	
	delay(100);
};

void showShift()
{
	millions8 = int(shift/1000000);
	hundredthousands8 = ((shift/100000)%10);
	tenthousands8 = ((shift/10000)%10);
	thousands8 = ((shift/1000)%10);
	hundreds8 = ((shift/100)%10);
	tens8 = ((shift/10)%10);
	ones8 = ((shift/1)%10);
	
	lcd.setCursor(0,0);
	lcd.print("                    ");
	
	if (millions8 > 9){
		lcd.setCursor(1,0);}
	else{
		lcd.setCursor(2,0);}
		
	lcd.print(millions8);
	lcd.print(" ");
	lcd.print(hundredthousands8);
	lcd.print(tenthousands8);
	lcd.print(thousands8);
	lcd.print(" ");
	lcd.print(hundreds8);
	lcd.print(tens8);
	lcd.print(ones8);
	lcd.print(" Hz  ");
	
	time_gone_by = millis();
	memory_status_1 = 0;
	
	delay(100);
};

void showMark()
{
	millions4 = int(mark/1000000);
	hundredthousands4 = ((mark/100000)%10);
	tenthousands4 = ((mark/10000)%10);
	thousands4 = ((mark/1000)%10);
	hundreds4 = ((mark/100)%10);
	tens4 = ((mark/10)%10);
	ones4 = ((mark/1)%10);
	
	lcd.setCursor(0,0);
	lcd.print("                    ");
	
	if (millions4 > 9){
		lcd.setCursor(1,0);}
	else{
		lcd.setCursor(2,0);}
		
	lcd.print(millions4);
	lcd.print(" ");
	lcd.print(hundredthousands4);
	lcd.print(tenthousands4);
	lcd.print(thousands4);
	lcd.print(" ");
	lcd.print(hundreds4);
	lcd.print(tens4);
	lcd.print(ones4);
	lcd.print(" Hz  ");
	
	time_gone_by = millis();
	memory_status_1 = 0;
	
	delay(100);
};

void showSpace()
{
	millions5 = int(space/1000000);
	hundredthousands5 = ((space/100000)%10);
	tenthousands5 = ((space/10000)%10);
	thousands5 = ((space/1000)%10);
	hundreds5 = ((space/100)%10);
	tens5 = ((space/10)%10);
	ones5 = ((space/1)%10);
	
	lcd.setCursor(0,0);
	lcd.print("                    ");
	
	if (millions5 > 9){
		lcd.setCursor(1,0);}
	else{
		lcd.setCursor(2,0);}
		
	lcd.print(millions5);
	lcd.print(" ");
	lcd.print(hundredthousands5);
	lcd.print(tenthousands5);
	lcd.print(thousands5);
	lcd.print(" ");
	lcd.print(hundreds5);
	lcd.print(tens5);
	lcd.print(ones5);
	lcd.print(" Hz  ");
	
	time_gone_by = millis();
	memory_status_1 = 0; 
	
	delay(100);
};

void showDownTime()
{
	millions6 = int(down_time/1000000);
	hundredthousands6 = ((down_time/100000)%10);
	tenthousands6 = ((down_time/10000)%10);
	thousands6 = ((down_time/1000)%10);
	hundreds6 = ((down_time/100)%10);
	tens6 = ((down_time/10)%10);
	ones6 = ((down_time/1)%10);
	
	lcd.setCursor(0,2);
	lcd.print("                    ");
	
	if (millions6 > 9){
		lcd.setCursor(1,2);}
	else{
		lcd.setCursor(2,2);}
		
	lcd.print(millions6);
	lcd.print(" ");
	lcd.print(hundredthousands6);
	lcd.print(tenthousands6);
	lcd.print(thousands6);
	lcd.print(" ");
	lcd.print(hundreds6);
	lcd.print(tens6);
	lcd.print(ones6);
	lcd.print(" DownT  ");
	
	time_gone_by = millis();
	memory_status_1 = 0;
	
	delay(100);
};

void showGuard()
{
	millions2 = int(guard/1000000);
	hundredthousands2 = ((guard/100000)%10);
	tenthousands2 = ((guard/10000)%10);
	thousands2 = ((guard/1000)%10);
	hundreds2 = ((guard/100)%10);
	tens2 = ((guard/10)%10);
	ones2 = ((guard/1)%10);
	
	lcd.setCursor(0,2);
	lcd.print("                    ");
	
	if (millions2 > 9){
		lcd.setCursor(1,2);}
	else{
		lcd.setCursor(2,2);}
		
	lcd.print(millions2);
	lcd.print(" ");
	lcd.print(hundredthousands2);
	lcd.print(tenthousands2);
	lcd.print(thousands2);
	lcd.print(" ");
	lcd.print(hundreds2);
	lcd.print(tens2);
	lcd.print(ones2);
	lcd.print(" GuardT");
	
	time_gone_by = millis();
	memory_status_1 = 0; 
	
	delay(100);
};

void showPeriod()
{
	millions3 = int( period/1000000);
	hundredthousands3 = (( period/100000) %10);
	tenthousands3 = (( period/10000) %10);
	thousands3 = (( period/1000) %10);
	hundreds3 = (( period/100) %10);
	tens3 = (( period/10) %10);
	ones3 = ( (period/1) %10);
	
	lcd.setCursor(0,3);
	lcd.print("                    ");
	
	if (millions3 > 9){
		lcd.setCursor(1,3);}
	else{
		lcd.setCursor(2,3);}
		
	lcd.print(millions3);
	lcd.print(" ");
	lcd.print(hundredthousands3);
	lcd.print(tenthousands3);
	lcd.print(thousands3);
	lcd.print(" ");
	lcd.print(hundreds3);
	lcd.print(tens3);
	lcd.print(ones3);
	lcd.print(" SendT.");
	
	time_gone_by = millis();
	memory_status_1 = 0;
	
	delay(100);
};

void showRepetitions()
{
	millions7 = int(repetitions/1000000);
	hundredthousands7 = ((repetitions/100000)%10);
	tenthousands7 = ((repetitions/10000)%10);
	thousands7 = ((repetitions/1000)%10);
	hundreds7 = ((repetitions/100)%10);
	tens7 = ((repetitions/10)%10);
	ones7 = ((repetitions/1)%10);
	
	lcd.setCursor(0,3);
	lcd.print("                    ");
	
	if (millions7 > 9){
		lcd.setCursor(1,3);}
	else{
		lcd.setCursor(2,3);}
		
	lcd.print(millions7);
	lcd.print(" ");
	lcd.print(hundredthousands7);
	lcd.print(tenthousands7);
	lcd.print(thousands7);
	lcd.print(" ");
	lcd.print(hundreds7);
	lcd.print(tens7);
	lcd.print(ones7);
	lcd.print(" Repet.");
	
	time_gone_by = millis();
	memory_status_1 = 0; 
	
	delay(100);	
}

void memory_storage()
{
	// Save/store main frequency to eeprom memory slots.
	EEPROM.write( 0, millions1 );
	EEPROM.write( 1, hundredthousands1 );
	EEPROM.write( 2, tenthousands1 );
	EEPROM.write( 3, thousands1 );
	EEPROM.write( 4, hundreds1 );
	EEPROM.write( 5, tens1 );
	EEPROM.write( 6, ones1 );
	
	memory_status_1 = 1;  // writing process complete
};

void setup()
{
	// Initialize 20x4 display
	lcd.begin(20, 4);					//  20x4 LCD display
	lcd.setBacklightPin(3, POSITIVE);	
	lcd.setBacklight(HIGH);

	// DDS setup
	DDS.begin(W_CLK, FQ_UD, DATA, RESET);
	DDS.calibrate(frequency_for_trim);
	
	// DDS setup
	pinMode( A0, INPUT ); // Connect to a button that goes to GND on push
	pinMode( A1, INPUT ); // Rotary button
	pinMode( A2, INPUT ); // Change_button - selects which function the encoder shall use
	pinMode( A3, INPUT ); // button that calls  the sendFunction

	// A0-A3 are the "Analog in" at Arduino UNO
	digitalWrite( A0, HIGH );   
	digitalWrite( A1, HIGH );		// Rotary button
	digitalWrite( A2, HIGH );		// Change_button - selects which function the encoder shall use
	digitalWrite( A3, HIGH );		// button that calls  the sendFunction
	PCICR |= ( 1 << PCIE2 );		// enable external interrupts. Tell the MCU to check PCMSK2 on a pin change
	PCMSK2 |= ( 1 << PCINT18 ) | ( 1 << PCINT19 ); // port 3 and 4 on port D enable interrupt
	sei();						// enable global interrupt
	
	pinMode( pinA1, INPUT );
	pinMode( pinB1, INPUT );
	pinMode( pinA2, INPUT );
	pinMode( pinB2, INPUT );
	current_time_1 = millis();
	loop_time_1 = current_time_1;
		
	pinMode( FQ_UD, OUTPUT );
	pinMode( W_CLK, OUTPUT );
	pinMode( DATA, OUTPUT );
	pinMode( RESET, OUTPUT );
	pulseHigh( RESET );
	pulseHigh( W_CLK );
	pulseHigh( FQ_UD );			// "this pulse enables serial mode on the AD9850" - AD9850 Data sheet p. 12
	lcd.setCursor(hertz_position_0,0);
	lcd.print(hertz0);
	lcd.setCursor(position_period,2);
	lcd.print(time_period);
	lcd.setCursor(position_guard,3);
	lcd.print(time_guard);

	// Store the main frequency
	if (force_frequency == 0) {
		frequency_string = String ( EEPROM.read(0) )
							+String ( EEPROM.read(1) )
							+String ( EEPROM.read(2) )
							+String ( EEPROM.read(3) )
							+String ( EEPROM.read(4) )
							+String ( EEPROM.read(5) )
							+String ( EEPROM.read(6) );
							
		frequency_1 = frequency_string.toInt();
	}
}

void loop()
{
	// Check if have a new frequency, and then update LCD if new
	if (frequency_1 != frequency_2)
	{
		show_frequency();
		frequency_2 = frequency_1;
	}

	// Update the display and guardtime if have gotten a new guardtime
	if (guard != guard2)
	{
		showGuard();
		guard2 = guard;
	}
	
	// Update the display and sendtime if have gotten a new sendtime
	if (period != period2)
	{
		showPeriod();
		period2 = period;
	}
	
	// Update the display and sendtime if have gotten a new sendtime
	if (mark != mark2)
	{
		showMark();
		mark2 = mark;
	}
	
	// Update the display and sendtime if have gotten a new sendtime
	if (space != space2)
	{
		showSpace();
		space2 = space;
	}
	
	// Update the display and sendtime if have gotten a new sendtime
	if (down_time != down_time2)
	{
		showDownTime();
		down_time2 = down_time;
	}
	
	// Update the display and sendtime if have gotten a new sendtime
	if (repetitions != repetitions2)
	{
		showRepetitions();
		repetitions2 = repetitions;
	}
	
	// Update the shift if have gotten a new shift
	if (shift != shift2)
	{
		showShift();
		shift2 = shift;
	}

	// Update the display and sendtime if have gotten a new sendtime
	if (send_function != send_function2)
	{
		send_function2 = send_function;
	}

	// (NOTE: Only A0-A3 are "Analog in" at the Arduino UNO)
	buttonstate4 = digitalRead(A1);    
	if(buttonstate4 == LOW) {
		if(send_function_nr == 1){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 1    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_1(frequency_1, period, guard);  			
		}
	
		if(send_function_nr == 2){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 2    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_2(frequency_1, period, guard);  
		}
		 
		if(send_function_nr == 3){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 3    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_3(frequency_1, period, guard); 
		}
		
		if(send_function_nr == 4){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 4    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_4(frequency_1, period, guard);
		}
		
		if(send_function_nr == 5){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 5    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_5(frequency_1, period, guard); 
		}

		if(send_function_nr == 6){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 6    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_6(frequency_1, period, guard, mark, space);
		}
		
		if(send_function_nr == 7){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 7    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
            sendFunction_7(frequency_1, period, guard, shift, down_time, mark, space); // period is the "sendtime"
		}
		
		if(send_function_nr == 8){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 8    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_8(frequency_1, period, guard, shift); 
		}
		
		if(send_function_nr == 9){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 9    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_9(frequency_1, period, guard); 
		}
		
		if(send_function_nr == 10){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 10    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_10(frequency_1, period, guard); 
		}	
	
		if(send_function_nr == 11){
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,1);
			lcd.print("Sending             ");
			lcd.setCursor(0,2);
			lcd.print("  Function nr. 11    ");
			lcd.setCursor(0,3);
			lcd.print("                    ");
			sendFunction_11(frequency_1, period, guard); 
		}
	}

	// select which to increment/increase
	buttonstate = digitalRead(A0);   // A0 is set to the encoders button - adjusting which to use the rotary for (freq, sendtime or guard)
	if(buttonstate == LOW)
	{
		set_increment_choice();
	};
	
	// set size of increments/increases
	buttonstate = digitalRead(A2);   
	if(buttonstate == LOW)
	{
		set_increment_step_function();
	};
		
	// Button for selecting which function the encoder uses
	buttonstate = digitalRead(A3);
	if(buttonstate == LOW)
	{
		if (increment_step_function == 1)
			setincrement();					// frequency
		if (increment_step_function == 2)
			setincrement_guard();			// guard time
		if (increment_step_function == 3)
			setincrement_period();			// send time
		if (increment_step_function == 4)
			setincrement_mark();			// mark frequency
		if (increment_step_function == 5)
			setincrement_space();			// space frequency
		if (increment_step_function == 6)
			setincrement_down_time();		// down time
		if (increment_step_function == 7)
			set_increment_repetitions();	// set repetitions
		if (increment_step_function == 8)
			setincrement_shift();			// set shift
		if (increment_step_function == 9)
			setincrement_send_function();	// set send function		
	};	

	// Store to eeprom if more than 2000 millisec have passed since last storage
	if(memory_status_1 == 0)
	{
		if( time_gone_by + 2000 < millis() )
		{
			memory_storage();
		}
	}
}	//    End of Code
