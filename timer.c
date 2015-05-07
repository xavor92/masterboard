/*
 * timer.c
 *
 *  Created on: 02.04.2015
 *      Author: olli
 */

/*
 *	Includes
 */

#include "timer.h"
#include <avr/io.h>

/*
 *	Defines
 */

#define TIMER2_FREQ 1000 //Overflow Frequency of Timer1
#define OCR2_VALUE (unsigned char)(((F_CPU/TIMER2_FREQ)/(32*2))-1)

/*
 *	Local Functions, Variables (Declaration, static)
 */

static unsigned char ms1_counter = 0;
static unsigned char ms100_counter = 0;
static unsigned char ms1000_counter = 0;
static unsigned char countdown;
unsigned char timerStatus = 0;

/*
 *	Global Functions (Definitions)
 */

extern void TIMER_Init()
{
	TCCR2 = (( 1 << WGM21) | ( 1 << CS22 )); //CTC Mode, Prescaler 32
	OCR2 = OCR2_VALUE;
	TIMSK |= (1 << OCIE2);
	timerStatus = 0;
}

extern unsigned char getCountdown()
{
	return countdown;
}

extern void setCountdown(unsigned char counter)
{
	countdown = counter;
}

/*
 *	Local Functions (Definitions)
 */

ISR(TIMER2_COMP_vect)	//Timer2 OCMatch Interrupt
{
	ms1_counter++;
	timerStatus |= TIMER_FLAG_MS1;
	if(countdown) countdown--;
	if(ms1_counter == 100)
	{
		ms1_counter = 0;
		ms100_counter++;
		timerStatus |= TIMER_FLAG_MS100;
	}
	if(ms100_counter == 10)
	{
		ms100_counter = 0;
		ms1000_counter++;
		timerStatus |= TIMER_FLAG_MS1000;
	}
};
