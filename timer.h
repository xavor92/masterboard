/*
 * timer.h
 *
 *  Created on: 02.04.2015
 *      Author: olli
 */

#ifndef TIMER_H_
#define TIMER_H_

/*
 *	Includes
 */

#include <avr/interrupt.h>

/*
 *	Defines
 */

#define TIMER_FLAG_MS1 0x01
#define TIMER_FLAG_MS100 0x02
#define TIMER_FLAG_MS1000 0x04

/*
 *	Global Functions, Variables (Declarations, extern)
 */

extern unsigned char timerStatus;
extern unsigned char getCountdown();
extern void setCountdown(unsigned char counter);
extern void TIMER_Init();

//TODO GetTimer0
//TODO Timer0 as RNG

#endif /* TIMER_H_ */
