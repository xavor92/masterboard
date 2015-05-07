/*
 * masterboard.c
 * 	Development Board Master for tool cabinet sensors
 *
 *  Created on: 16.04.2015
 *      Author: olli
 */

/*
 * DEFINES
 */

#define UART_BAUD_RATE 9600



#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "bitio.h"
#include "timer.h"
#include "dbus_master.h"
#include "uart.h"


int main(void)
{
	DDRD = 0xFF;
	TIMER_Init();
	dbus_init();
	sei();
	while(1)
	{
		if(timerStatus & TIMER_FLAG_MS1)
		{
			timerStatus &= ~TIMER_FLAG_MS1;
		}
		if(timerStatus & TIMER_FLAG_MS100)
		{
			timerStatus &= ~TIMER_FLAG_MS100;
			dbus_perform();
		}
		if(timerStatus & TIMER_FLAG_MS1000)
		{
			timerStatus &= ~TIMER_FLAG_MS1000;
		}
	}
}



