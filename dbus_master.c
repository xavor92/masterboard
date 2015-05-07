/*
 * dbus_master.c
 *
 *  Created on: 02.04.2015
 *      Author: olli
 */

/*
 *	Includes
 */

#include "dbus_master.h"
#include "uart.h"
#include "bitio.h"
#include "timer.h"
#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

/*
 *	Defines & Variables
 */

#define LED_MASK ((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7))

#define START_REQ 0x3F
#define START_RES 0x2A


static enum Status frameStatus;
static frame frameBuffer;
static unsigned char repeated_check = 0;


/*
 *	Local Functions (Declaration, static)
 */

static unsigned int calc_crc(frame *framePointer);
static unsigned char send_Frame(frame *framePointer);
static void check_low();
static void check_high();
static void invert_leds();
static void check_occupied();

/*
 *	Global Functions (Definitions)
 */

extern void dbus_init()
{
	uart_init(UART_BAUD_SELECT(DBUS_BAUD, F_CPU));
	DDRB |= LED_MASK;
	repeated_check = 0;
	PORTB |= ( (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4));
}

extern void dbus_receive()
{
	unsigned int c = uart_getc();
	unsigned char* helper;
	if(!(c & UART_NO_DATA))				//if Data available
	{
		if( ( (c & 0xFF) == START_REQ) || ( (c & 0xFF) == START_RES) )
		{
			frameBuffer.start = (c & 0xFF);
			frameStatus = statusStart;
		} else
		{
			switch(frameStatus)
			{
			case statusStart:
				frameBuffer.address = (c & 0xFF);
				uart_puts("start");
				frameStatus = statusAddress;
				break;
			case statusAddress:
				frameBuffer.function = (c & 0xFF);
				uart_puts("address");
				frameStatus = statusFunction;
				break;
			case statusFunction:
				frameBuffer.data = (c & 0xFF);
				uart_puts("data");
				frameStatus = statusData;
				break;
			case statusData:
				helper = (unsigned char*)&frameBuffer.crc;
				uart_puts("crch");
				helper[1] = (c & 0xFF);
				frameStatus = statusCrcH;
				break;
			case statusCrcH:
				helper = (unsigned char*)&frameBuffer.crc;
				uart_puts("crcl");
				helper[0] = (c & 0xFF);
				frameStatus = statusReceiveComplete;
				break;
			default:
				uart_puts("frame error");
				break;
			}
		}
	}
}

extern void dbus_perform()
{
	if((PINB & (1 << PB4)) == 0)
	{
		check_low();
	}
	if((PINB & (1 << PB3)) == 0)
	{
		check_high();
	}
	if((PINB & (1 << PB2)) == 0 || repeated_check == 1)
	{
		check_occupied();
	}
	if((PINB & (1 << PB1)) == 0)		//LED invert
	{
		invert_leds();
	}
}

/*
 *	Local Functions (Definitions)
 */

static unsigned int calc_crc(frame *framePointer)
{
	unsigned int crc_reg = 0xFFFF;	//init crc

	//calculate crc of complete frame

	crc_reg ^= framePointer->start;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	crc_reg ^= framePointer->address;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	crc_reg ^= framePointer->function;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	crc_reg ^= framePointer->data;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	return crc_reg;
}

static unsigned char send_Frame(frame *framePointer)
{
	unsigned char *helper;
	helper = (unsigned char *)&framePointer->crc;
	framePointer->crc = calc_crc(framePointer);
	uart_putc(framePointer->start);
	uart_putc(framePointer->address);
	uart_putc(framePointer->function);
	uart_putc(framePointer->data);
	uart_putc(helper[1]);
	uart_putc(helper[0]);
	return 0x00;
}

void check_low()
{
	frameBuffer.address = 0x01;
	frameBuffer.start = START_REQ;
	frameBuffer.function = FUNCTION_GET_LOW_SENS;
	frameBuffer.data = 0x00;
	send_Frame(&frameBuffer);
	frameStatus = statusUndefined;
	setCountdown(10);
	while(getCountdown()/* && frameStatus != statusReceiveComplete*/)
	{
		dbus_receive();
	}
	if(frameStatus == statusReceiveComplete)
	{
		if(frameBuffer.crc == calc_crc(&frameBuffer))
		{
			frameStatus = statusCrcCorrect;
			BIT_SET(&PORTD, PD6);
			BIT_SET(&PORTD, PD7);
			if(frameBuffer.data > 128)
			{
				BIT_CLEAR(&PORTD, PD6);
				if(frameBuffer.data > 192) BIT_CLEAR(&PORTD, PD7);
			} else
			{
				if(frameBuffer.data > 64) BIT_CLEAR(&PORTD, PD7);
			}
		}
	}
}

void check_high()
{
	frameBuffer.address = 0x01;
	frameBuffer.start = START_REQ;
	frameBuffer.function = FUNCTION_GET_HIGH_SENS;
	frameBuffer.data = 0x00;
	send_Frame(&frameBuffer);
	frameStatus = statusUndefined;
	setCountdown(10);
	while(getCountdown()/* && frameStatus != statusReceiveComplete*/)
	{
		dbus_receive();
	}
	if(frameStatus == statusReceiveComplete)
	{
		if(frameBuffer.crc == calc_crc(&frameBuffer))
		{
			frameStatus = statusCrcCorrect;
			BIT_SET(&PORTD, PD4);
			BIT_SET(&PORTD, PD5);
			if(frameBuffer.data > 128)
			{
				BIT_CLEAR(&PORTD, PD5);
				if(frameBuffer.data > 192) BIT_CLEAR(&PORTD, PD4);
			} else
			{
				if(frameBuffer.data > 64) BIT_CLEAR(&PORTD, PD4);
			}
		}
	}
}

void invert_leds()
{
	frameBuffer.address = 0x01;
	frameBuffer.start = START_REQ;
	frameBuffer.function = FUNCTION_GET_PORTD;
	frameBuffer.data = 0x00;
	send_Frame(&frameBuffer);
	frameStatus = statusUndefined;
	setCountdown(10);
	while(getCountdown()/* && frameStatus != statusReceiveComplete*/)
	{
		dbus_receive();
	}
	if(frameStatus == statusReceiveComplete)
	{
		if(frameBuffer.crc == calc_crc(&frameBuffer))
		{
			frameStatus = statusCrcCorrect;
			if(!frameBuffer.data)
			{
				BIT_SET(&PORTD, PD2);
				repeated_check = 1;
			} else
			{
				BIT_CLEAR(&PORTD, PD2);
				repeated_check = 0;
			}
			frameBuffer.data = ~frameBuffer.data;
			frameBuffer.function = FUNCTION_SET_PORTD;
			frameBuffer.start = START_REQ;
			send_Frame(&frameBuffer);
			frameStatus = statusFrameProcessed;
		}
	}
}

void check_occupied()
{
	unsigned int  sum = 0;
	frameBuffer.address = 0x01;
	frameBuffer.start = START_REQ;
	frameBuffer.function = FUNCTION_GET_HIGH_SENS;
	frameBuffer.data = 0x00;
	send_Frame(&frameBuffer);
	frameStatus = statusUndefined;
	setCountdown(10);
	while(getCountdown()/* && frameStatus != statusReceiveComplete*/)
	{
		dbus_receive();
	}
	if(frameStatus == statusReceiveComplete)
	{
		if(frameBuffer.crc == calc_crc(&frameBuffer))
		{
			frameStatus = statusCrcCorrect;
			sum += frameBuffer.data;
		}
	} else
	{
		BIT_SET(&PORTD, PD2);
		return;
	}
	frameBuffer.start = START_REQ;
	frameBuffer.function = FUNCTION_GET_LOW_SENS;
	frameBuffer.data = 0x00;
	send_Frame(&frameBuffer);
	frameStatus = statusUndefined;
	setCountdown(10);
	while(getCountdown()/* && frameStatus != statusReceiveComplete*/)
	{
		dbus_receive();
	}
	if(frameStatus == statusReceiveComplete)
	{
		if(frameBuffer.crc == calc_crc(&frameBuffer))
		{
			frameStatus = statusCrcCorrect;
			sum += frameBuffer.data;
		}
	} else
	{
		BIT_SET(&PORTD, PD2);
		return;
	}
	if(sum < 0xFF + 0xFF)
	{
		BIT_SET(&PORTD, PD3);
	} else
	{
		BIT_CLEAR(&PORTD, PD3);
	}
}
