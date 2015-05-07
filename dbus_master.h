/*
 * dbus.h
 *
 *  Created on: 02.04.2015
 *      Author: olli
 */

#ifndef DBUS_MASTER_H_
#define DBUS_MASTER_H_

/*
 *	Includes
 */

#include "uart.h"

/*
 *	Defines
 */

#define DBUS_BAUD 115200UL
#define DBUS_ADDRESS 0x01

//Functions
//0x00 - 0x0F System
#define FUNCTION_SET_PORTD 0x01
#define FUNCTION_GET_PORTD 0x02
#define FUNCTION_GET_LOW_SENS 0x03
#define FUNCTION_GET_HIGH_SENS 0x04

enum Status {
	//Typical States
	statusUndefined,
	statusStart,
	statusAddress,
	statusFunction,
	statusData,
	statusCrcH,
	statusCrcL,
	statusReceiveComplete,
	statusCrcCorrect,
	statusFrameProcessed,

	//Error States
	statusCrcWrong,
	statusFunctionUnknown,
};

typedef struct {
	unsigned char start;
	unsigned char address;
	unsigned char function;
	unsigned char data;
	unsigned int crc;
} frame;

/*
 *	Global Functions (Declarations, extern)
 */

extern void dbus_perform();
extern void dbus_init();
extern void dbus_receive();
extern unsigned char dbus_status();

#endif /* DBUS_MASTER_H_ */
