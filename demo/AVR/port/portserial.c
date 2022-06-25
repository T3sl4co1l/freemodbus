/*
 * FreeModbus Library: ATMega168 Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *   - Initial version and ATmega168 support
 * Modifications Copyright (C) 2006 Tran Minh Hoang:
 *   - ATmega8, ATmega16, ATmega32 support
 *   - RS485 support for DS75176
 *
 * Modifications Copyright (C) 2006 Michał:
 *   - Prepared to compile with atmega328p
 *
 * Modified 2022-06-22 by Tim Williams
 *   - Migrated to AVR64DA64
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//#define PORT_TESTING

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable) {
#ifdef AVR_DA_USART

#if defined(RTS_ENABLE) && !defined(RTS_HARDWARE)
	USART_PORT.CTRLA |= USART_TXCIE_bm;
#endif
	USART_PORT.CTRLB |= USART_TXEN_bm;

	if (xRxEnable) {
		USART_PORT.CTRLB |= USART_RXEN_bm;
		USART_PORT.CTRLA |= USART_RXCIE_bm;
	} else {
		USART_PORT.CTRLB &= ~USART_RXEN_bm;
		USART_PORT.CTRLA &= ~USART_RXCIE_bm;
	}

	if (xTxEnable) {
		USART_PORT.CTRLA |= USART_DREIE_bm;
#ifdef RTS_ENABLE
		RTS_HIGH;
#endif
	} else {
		USART_PORT.CTRLA &= ~USART_DREIE_bm;
	}

#else

#ifdef RTS_ENABLE
	UCSRB |= _BV( TXEN ) | _BV(TXCIE);
#else
	UCSRB |= _BV( TXEN );
#endif

	if( xRxEnable ) {
		UCSRB |= _BV( RXEN ) | _BV( RXCIE );
	} else {
		UCSRB &= ~( _BV( RXEN ) | _BV( RXCIE ) );
	}

	if( xTxEnable ) {
		UCSRB |= _BV( TXEN ) | _BV( UDRE );
#ifdef RTS_ENABLE
		RTS_HIGH;
#endif
	} else {
		UCSRB &= ~( _BV( UDRE ) );
	}

#endif // AVR_DA_USART
}

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity) {
	UCHAR ucUCSRC = 0;

	/* prevent compiler warning. */
	(void)ucPORT;

	UART_SET_BAUD(ulBaudRate);

	switch (eParity) {
	case MB_PAR_EVEN:
		ucUCSRC = USART_PMODE_EVEN_gc;
		break;
	case MB_PAR_ODD:
		ucUCSRC = USART_PMODE_ODD_gc;
		break;
	case MB_PAR_NONE:
		break;
	}

	switch (ucDataBits) {
	case 8:
		ucUCSRC |= USART_CHSIZE_8BIT_gc;
		break;
	case 7:
		ucUCSRC |= USART_CHSIZE_7BIT_gc;
		break;
	}

#if defined (__AVR_ATmega168__)
    UCSRC |= ucUCSRC;
#elif defined (__AVR_ATmega169__)
    UCSRC |= ucUCSRC;
#elif defined (__AVR_ATmega8__)
    UCSRC = _BV( URSEL ) | ucUCSRC;
#elif defined (__AVR_ATmega16__)
    UCSRC = _BV( URSEL ) | ucUCSRC;
#elif defined (__AVR_ATmega32__)
    UCSRC = _BV( URSEL ) | ucUCSRC;
#elif defined (__AVR_ATmega128__)
    UCSRC |= ucUCSRC;
#elif defined(AVR_DA_USART)
	USART_PORT.CTRLC = (USART_PORT.CTRLC & ~(USART_CHSIZE_gm | USART_PMODE_gm)) | ucUCSRC;
#endif

	vMBPortSerialEnable(FALSE, FALSE);

#ifdef RTS_ENABLE
	RTS_INIT;
#endif
	return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte) {
	UDTX = ucByte;
	return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte) {
	*pucByte = UDRX;
	return TRUE;
}

/**
 * Interrupt generated when output buffer is empty
 * USART, UDR Empty Handle (DRE)
 */
ISR(ISR_USART_DATA) {
#ifndef PORT_TESTING
	pxMBFrameCBTransmitterEmpty();
#else
	static UCHAR c = 0;
	if (c < 10) {
		xMBPortSerialPutByte('a');
		c++;
	} else {
		vMBPortSerialEnable(FALSE, FALSE);
	}
#endif // PORT_TESTING
}

/**
 * Interrupt generated after byte received.
 * received data stored in UDR
 * USART, RX Complete Handler (RXC)
 */
ISR(ISR_USART_RECV) {
#ifndef PORT_TESTING
	pxMBFrameCBByteReceived();
#else
	CHAR b;
	xMBPortSerialGetByte(&b);
	//serPutByte(b);	//	some debug statement here (forward to another serial port? pin wiggle?)
#endif // PORT_TESTING
}

#if defined(RTS_ENABLE) && !defined(RTS_HARDWARE)
ISR(ISR_USART_TRAN) {

#ifdef AVR_DA_USART
	USART_PORT.STATUS = USART_TXCIF_bm;
#endif // AVR_DA_USART
	RTS_LOW;

}
#endif
