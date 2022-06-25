/*
 * FreeModbus Library: AVR Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *   - Initial version + ATmega168 support
 * Modifications Copyright (C) 2006 Tran Minh Hoang:
 *   - ATmega8, ATmega16, ATmega32 support
 *   - RS485 support for DS75176
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

#ifndef _PORT_H
#define _PORT_H

/* ----------------------- Platform includes --------------------------------*/

#include <avr/io.h>
#include <avr/interrupt.h>

/* ----------------------- Defines ------------------------------------------*/
#define	INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define ENTER_CRITICAL_SECTION( )   cli()
#define EXIT_CRITICAL_SECTION( )    sei()

#define assert( x )

typedef char    BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef unsigned short USHORT;
typedef short   SHORT;

typedef unsigned long ULONG;
typedef long    LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif


/* ----------------------- AVR platform specifics ---------------------------*/

#if __AVR_ARCH__ < 100

#define UART_SET_BAUD(b)   do { UBRR = ((F_OSC) / ((b) * 16UL) - 1); } while (0)

#define USART_PMODE_EVEN_gc		_BV( UPM1 )
#define USART_PMODE_ODD_gc		(_BV( UPM1 ) | _BV( UPM0 ))
#define USART_CHSIZE_8BIT_gc	(_BV( UCSZ0 ) | _BV( UCSZ1 ))
#define USART_CHSIZE_7BIT_gc	_BV( UCSZ1 )

#endif

#if defined (__AVR_ATmega168__)
#define UCSRB           UCSR0B
#define TXEN            TXEN0
#define RXEN            RXEN0
#define RXCIE           RXCIE0
#define TXCIE           TXCIE0
#define UDRE            UDRE0
#define UBRR            UBRR0
#define UCSRC           UCSR0C
#define UPM1            UPM01
#define UPM0            UPM00
#define UCSZ0           UCSZ00
#define UCSZ1           UCSZ01
#define UDRX            UDR0
#define UDTX            UDR0
#define ISR_USART_TRAN  USART0_TXC_vect
#define ISR_USART_DATA  USART0_UDRE_vect
#define ISR_USART_RECV  USART0_RXC_vect

#elif defined (__AVR_ATmega169__)

#define ISR_USART_TRAN  USART_TXC_vect

#elif defined (__AVR_ATmega8__)
#define UBRR            UBRRL
#define TCCR1C          TCCR1A  /* dummy */
#define TIMSK1          TIMSK
#define TIFR1           TIFR
#define ISR_USART_TRAN  USART_TXC_vect
#define ISR_USART_DATA  USART_UDRE_vect
#define ISR_USART_RECV  USART_RXC_vect

#elif defined (__AVR_ATmega16__)
#define UBRR            UBRRL
#define TCCR1C          TCCR1A  /* dummy */
#define TIMSK1          TIMSK
#define TIFR1           TIFR
#define ISR_USART_TRAN  USART_TXC_vect
#define ISR_USART_DATA  USART_UDRE_vect
#define ISR_USART_RECV  USART_RXC_vect

#elif defined (__AVR_ATmega32__)
#define UBRR            UBRRL
#define TCCR1C          TCCR1A  /* dummy */
#define TIMSK1          TIMSK
#define TIFR1           TIFR
#define ISR_USART_TRAN  USART_TXC_vect
#define ISR_USART_DATA  USART_UDRE_vect
#define ISR_USART_RECV  USART_RXC_vect

#elif defined (__AVR_ATmega128__)
#define UCSRB           UCSR0B
#define UBRR            UBRR0L
#define UDRX            UDR0
#define UDTX            UDR0
#define TIMSK1          TIMSK
#define TIFR1           TIFR
#define ISR_USART_TRAN  USART0_TXC_vect
#define ISR_USART_DATA  USART0_UDRE_vect
#define ISR_USART_RECV  USART0_RXC_vect
#define UCSZ0           UCSZ00
#define UCSZ1           UCSZ01
#define UPM0            UPM00
#define UPM1            UPM01
#define UCSRC           UCSR0C

#elif defined (__AVR_ATmega328P__)
#define UCSRA           UCSR0A
#define UCSRB           UCSR0B
#define UCSRC           UCSR0C
#define UBRR            UBRR0L
#define UDRX            UDR0
#define UDTX            UDR0
#define TIMSK1          TIMSK
#define TIFR1           TIFR
#define ISR_USART_TRAN  USART0_TXC_vect
#define ISR_USART_DATA  USART0_UDRE_vect
#define ISR_USART_RECV  USART0_RXC_vect
#define UCSZ0           UCSZ00
#define UCSZ1           UCSZ01
#define UPM0            UPM00
#define UPM1            UPM01

#define RXEN			RXEN0
#define RXCIE			RXCIE0
#define UDRIE			UDRIE0
#define TXEN			TXEN0
#define TXCIE			TXCIE0

#elif defined (__AVR_AVR64DA64__)

#define AVR_DA_USART	1
#define AVR_DA_TIMER	1

/*	If not using RS485, just ignore DE */
#define PIN_RS4DI 		PIN0
#define PORT_RS4DI		PORTF
#define PIN_RS4RO 		PIN1
#define PORT_RS4RO		PORTF
#define PIN_RS4DE 		PIN3
#define PORT_RS4DE		PORTF

#define USART_PORT      USART2
#define ISR_USART_TRAN  USART2_TXC_vect
#define ISR_USART_DATA  USART2_DRE_vect
#define ISR_USART_RECV  USART2_RXC_vect

#define UDRX            USART_PORT.RXDATAL
#define UDTX            USART_PORT.TXDATAL
#define RTS_DDR         PORT_RS4DE.DIR
#define RTS_PORT        PORT_RS4DE.OUT
#define RTS_PIN         PIN_RS4DE

#define UART_SET_BAUD(b)	do { USART_PORT.BAUD = (((float)(F_CPU) * 64 / (16 * (float)(b))) + 0.5); } while (0)

#endif


/* ----------------------- RS485 specifics ----------------------------------*/

#define RTS_ENABLE      1


#ifdef  RTS_ENABLE

#ifdef AVR_DA_USART

#define RTS_INIT        do { USART_PORT.CTRLA |= USART_RS485_bm; } while( 0 )
#define RTS_HIGH
#define RTS_LOW
#define RTS_HARDWARE    1

#else

#define RTS_PIN         PB0
#define RTS_DDR         DDRB
#define RTS_PORT        PORTB

#define RTS_INIT        \
    do { \
        RTS_DDR |= _BV( RTS_PIN ); \
        RTS_PORT &= ~( _BV( RTS_PIN ) ); \
    } while( 0 )

#define RTS_HIGH        \
    do { \
        RTS_PORT |= _BV( RTS_PIN ); \
    } while( 0 )

#define RTS_LOW         \
    do { \
        RTS_PORT &= ~( _BV( RTS_PIN ) ); \
    } while( 0 )

#endif // AVR_DA_USART

#endif // RTS_ENABLE

#endif // _PORT_H
