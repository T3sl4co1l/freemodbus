/*
 * FreeModbus Library: ATMega168 Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
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

/* ----------------------- AVR includes -------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>


/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#ifdef AVR_DA_TIMER
#define MB_TIMER_PRESCALER      ( 1200UL )
#else
#define MB_TIMER_PRESCALER      ( 1024UL )
#endif // AVR_DA_TIMER
#define MB_TIMER_TICKS          ( F_CPU / MB_TIMER_PRESCALER )
#define MB_50US_TICKS           ( 20000UL )

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usTimerOCRADelta;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us) {
#ifdef AVR_DA_TIMER
	/* Calculate OCR value for Timer */
	usTimerOCRADelta = (MB_TIMER_TICKS * usTim1Timerout50us) / (MB_50US_TICKS);

    //	Not enough prescaler options available on TCB; just use TCB4 (via event ch0) to prescale TCB3 exactly.
    //	Could also count ticks in ISR (SW emulated tick counter, as it were)...
    TCB4.CTRLA = TCB_ENABLE_bm;
    TCB4.CTRLB = TCB_CCMPEN_bm;
    TCB4.EVCTRL = 0;
    TCB4.INTCTRL = 0;
    TCB4.CCMP = MB_TIMER_PRESCALER;
    EVSYS.CHANNEL0 = EVSYS_CHANNEL0_TCB4_CAPT_gc;
	EVSYS_USERTCB3COUNT = 0 + 1;
	TCB3.CTRLA = TCB_CLKSEL_EVENT_gc;

#else

	TCCR1A = 0x00;
	TCCR1B = 0x00;
	TCCR1C = 0x00;

#endif // AVR_DA_TIMER
    vMBPortTimersDisable();

    return TRUE;
}

inline void vMBPortTimersEnable() {

#ifdef AVR_DA_TIMER
	TCB3.CNT = 0;
	if (usTimerOCRADelta > 0) {
		TCB3.INTCTRL = TCB_CAPT_bm;
		TCB3.CCMP = usTimerOCRADelta;
	}
	TCB3.CTRLA = TCB_CLKSEL_EVENT_gc | TCB_ENABLE_bm;
#else
	TCNT1 = 0;
	if (usTimerOCRADelta > 0) {
		TIMSK1 |= _BV(OCIE1A);
		OCR1A = usTimerOCRADelta;
	}
	TCCR1B |= _BV(CS12) | _BV(CS10);
#endif // AVR_DA_TIMER

}

inline void vMBPortTimersDisable() {

	/* Disable the timer. */
#ifdef AVR_DA_TIMER
	TCB3.CTRLA = 0;
	TCB3.INTCTRL = 0;
#else
	TCCR1B &= ~(_BV(CS12) | _BV(CS10));
	/* Disable the output compare interrupts for channel A/B. */
	TIMSK1 &= ~(_BV(OCIE1A));
	/* Clear output compare flags for channel A/B. */
	TIFR1 |= _BV(OCF1A);
#endif // AVR_DA_TIMER

}

ISR(TCB3_INT_vect) {

#ifdef AVR_DA_TIMER
	TCB3.INTFLAGS = TCB_OVF_bm | TCB_CAPT_bm;	//	clear both just in case
#endif // AVR_DA_TIMER
	(void)pxMBPortCBTimerExpired();

}
