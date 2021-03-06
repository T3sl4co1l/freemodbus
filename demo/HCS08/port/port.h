/*
 * FreeModbus  Library:: HCS08 Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 * Copyright (C) 2008 Trond Melen
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

#include <assert.h>

/* The include file "derivative.h" is automatically generated by Code Warrior */
#include "derivative.h"

/* Tne include fila inttypes.h is not available for HCS08, so we use our own definitions instead */
//#include <inttypes.h>
typedef char        int8_t;
typedef unsigned char uint8_t;
typedef int         int16_t;
typedef unsigned int uint16_t;
typedef long        int32_t;
typedef unsigned long uint32_t;


/* Michael Barrs verification code */
#pragma MESSAGE ERROR DISABLE C1135
static union
{
    char            int8_t_incorrect[sizeof( int8_t ) == 1];
    char            uint8_t_incorrect[sizeof( uint8_t ) == 1];
    char            int16_t_incorrect[sizeof( int16_t ) == 2];
    char            uint16_t_incorrect[sizeof( uint16_t ) == 2];
    char            int32_t_incorrect[sizeof( int32_t ) == 4];
    char            uint32_t_incorrect[sizeof( uint32_t ) == 4];
};

#define	INLINE
#define PR_BEGIN_EXTERN_C       extern "C" {
#define	PR_END_EXTERN_C         }
#define	memcpy                  (void) memcpy   // gets wid of a warning in mbfuncother.c

/* Next 2 lines copied from code generated by Processor Expert */
#define SaveStatusReg()         { asm PSHA; asm TPA; asm SEI; asm STA CCR_reg; asm PULA; }  /* This macro is used by Processor Expert. It saves CCR register and disable global interrupts. */
#define RestoreStatusReg()      { asm PSHA; asm LDA CCR_reg; asm TAP; asm PULA; }   /* This macro is used by Processor Expert. It restores CCR register saved in SaveStatusReg(). */

#define ENTER_CRITICAL_SECTION( )   DisableInterrupts;
#define EXIT_CRITICAL_SECTION( )    EnableInterrupts;

typedef uint8_t                 BOOL;
typedef unsigned char           UCHAR;
typedef char                    CHAR;
typedef uint16_t                USHORT;
typedef int16_t                 SHORT;
typedef uint32_t                ULONG;
typedef int32_t                 LONG;

#ifndef TRUE
#define TRUE                    1
#endif

#ifndef FALSE
#define FALSE                   0
#endif

/* SaveStatusReg() stores the Condition Code Register here */
extern UCHAR                    CCR_reg;

/* The bus clock is used for computing timer and baud rate register values */
#define BM_BUS_CLOCK			4194304 // Hz

#endif
