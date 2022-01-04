/*
 * Copyright (c) 2007 by Hartmut Birr, Thoralt Franz
 *
 * This program is free software; you can redistribute it and/or
 * mmodify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include <inttypes.h>
#include <avr/interrupt.h>
#include "dds.h"

//----------------------------------------------------------------------------
// external declaration of all timer variables
//----------------------------------------------------------------------------
extern volatile uint16_t xg_uButtonTimer;
extern volatile uint16_t xg_uMillisecondTimer;
extern volatile uint16_t xg_uScrollTimer;
extern volatile uint16_t xg_uTRMSC_Timer;
extern volatile uint16_t xg_uPermanentTRMSC_Timer;
extern volatile uint16_t xg_uSweepTimer;
extern volatile int16_t  xg_iSweepIndex;
extern volatile uint16_t xg_uBurstTimer;
extern volatile uint8_t   g_ucBurstState;
extern volatile uint16_t xg_uEEPROM_Timer;

#ifdef ENCODER_NAVIGATION
extern volatile uint16_t g_uTurnSymbolTimer;
extern volatile uint16_t g_uEncoderMenuTimer;
#endif

extern volatile float xg_dSweepFrequency;
extern volatile float  g_dLastSweepFrequency;
extern volatile float  g_dSweepFrequencyFactorUp;
extern volatile float  g_dSweepFrequencyFactorDown;

extern volatile uint32_t xg_uiSweepStartFrequency;
extern volatile uint32_t xg_uiSweepEndFrequency;
extern volatile uint32_t xg_uiSweepFrequencyIncrement;

extern volatile uint8_t g_ucSweepSync;
extern volatile float  g_dDistanceMarker;
extern volatile float  g_dFirstLogMarker;
extern volatile float  g_dLastLogMarker;
extern volatile float  g_dFirstLinMarkerLow;
extern volatile float  g_dFirstLinMarkerHigh;
extern volatile float  g_dMarkerIndex;
extern volatile uint8_t g_ucSweepMark;
extern volatile uint16_t g_uiMarkerLevel;
extern volatile uint16_t g_uiNormalLevel;

#ifdef COMPILE_WITH_PWM
extern volatile int16_t g_iPWMimpulseCount;
extern volatile uint8_t g_ucLastPulse;
#endif

//----------------------------------------------------------------------------
// prototypes
//----------------------------------------------------------------------------
void InitTimer(void);
void wait_ms(uint16_t);


// atomic access to multi-byte variables which are modified during interrupts
#define ATOMIC_RW(dst, src) {cli(); dst = src; sei();}


//----------------------------------------------------------------------------
// PWM related things
//----------------------------------------------------------------------------
#ifdef COMPILE_WITH_PWM
// TIMER1 initialization macros
#define TCCR1B_INIT ((1<<WGM13)|(1<<WGM12))
#define SET_TIMER1_PRESCALER_1()    TCCR1B = TCCR1B_INIT|(1<<CS10)
#define SET_TIMER1_PRESCALER_8()    TCCR1B = TCCR1B_INIT|(1<<CS11)
#define SET_TIMER1_PRESCALER_64()   TCCR1B = TCCR1B_INIT|(1<<CS10)|(1<<CS11)
#define SET_TIMER1_PRESCALER_256()  TCCR1B = TCCR1B_INIT|(1<<CS12)
float CalculatePWMFrequencyValue(uint16_t iTimerValue, uint16_t uPrescaler);
uint16_t CalculatePWMTimerValue(float dFrequency, uint16_t uPrescaler);
void SetPWMPrescaler(float *dFrequency, uint16_t *uPrescaler);
#endif

#endif
