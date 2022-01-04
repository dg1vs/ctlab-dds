/*
 * Copyright (c) 2008 by Thoralt Franz
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

#ifndef _DDS_HW_H_
#define _DDS_HW_H_

#include <inttypes.h>

//----------------------------------------------------------------------------
// relay names
//----------------------------------------------------------------------------
#define RELAY_ATTENUATOR    0
#define RELAY_EXTERNAL_IN   1
#define RELAY_OFFSET_NULL   2

//----------------------------------------------------------------------------
// control register bit definitions for AD9833
//----------------------------------------------------------------------------
#define AD9833_B28      (1<<13)
#define AD9833_HLB      (1<<12)
#define AD9833_FSELECT  (1<<11)
#define AD9833_PSELECT  (1<<10)
#define AD9833_RESET    (1<<8)
#define AD9833_SLEEP1   (1<<7)
#define AD9833_SLEEP2   (1<<6)
#define AD9833_OPTBITEN (1<<5)
#define AD9833_DIV2     (1<<3)
#define AD9833_MODE     (1<<1)

//----------------------------------------------------------------------------
// bit definitions for control lines
//----------------------------------------------------------------------------
#define SPI_DATA        (1<<PB1)
#define SPI_CLOCK       (1<<PB0)
#define LTC1257_STROBE  (1<<PB4)
#define SR4094_STROBE   (1<<PB3)
#define AD9833_STROBE   (1<<PB2)
#define CLK_LO()        PORTB &= ~SPI_CLOCK
#define CLK_HI()        PORTB |= SPI_CLOCK

//----------------------------------------------------------------------------
// LEDs
//----------------------------------------------------------------------------
#define ACTIVITY_LED    (1<<PD2)
#define OFFSET_LED      (1<<PD3)

//----------------------------------------------------------------------------
// offset calibration
//----------------------------------------------------------------------------
// value which represents an offset voltage of 0 V
#define OFFSET_NULL_VOLTAGE 2048

//----------------------------------------------------------------------------
// TRMSC Ports
//----------------------------------------------------------------------------
#define ucAttenuator200     (1<<PC3)
#define ucGain10            (1<<PC2)
#define ucSyncOut           (1<<PC4)

//----------------------------------------------------------------------------
// function prototypes
//----------------------------------------------------------------------------
uint16_t GetADC(uint8_t Channel);
void ShiftOut1257(uint16_t Value);
void ShiftOut4094(uint16_t Value);
void ShiftOut9833(uint16_t Value);

void SetRelay(uint8_t uRelay, uint8_t uState);
void SetAttenuation(uint16_t u);
#define SetLED(ucLED, ucState)  if(ucState) PORTD &= ~ucLED; else   PORTD |= ucLED;
void SetOffsetVoltage(int16_t iVoltage);
void SetLevel(float fLevel);

void AD9833_Init(void);
void AD9833_SetFrequency(uint32_t u);
uint8_t AD9833_SetWaveform(uint8_t uWave);
void SetGain10(uint8_t ucState);
void SetAttenuator200(uint8_t ucState);

#ifdef COMPILE_WITH_PWM
extern uint16_t g_uPWMPrescaler;
#endif

#endif
