/*
 * Copyright (c) 2007 by Thoralt Franz, Joerg Wilke
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

//============================================================================
// IO MAP
//
// PA0 ADC0 (panel)
// PA1 ADC1 (panel)
// PA2 ADC2 (extension)
// PA3 ADC3 (extension)
// PA4 --
// PA5 --
// PA6 --
// PA7 --
//
// PB0 SCLK for U3, U4 (2x4094), U7 (AD9833), U11 (LTC1257)
// PB1 SDATA for U3, U4 (2x4094), U7 (AD9833), U11 (LTC1257)
// PB2 FSYNC for U7 (AD9833, DDS)
// PB3 STROBE for U3, U4 (double shift register 4094)
// PB4 STRDAC for U11 (LTC1257, offset D/A)
// PB5 MOSI (ISP)
// PB6 MISO (ISP)
// PB7 SCK (ISP)
//
// PC0 I2C SCL (extension, panel)
// PC1 I2C SDA (extension, panel)
// PC2 Gain10  (extension)
// PC3 Attentuator200 (extension)
// PC4 SyncOut (extension)
// PC5 (extension)
// PC6 --
// PC7 --
//
// PD0 OptoBus Rx
// PD1 OptoBus Tx
// PD2 INT0 activity LED (panel)
// PD3 INT1 offset LED (panel)
// PD4 PWM Output
// PD5 OptoBusAdr bit 0
// PD6 OptoBusAdr bit 1
// PD7 OptoBusAdr bit 2
//
//============================================================================

//   avr-gcc -mmcu=atmega32 -I. -gdwarf-2 -DF_CPU=16000000UL -I /usr/local/AVRMacPack/include -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wextra -Werror -Wstrict-prototypes -Wa,-adhlns=uart.lst  -std=gnu99 -fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop -ffreestanding -ffunction-sections -fdata-sections -mcall-prologues --combine -fwhole-program -Wl,--relax,--gc-sections,-Map=dds.map,--cref,-u,vfprintf -MD -MP parser.c timer.c dds-hw.c dds-parser.c encoder.c i2c.c i2creg.c lcd.c panel.c main.c uart.c --output dds.elf -lprintf_flt -lm

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>

#include "main.h"
#include "dds-hw.h"

#include "I2CRegister.h"
#include "Lcd.h"
#include "Encoder.h"
#include "Uart.h"

#include "panel.h"
#include "parser.h"
#include "timer.h"
#include "dds.h"
#include "dds-hw.h"

STATUS Status;

PARAMS Params;
PARAMS *pParams = &Params;

PARAMS LastParams;
PARAMS *pLastParams = &LastParams;

uint8_t g_ucSlaveCh;
uint8_t g_ucErrCount;
uint8_t ActivityTimer;
uint8_t g_ucLCDpresent;

#ifdef COMPILE_WITH_PWM // hardware modification required

    const char g_cVersStrLong[] = VERSSTRLONG_PWM ;
    const PROGMEM char g_cVersStrShort_P[] = VERSSTRSHORT_PWM ;

#else                   // standard hardware

    const char g_cVersStrLong[] = VERSSTRLONG_STD ;
    const PROGMEM char g_cVersStrShort_P[] = VERSSTRSHORT_STD ;

#endif


const PROGMEM uint16_t g_uiTerzArray[] =
{
// code     frequency (Hz)
    /*
      0x0640,   //1
      0x07D0,   //1.25
      0x0A00,   //1.6
      0x0C80,   //2.0
      0x0FA0,   //2.5
      0x13B0,   //3.15
      0x1900,   //4.0
      0x1F40,   //5.0
      0x2760,   //6.3
      0x3200,   //8.0
    */
    0x0641,   //10
    0x07D1,   //12.5
    0x0A01,   //16
    0x0C81,   //20
    0x0FA1,   //25
    0x13B1,   //31.5
    0x1901,   //40
    0x1F41,   //50
    0x2761,   //63
    0x3201,   //80

    0x0642,   //100
    0x07D2,   //125
    0x0A02,   //160
    0x0C82,   //200
    0x0FA2,   //250
    0x13B2,   //315
    0x1902,   //400
    0x1F42,   //500
    0x2762,   //630
    0x3202,   //800

    0x0643,   //1000
    0x07D3,   //1250
    0x0A03,   //1600
    0x0C83,   //2000
    0x0FA3,   //2500
    0x13B3,   //3150
    0x1903,   //4000
    0x1F43,   //5000
    0x2763,   //6300
    0x3203,   //8000

    0x0644,   //10000
    0x07D4,   //12500
    0x0A04,   //16000
    0x0C84,   //20000
    0x0FA4,   //25000

    /*
      0x13B4,   //31500
      0x1904,   //40000
      0x1F44,   //50000
      0x2764,   //63000
      0x3204,   //80000

      0x0645,   //100000
    */
    0
};

// Coding for frequencies: (ui_TerzArray[x]>>4 ) * 10 ^ ( int16_t)(ui_TerzArray[x] & 0x000F ) - 2)
// 3 MSB nibbles: frequency f
// LSB nibble: exponent e  (with offset 2)
// frequency = f*10^(e-2)


// variables to hold the TRMSC measurement values
float g_dTRMSC_RMS;
float g_dTRMSC_Peak;
float g_dLastTRMSC_RMS;
float g_dLastTRMSC_Peak;

// buffer for averaging the input values
#define TRMSC_BUFFER_SIZE 4
float g_dTRMSC_RMS_Buffer[TRMSC_BUFFER_SIZE];
float g_dTRMSC_Peak_Buffer[TRMSC_BUFFER_SIZE];

// TRMSC auto range setting
uint8_t g_ucTRMSC_Range = 0, g_ucLastTRMSC_Range = -1;

// modified Params structure flag
uint8_t g_ucModifiedParams = 0;

//----------------------------------------------------------------------------
// uart_putchar
//
// write one character to the UART, wait if UART buffer is full
// '\n' is replaced by '\r'+'\n'
//
// -> c = character to send
//    stream = open file (unused, only for compatibility with I/O streams
//             and printf)
// <- 0
//----------------------------------------------------------------------------
int uart_putchar(char c, FILE* stream __attribute__((unused)))
{
    if (c == '\n')
    {
        c = '\r';
        while (1 != Uart_SetTxData((uint8_t*)&c, 1, 0));
        c = '\n';
    }
    while(1 != Uart_SetTxData((uint8_t*)&c, 1, 0));
    return 0;
}

// file pointer for UART output (used by printf)
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//----------------------------------------------------------------------------
// InitIO
//
// Initialize all static IO lines
//
// -> --
// <- --
//----------------------------------------------------------------------------
void InitIO(void)
{
    // these settings only address the current debug setup, this is not final!

    DDRA = 0x00;

    // SCLK, SDATA, FSYNC, STROBE, STRDAC = output: 0b00011111
    DDRB = 0x1F;

    // all strobe lines high (inactive)
    PORTB = AD9833_STROBE | SR4094_STROBE | LTC1257_STROBE;

    DDRC = 0x3C; // PC4 and PC5 output for sync and debug marker
#ifdef COMPILE_WITH_PWM
#ifdef COMPILE_WITH_PWM_OC1A // TODO: auto detection of PWM on PD4/PD5
    DDRD = 0x3C; // temporarily both PD5+PD4 / OA1A+OC1B
#else
    DDRD = 0x1C; // PD4 / OC1B only
#endif

#else
    DDRD = 0x0C;
#endif
    PORTD = 0xFC;
}

void SetActivityTimer(uint8_t Value)
{
    if (Value > ActivityTimer)
    {
        ActivityTimer = Value;
        SetLED(ACTIVITY_LED, 1);             // Activity LED on
    }
}

void jobActivityTimer(void)
{
    if (ActivityTimer)
    {
        ActivityTimer--;
        if (ActivityTimer == 0)
        {
            SetLED(ACTIVITY_LED,0);          // Activity LED off
        }
    }
}

//----------------------------------------------------------------------------
// Round5
//
// Rounds a value to a multiple of 5 mV
//
// -> --
// <- --
//----------------------------------------------------------------------------
//float Round5(float d)
//{
//  int16_t i;
//
//  i = d * 1000.0f;
//  if(i<0) i--; else i++;
//  i /= 5; i *= 5;
//  d = i;
//  d /= 1000.0f;
//
//  return d;
//}

//----------------------------------------------------------------------------
// Support Functions for CheckLimits
//
// This function checks all members of Sweep Params against their limits and
// corrects them if necessary.
//
// -> --
// <- --
//----------------------------------------------------------------------------
void CrossCalcSweep(uint8_t mode)
{
    if (STARTEND2CENTER == mode)
    {
        if (pParams->ucSweepMode == SWEEP_LOG)
        {
            pParams->dSweepSpanFactor = powf( (pParams->dSweepEnd / pParams->dSweepStart), 0.5f);
            pParams->dSweepCenter = pParams->dSweepStart * pParams->dSweepSpanFactor;
        }
        else
        {
            pParams->dSweepCenter = (pParams->dSweepStart + pParams->dSweepEnd) / 2.0f;
            pParams->dSweepSpanFactor = pParams->dSweepEnd / pParams->dSweepCenter;
        }
    }
    else
    {
        if (pParams->ucSweepMode == SWEEP_LOG)
        {
            pParams->dSweepStart = pParams->dSweepCenter / pParams->dSweepSpanFactor;
            pParams->dSweepEnd   = pParams->dSweepCenter * pParams->dSweepSpanFactor;
        }
        else
        {
            pParams->dSweepEnd   = pParams->dSweepCenter * pParams->dSweepSpanFactor;
            pParams->dSweepStart = 2.0f * pParams->dSweepCenter - pParams->dSweepEnd;
        }
    }
}


void CrossCalcLevel(uint8_t mode)
{
    float dFactor;

    switch (pParams->ucWaveForm)    // 2* CrestFactor
    {
        case WAVE_TRIANGLE:
            dFactor = 3.46410162f;  // 2 * sqrt(3)
            break;
        case WAVE_SQUARE:
            dFactor = 2.0f;         // 2 * 1
            break;
        case WAVE_SINE:
        case WAVE_LOGIC:
#ifdef COMPILE_WITH_PWM
        case WAVE_PWM:
#endif
        default:
            dFactor = 2.82842712f;  // 2 * sqrt(2)
            break;
    }

    if (LEVEL2PEAKLEVEL == mode)
    {
        pParams->dPeakLevel = pParams->dLevel * dFactor ;
    }
    else
    {
        pParams->dLevel = pParams->dPeakLevel / dFactor ;
    }
}




//----------------------------------------------------------------------------
// CalculateSweepParameters
//
// Calculates the increment value for sweep mode (so it doesn't need to be
// calculated inside the timer interrupt)
//
// -> --
// <- --
//----------------------------------------------------------------------------
void CalculateSweepParameters(void)
{
    float d_Exponent;

#define DECADEOCTAVEFACTOR 3.32192809489f
    float dOne = 1.0f;
    float dCountMarker = 0.0f;
    float dFirstMarker;
    float dLastMarker;


//  Start and End frequencies
    xg_uiSweepStartFrequency = (unsigned long) (pParams->dSweepStart * 64.0f);
    xg_uiSweepEndFrequency = (unsigned long) (pParams->dSweepEnd * 64.0f);


//  for Linear Sweep
    xg_uiSweepFrequencyIncrement = (unsigned long) ((pParams->dSweepEnd - pParams->dSweepStart) / ( (float)pParams->iSweepTime - (float)SWEEP_UPDATE_INTERVAL ) * 64.0f);


//  for Log Sweep
    d_Exponent = (float)SWEEP_UPDATE_INTERVAL / ( (float)pParams->iSweepTime - (float)SWEEP_UPDATE_INTERVAL );

    g_dSweepFrequencyFactorUp =     powf( (float)pParams->dSweepEnd / (float)pParams->dSweepStart, d_Exponent );
    g_dSweepFrequencyFactorDown =   powf( (float)pParams->dSweepStart / (float)pParams->dSweepEnd, d_Exponent );


    //  for Markers on Lin+Log Sweep

    if (pParams->ucSweepMarker != MARKER_OFF)
    {
        float dFactor1 = ( (pParams->ucSweepMarker==MARKER_OCTAVE) ? (float)DECADEOCTAVEFACTOR : (float)1.0f );

        dCountMarker = log10((float)pParams->dSweepEnd / (float)pParams->dSweepStart) * dFactor1;
        g_dDistanceMarker = ( (float)pParams->iSweepTime - (float)SWEEP_UPDATE_INTERVAL ) / dCountMarker;

        // absolute Marker Mode - Markers are set to  10^x Hz for Decade and 2^x * 440Hz for Octave
        // relative Marker Mode - Markers are set to  2^x or 10^x *fCenter for Log and 2^x or 10^x *fStart for Linear

        float dFactor2 = ((pParams->ucSweepMarkerMode == MARKER_ABSOLUTE) ?
                           ( (pParams->ucSweepMarker==MARKER_OCTAVE) ? (float)6.875f : (float)1.0f ) :
                               ( (pParams->ucSweepMode==SWEEP_LINEAR)    ? (float)pParams->dSweepStart : (float)pParams->dSweepCenter ));



        dFirstMarker = modff( 100.0f - log10( (float)pParams->dSweepStart / dFactor2 ) * dFactor1 , &dOne );
        dLastMarker = dFirstMarker + (uint16_t)dCountMarker ;


        g_dFirstLogMarker = dFirstMarker * g_dDistanceMarker;
        g_dLastLogMarker  = dLastMarker  * g_dDistanceMarker + (float)SWEEP_UPDATE_INTERVAL ;


        float dFactor3 = ((pParams->ucSweepMarker==MARKER_OCTAVE) ? (float)2.0f : (float)10.0f );

        g_dFirstLinMarkerLow  = powf( dFactor3, dFirstMarker ) * xg_uiSweepStartFrequency;
        g_dFirstLinMarkerHigh = powf( dFactor3, dLastMarker  ) * xg_uiSweepStartFrequency;



        float dSwitchPoint;
        float fLevel;

        // 4000 = used DAC counts, 8.0 (0.2) = scale end
        // value, 2.0f = amplification of output buffer
        dSwitchPoint = 4000.0f * pParams->dOutGain / pParams->dAttnFactor;

        fLevel = pParams->dLevel;

        switch (pParams->ucWaveForm)
    {
            case WAVE_TRIANGLE:
                fLevel *= 1.224745;
                break;
            case WAVE_SQUARE:
                fLevel *= 0.70711;
                break;
            case WAVE_LOGIC:
#ifdef COMPILE_WITH_PWM
            case WAVE_PWM:
#endif
                // no sweep markers in logic mode
                break;
        }

        // check against maximum value
        if (fLevel>8191.0) fLevel = 8191.0;


        // pre-calculate normal signal level based on switch position of passive attenuator

        if( fLevel <= dSwitchPoint)
        {
            g_uiNormalLevel = (int)(fLevel * 4000.0f * pParams->dLevelScaleLow / dSwitchPoint );
        }
        else
        {
            g_uiNormalLevel = (int)(fLevel * pParams->dLevelScaleHigh / pParams->dOutGain);
        }

        // pre-calculate marker level and limit this level, if necessary
        g_uiMarkerLevel = (int)( g_uiNormalLevel * pParams->dSweepMarkerHeight);

        if (g_uiMarkerLevel >= 0x0FFF)
        {
            g_uiMarkerLevel = 0x0FFF;
        }
    }

}

void LIMIT_DOUBLE(float *param, float min, float max)
{
    if (*param > max)
        *param = max;
    else if (*param < min)
        *param = min;
}

void LIMIT_UINT8(uint8_t *param, uint8_t min, uint8_t max)
{
    if (*param > max)
        *param = max;
    else if (*param < min)
        *param = min;
}

void LIMIT_INT16(int16_t *param, int16_t min, int16_t max)
{
    if (*param > max)
        *param = max;
    else if (*param < min)
        *param = min;
}

//----------------------------------------------------------------------------
// CheckLimits
//
// This function checks all members of Params against their limits and
// corrects them if necessary.
//
// -> --
// <- --
//----------------------------------------------------------------------------
void CheckLimits(void)
{
    /*
    #define LIMIT_INT16(param, min, max) { \
        if(param>max) param=max; \
        else if(param<min) param=min; \
    }

    #define LIMIT_UINT8(param, min, max) { \
        if((int8_t)param>max) param=max; \
        else if((int8_t)param<min) param=min; \
    }
    */

    LIMIT_DOUBLE(&Params.dFrequency, 0.015625, MAX_FREQUENCY);
    LIMIT_DOUBLE(&Params.dLevel, 0.025f, 8191.0f);
    LIMIT_INT16(&Params.iOffset, -10000, 10000);

    // Make sure Offset can be divided by 5 mV
    Params.iOffset -= Params.iOffset % 5;

    LIMIT_DOUBLE(&Params.dSweepStart, 0.015625, MAX_FREQUENCY);
    LIMIT_DOUBLE(&Params.dSweepEnd, 0.015625, MAX_FREQUENCY);

    // check sweep start against sweep end (only if changed)
    // and transfer values to other pair of settings
    // use this point to force re-calculation in case mode has switched from lin to log or vice versa in order to change the CenterSpan values.
    if((pParams->dSweepStart!=LastParams.dSweepStart) || (pParams->ucSweepMode!=LastParams.ucSweepMode) )
    {
        if(pParams->dSweepStart>pParams->dSweepEnd)
            pParams->dSweepStart = pParams->dSweepEnd;

        CrossCalcSweep(STARTEND2CENTER);
    }
    else
    {
        // check sweep end against sweep start (only if changed)
        // and transfer values to other pair of settings
        if(pParams->dSweepEnd!=LastParams.dSweepEnd)
        {
            if(pParams->dSweepEnd<pParams->dSweepStart)
                pParams->dSweepEnd = pParams->dSweepStart;

            CrossCalcSweep(STARTEND2CENTER);
        }
        else
        {
            LIMIT_DOUBLE(&Params.dSweepCenter, 0.015625, MAX_FREQUENCY);
            LIMIT_DOUBLE(&Params.dSweepSpanFactor, 1.0f,1000.0f);

            // check sweep end and sweep start by modification of sweep center (only if changed)
            // and transfer values to other pair of settings
            if(pParams->dSweepCenter!=LastParams.dSweepCenter)
            {
                if( (pParams->dSweepCenter * pParams->dSweepSpanFactor) > MAX_FREQUENCY)
                    pParams->dSweepCenter = MAX_FREQUENCY / pParams->dSweepSpanFactor;

                if (pParams->ucSweepMode == SWEEP_LOG)
                {
                    if( (pParams->dSweepCenter / pParams->dSweepSpanFactor) < 0.015625)
                        pParams->dSweepCenter = 0.015625 * pParams->dSweepSpanFactor;
                }
                else
                {
                    if( ((2.0f - pParams->dSweepSpanFactor) * pParams->dSweepCenter) < 0.015625)
                        pParams->dSweepCenter = 0.015625 * (2.0f - pParams->dSweepSpanFactor);
                }

                CrossCalcSweep(CENTER2STARTEND);
            }
            else
                // check sweep end and sweep start by modification of sweep span factor (only if changed)
                // and transfer values to other pair of settings
                if(pParams->dSweepSpanFactor!=LastParams.dSweepSpanFactor)
                {
                    if( (pParams->dSweepCenter * pParams->dSweepSpanFactor) > MAX_FREQUENCY)
                        pParams->dSweepSpanFactor = MAX_FREQUENCY / pParams->dSweepCenter;

                    if (pParams->ucSweepMode == SWEEP_LOG)
                    {
                        if( (pParams->dSweepCenter / pParams->dSweepSpanFactor) < 0.015625)
                            pParams->dSweepSpanFactor = pParams->dSweepCenter / 0.015625;
                    }
                    else
                    {
                        if( ((2.0f - pParams->dSweepSpanFactor) * pParams->dSweepCenter) < 0.015625)
                            pParams->dSweepSpanFactor = 2.0f - (0.015625 / pParams->dSweepCenter);
                    }

                    CrossCalcSweep(CENTER2STARTEND);
                }
        }
    }


    LIMIT_INT16(&Params.iSweepTime, 100, 30000);

    // check LogicHi against LogicLo (only if changed)
    if((pParams->iLogicHi!=LastParams.iLogicHi)&&
            (pParams->iLogicHi<=pParams->iLogicLo))
        pParams->iLogicHi = pParams->iLogicLo + 5;

    // check LogicLo against LogicHi (only if changed)
    if((pParams->iLogicLo!=LastParams.iLogicLo)&&
            (pParams->iLogicLo>=pParams->iLogicHi))
        pParams->iLogicLo = pParams->iLogicHi - 5;

    LIMIT_INT16(&Params.iLogicHi, -9995, 10000);
    LIMIT_INT16(&Params.iLogicLo, -10000, 9995);

    // Make sure LogicHi and LogicLo can be divided by 5 mV
    pParams->iLogicHi -= pParams->iLogicHi % 5;
    pParams->iLogicLo -= pParams->iLogicLo % 5;

    LIMIT_UINT8(&Params.ucWaveForm, 0, 5);
    LIMIT_INT16(&Params.iBurst1, 0, 30000);     // ms
    LIMIT_INT16(&Params.iBurst0, 0, 30000);     // ms
    LIMIT_INT16(&Params.iBurstOnOff, 0, 3000);  // ms*10
    LIMIT_UINT8(&Params.ucRange, 0, 4);
    LIMIT_UINT8(&Params.ucPermanentTRMSC, 0, 2);
    LIMIT_INT16(&Params.iPermanentTRMSC_Delay, 500, 5000);
    LIMIT_INT16(&Params.iScrollSpeed, 100, 1500);
    LIMIT_UINT8(&Params.ucSweepMenu, 0, 1);
    LIMIT_UINT8(&Params.ucSweepMode, 0, 1);
    LIMIT_UINT8(&Params.ucSweepSlope, 0, 2);
    LIMIT_UINT8(&Params.ucSweepMarker, 0, 2);
    LIMIT_DOUBLE(&Params.dSweepMarkerHeight, 0.50f, 2.0f);
    LIMIT_UINT8(&Params.ucSweepMarkerMode, 0, 1);

    LIMIT_UINT8(&Params.ucVolt_dB, 0, 1);
    LIMIT_UINT8(&Params.ucAutosave, 0, 1);
    LIMIT_UINT8(&Params.ucEncoderPrescaler, 1, 32);
    //  LIMIT_UINT8(&Params.ucSerBaudReg, x, x);
    //  LIMIT_DOUBLE(&Params.dInputGainFactor, x, x);
    //  LIMIT_UINT8(&Params.ucDisplayedMenu, x, x);
    LIMIT_DOUBLE(&Params.dPeakLevel, 0.071f, 23168.f);
    LIMIT_DOUBLE(&Params.dBULevel, -89.82, 20.49);

    LIMIT_INT16(&Params.iPWMDuty, 1, 999);
    LIMIT_INT16(&Params.iPWMimpulses, 0, 30000);
    LIMIT_UINT8(&Params.ucPWMpolarity, 0, 1);
}



//----------------------------------------------------------------------------
// main
//
// This is the main function.
//
// -> --
// <- never returns
//----------------------------------------------------------------------------
int __attribute__((OS_main)) main(void)
{
    uint8_t i;

    pParams = &Params;

    // enable interrupts
    sei();

    // set up IO lines
    InitIO();

    // initialize timer
    InitTimer();

    // wait for capacitors at MAX232 to charge
    wait_ms(20);

    g_ucSlaveCh = ((uint8_t)~PIND) >> 5;
    g_ucErrCount = 0;

    // init I2C
    I2C_Init();

    // Init LCD
    g_ucLCDpresent = 0;
    if(Lcd_Init())
    {
        g_ucLCDpresent = 1;
        Panel_SplashScreen();
    }

    SetLED(ACTIVITY_LED, 1);// Activity LED on
    wait_ms(3000);


#ifdef COMPILE_WITH_DISPLAY204
    /* Clear display and write softkey-line*/
    Lcd_Write_P(0, 0, 20, ucWhites);
    Lcd_Write_P(0, 1, 20, ucWhites);
    Lcd_Write_P(0, 2, 20, ucWhites);

    // Attention 3 string as 4th parameter!
    Lcd_Write(0, 3, 20, EXTRA_EA_DIP_SIGN_DOWN "   K2        K1   "EXTRA_EA_DIP_SIGN_UP);
#else
//    LCDOverwrite_P(2, 0, 3, PSTR("RC0")); // can be used to show "beta versions" etc.
#endif

    SetLED(ACTIVITY_LED, 0);// Activity LED off
    wait_ms(500);

    // Display Adress of card with blinking LED
    for (i = 0; i < g_ucSlaveCh; i++)
    {
        SetLED(ACTIVITY_LED, 1);// Activity LED on
        wait_ms(100);
        SetLED(ACTIVITY_LED, 0);// Activity LED off
        wait_ms(250);
    }

    // initialize DDS chip
    AD9833_Init();

    // initialize relays
    SetRelay(0, 0); // passive attenuator
    SetRelay(1, 0); // external in
    SetRelay(2, 1); // offset null

    // initialize parameters, load from EEPROM if it contains valid data,
    // write default data if not
    initParams();


    if(0xaa55 == eeprom_read_word(EEPROM_VALID))
    {
        eeprom_read_block(&Params, EEPROM_PARAMS, sizeof(Params));
    }
    else
    {
        eeprom_write_block(&Params, EEPROM_PARAMS, sizeof(Params));
        eeprom_write_word(EEPROM_VALID, 0xaa55);
    }

    // init UART and set stream pointer (for printf)
    Uart_InitUBRR(pParams->ucSerBaudReg);
    stdout = &mystdout;

    printf_P(PSTR("#%d:254=%s\n"), g_ucSlaveCh, (char*)g_cVersStrLong );

#ifdef COMPILE_WITH_DISPLAY204
    // wegen statusline
    // TODO Review notwendig
    if (g_ucLCDpresent)
    {
        jobPanel();
    }
#endif

    jobExecute();

    set_sleep_mode(SLEEP_MODE_IDLE);

    while(1)
    {
        JobLoop();
    }

}

void initParams()
{
    memset(&Params, 0x0, sizeof(PARAMS));
    pParams->dFrequency            = 1000.0f;
    pParams->dLevel                = 775.0f;
    pParams->ucWaveForm            = 1;
    pParams->iOffset               = 0;
    pParams->ucRange               = RANGE_AUTO;
    pParams->iBurst0               = 1000;
    pParams->iBurst1               = 10;
//  pParams->iBurstOnOff           = 0;
    pParams->iLogicHi              = 5000;
    pParams->iLogicLo              = 0;
    pParams->ucPermanentTRMSC      = PERMANENT_TRMSC_OFF;
    pParams->ucSweepMenu           = SWEEP_OFF;
    pParams->dSweepStart           = 1000.0f;
    pParams->dSweepEnd             = 3000.0f;
    pParams->dSweepCenter            = 1000.0f;
    pParams->dSweepSpanFactor        = 10.0f;
    pParams->iSweepTime            = 1000;
    pParams->ucSweepMode           = SWEEP_LOG;
    pParams->ucSweepSlope          = SWEEP_UP;
    pParams->ucSweepMarker         = MARKER_OFF;
    pParams->dSweepMarkerHeight  = 1.2f;
    pParams->ucSweepMarkerMode   = MARKER_ABSOLUTE;
//  pParams->ucVolt_dB             = 0;
//  pParams->ucAutosave            = 0;
    pParams->iPermanentTRMSC_Delay = 2000;
    pParams->iScrollSpeed          = 500;
    pParams->ucEncoderPrescaler    = 4;
    pParams->dLevelScaleLow        = 1.0f; //* Level correction  <200 mV
    pParams->dLevelScaleHigh       = 1.0f; //* Level correction  >=200 mV
    pParams->dRMSScale100m         = 1.0f; //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    pParams->dRMSScale1            = 1.0f; //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    pParams->dRMSScale10           = 1.0f; //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    pParams->dRMSScale100          = 1.0f; //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    pParams->dOutGain              = 2.0f; //* Output Gain
    pParams->dAttnFactor           = 40.0f; //* Attenuation 1/40
    pParams->ucDisplayedMenu        = 1;
    pParams->ucSerBaudReg           = INIT_UBRR % 256;
    pParams->iPWMDuty               = 150;
    pParams->iPWMimpulses           = 0;
    pParams->ucPWMpolarity          = 0;

    // set parameters at once
    memset(&LastParams, 0xFF, sizeof(PARAMS));

    CrossCalcLevel(LEVEL2PEAKLEVEL);
    CalculateSweepParameters();

};


void RestoreAmplitude(void)
{
#ifdef COMPILE_WITH_PWM
    if ( (WAVE_LOGIC!=pParams->ucWaveForm) && (WAVE_PWM!=pParams->ucWaveForm) )
#else
    if   (WAVE_LOGIC!=pParams->ucWaveForm)
#endif
    {
        SetLevel(pParams->dLevel*0.001f);
    }
}


void JobLoop()
{
    int8_t i;
    uint16_t u;
    float d;
    static uint8_t ucSwitchDelay = 0;

    // approx. 280 us cycle time when idle
    ATOMIC_RW(u, xg_uTRMSC_Timer);
    if(0==u)
    {
        for(i=TRMSC_BUFFER_SIZE-2; i>=0; i--)
        {
            g_dTRMSC_RMS_Buffer[i+1] = g_dTRMSC_RMS_Buffer[i];
            g_dTRMSC_Peak_Buffer[i+1] = g_dTRMSC_Peak_Buffer[i];
        }
        g_dTRMSC_RMS_Buffer[0] = GetTRMSC_RMS();
        g_dTRMSC_Peak_Buffer[0] = GetTRMSC_Peak();

        d = 0;
        for(i=0; i<TRMSC_BUFFER_SIZE; i++) d+=g_dTRMSC_RMS_Buffer[i];
        g_dTRMSC_RMS = d / (float)TRMSC_BUFFER_SIZE;
        d = 0;
        for(i=0; i<TRMSC_BUFFER_SIZE; i++) d+=g_dTRMSC_Peak_Buffer[i];
        g_dTRMSC_Peak = d / (float)TRMSC_BUFFER_SIZE;

        // auto range function
        if(RANGE_AUTO == Params.ucRange)
        {
#define currentRMS g_dTRMSC_RMS_Buffer[0]

//printf_P(PSTR("#%d:Range=%u, Value=%f\n"), g_ucSlaveCh, g_ucTRMSC_Range, currentRMS);

            if (ucSwitchDelay)
            {
                ucSwitchDelay--;
            }
            else
            {
                switch(g_ucTRMSC_Range)
                {
                    case RANGE_100mV:
                        if(currentRMS==-9999.0f) g_ucTRMSC_Range = RANGE_1V;
                        break;
                    case RANGE_1V:
                        if(currentRMS==-9999.0f) g_ucTRMSC_Range = RANGE_10V;
                        else if(currentRMS<90.0f) g_ucTRMSC_Range = RANGE_100mV;
                        break;
                    case RANGE_10V:
                        if(currentRMS==-9999.0f) g_ucTRMSC_Range = RANGE_100V;
                        else if(currentRMS<900.0f) g_ucTRMSC_Range = RANGE_1V;
                        break;
                    case RANGE_100V:
                        if(currentRMS==-9999.0f) break;
                        if(currentRMS<9000.0f) g_ucTRMSC_Range = RANGE_10V;
                        break;
                    default:
                        g_ucTRMSC_Range = RANGE_100V;
                }

                SwitchRange(g_ucTRMSC_Range);

                // if range setting did change, set new range and let the new range settle for some measurements
                if(g_ucTRMSC_Range!=g_ucLastTRMSC_Range)
                {
                    g_ucLastTRMSC_Range = g_ucTRMSC_Range;
                    ucSwitchDelay = 4;
                }
            }
        }

        // allow next measurement in 125 ms
        ATOMIC_RW(xg_uTRMSC_Timer, 250);
    }

    if (g_ucLCDpresent)
    {
        jobPanel();
    };

    jobParseData();
    jobExecute();

    // if parameters were modified and EEPROM timer is zero and
    // autosave is enabled, write parameter to EEPROM
    ATOMIC_RW(u, xg_uEEPROM_Timer);
    if(g_ucModifiedParams&&Params.ucAutosave&&(0==u))
    {
        eeprom_write_block(&Params, EEPROM_PARAMS, sizeof(Params));
        g_ucModifiedParams = 0;
    }
    jobActivityTimer();
}




void jobExecute()
{
    int16_t h, l;
    void (*pUpdateDisplayFunct)(uint8_t, void*);

    // one global function to process all limits
    CheckLimits();

    if (LastParams.ucDisplayedMenu != pParams->ucDisplayedMenu)
    {
        if (findMenuEntry(pParams->ucDisplayedMenu) != g_currentMenuItem)
        {
            g_currentMenuItem = findMenuEntry(pParams->ucDisplayedMenu);
            g_ucMenuItemChanged = 1;
        };
    };

    if (pParams->iSweepTime > 0)
    {
        // Make sure SweepTime can be divided by SWEEP_UPDATE_INTERVAL
        if((pParams->iSweepTime % SWEEP_UPDATE_INTERVAL) != 0)
        {
            pParams->iSweepTime = pParams->iSweepTime / SWEEP_UPDATE_INTERVAL
                                  * SWEEP_UPDATE_INTERVAL;
        }
    }

    if (((LastParams.dFrequency != pParams->dFrequency)||
            (LastParams.iPWMDuty!=pParams->iPWMDuty))
            &&(pParams->ucSweepMode == SWEEP_OFF))
    {

        if(pParams->iBurstOnOff)
        {
            AD9833_SetWaveform(pParams->ucWaveForm);
            g_ucBurstState = 0;
            ATOMIC_RW(xg_uBurstTimer, 0);
        }
        AD9833_SetFrequency(64.0f * pParams->dFrequency);

    }

    // waveform changed?
    if (LastParams.ucWaveForm != pParams->ucWaveForm)
    {
        // if burst is active, reset burst timer and state before
        // changing waveform
        if (pParams->iBurstOnOff)
        {
            g_ucBurstState = 0;
            ATOMIC_RW(xg_uBurstTimer, 0);
        }

        // stop any PWM burst that may be going on
        if (Params.iPWMimpulses != 0)
        {
 #ifdef COMPILE_WITH_PWM
           ATOMIC_RW(g_iPWMimpulseCount, 0);
#endif           
		   PORTC &= ~ucSyncOut;
        }

        // set new waveform
        AD9833_SetWaveform(Params.ucWaveForm);

        // force RMS value calculation after switching waveform
        CrossCalcLevel(PEAKLEVEL2LEVEL);

        // reset offset, level and logic levels to
        // force an update after waveform change

        LastParams.iLogicLo = -32000;
        LastParams.iLogicHi = -32000;
        LastParams.dLevel = -1;
        LastParams.dPeakLevel = -1;
        LastParams.iOffset = -32000;
    }

    // if waveform = logic or PWM, set offset and level to form a square wave
    // with parameters LogicHi and LogicLo
#ifdef COMPILE_WITH_PWM
    if ( (WAVE_LOGIC==pParams->ucWaveForm) || (WAVE_PWM==pParams->ucWaveForm) )
#else
    if   (WAVE_LOGIC==pParams->ucWaveForm)
#endif
    {
        if ((LastParams.iLogicHi != pParams->iLogicHi) ||
                (LastParams.iLogicLo != pParams->iLogicLo) )
        {
            h = pParams->iLogicHi;
            l = pParams->iLogicLo;

            SetOffsetVoltage((l + h) >> 1);                         // calibration done in HW for LOGIC and PWM
            SetLevel((float)(h-l)/1000.0f);
        }
    }
    // every other waveform: set normal offset and level
    else
    {
        // did offset change?
        if (LastParams.iOffset != pParams->iOffset)
        {
            // Make sure Offset can be divided by 5 mV
            pParams->iOffset -= pParams->iOffset % 5;

            // set new offset voltage
            SetOffsetVoltage(pParams->iOffset);
        }
    }
    // do the calculation back and forth even in logic mode,however, RestoreAmplitude will not change the level
    // assume that the calculation is done for sine mode.

    // did level or scale change?
    if ((LastParams.dLevel != pParams->dLevel) ||
            (LastParams.dLevelScaleLow != pParams->dLevelScaleLow) ||
            (LastParams.dLevelScaleHigh != pParams->dLevelScaleHigh))
    {
        // calculate appropriate level depending on waveform
        RestoreAmplitude();

        // calculate peak level and dBU level for display
        CrossCalcLevel(LEVEL2PEAKLEVEL);
        LastParams.dBULevel =
            pParams->dBULevel = 20 * log10(pParams->dLevel/774.597f);
        LastParams.dLevelScaleHigh = pParams->dLevelScaleHigh;
        LastParams.dLevelScaleLow = pParams->dLevelScaleLow;
    }


    // Adjust Peak 2 DACLevel (sine rms) => SetLevel adjusts to Waveform
    if (pParams->dPeakLevel != LastParams.dPeakLevel)
    {
        CrossCalcLevel(PEAKLEVEL2LEVEL);

        LastParams.dBULevel = pParams->dBULevel = 20 * log10(pParams->dLevel / 774.597f);
        LastParams.dLevel = -1; // force sweep to recalculate parameters

        // calculate appropriate level depending on waveform
        RestoreAmplitude();
    }

    // Adjust dBU 2 DACLevel (sine rms) => SetLevel adjusts to Waveform
    if (pParams->dBULevel != LastParams.dBULevel)
    {
        pParams->dLevel = 774.597f * powf(10, (pParams->dBULevel/20.0f));
        CrossCalcLevel(LEVEL2PEAKLEVEL);
        LastParams.dBULevel = pParams->dBULevel;
        LastParams.dLevel = -1; // force sweep to recalculate parameters

        // calculate appropriate level depending on waveform
        RestoreAmplitude();
    }


    if ( (LastParams.ucSweepMenu != pParams->ucSweepMenu) ||
            (LastParams.ucSweepMarker != pParams->ucSweepMarker) ||
            (LastParams.iSweepTime != pParams->iSweepTime)||
            (LastParams.ucSweepMode != pParams->ucSweepMode)||
            (LastParams.ucSweepMarkerMode != pParams->ucSweepMarkerMode) ||
            (LastParams.dSweepMarkerHeight != pParams->dSweepMarkerHeight)||
            (LastParams.dSweepStart != pParams->dSweepStart) ||
            (LastParams.dSweepEnd != pParams->dSweepEnd) ||
            (LastParams.dSweepCenter != pParams->dSweepCenter) ||
            (LastParams.dSweepSpanFactor != pParams->dSweepSpanFactor) ||
            (LastParams.dLevel != pParams->dLevel))

    {
        CalculateSweepParameters();

        if(SWEEP_OFF==pParams->ucSweepMenu)
        {
            // Restore Frequency to static mode
            AD9833_SetFrequency(pParams->dFrequency*64.0f);

            // Set Sync Signal to low (may be asserted)
            PORTC &= ~ucSyncOut;

            // Set Level to Normal (may be marker mode)
            RestoreAmplitude();
        }
    }

    // note: handling of sweep frequency, sync and marker handling completely moved to timer ISR to avoid jitter and gaps.

    // did TRMSC range change?
    if ((LastParams.ucRange != pParams->ucRange) ||
            (LastParams.dRMSScale100m != pParams->dRMSScale100m) ||
            (LastParams.dRMSScale1 != pParams->dRMSScale1) ||
            (LastParams.dRMSScale10 != pParams->dRMSScale10) ||
            (LastParams.dRMSScale100 != pParams->dRMSScale100) )
    {
        // set new range and save to EEPROM
        SwitchRange(pParams->ucRange);
    }

    // update all the parameters which do not directly change
    // hardware settings (in this case LastParams serves only as indicator
    // that the Parameter has to be re-displayed)
    if(LastParams.iBurstOnOff != pParams->iBurstOnOff)
    {
        // pParams->BurstOnOff is normally 0 or 1 if set from
        // panel but can be set to values > 1 from UART
        if (pParams->iBurstOnOff > 1)
        {
            // Set from Uart
            pParams->iBurst0 = pParams->iBurst1 = pParams->iBurstOnOff * 10;
        }
        else
        {
            if (pParams->iBurstOnOff == 0 && LastParams.iBurstOnOff > 1)
            {
                // Last parameter set from uart => reset new parms
                pParams->iBurst0 = pParams->iBurst1 = 0;
            }
            else
            {
                // Initialize new parms if coming from uart
                if (pParams->iBurst0 == 0 && pParams->iBurst1 == 0)
                {
                    Params.iBurst0 = Params.iBurst1 = 10;
                };
            }
        };

        // if burst was switched off, restore old waveform
        if(0==Params.iBurstOnOff) AD9833_SetWaveform(Params.ucWaveForm);
    }

    // did burst off time change?
    if(LastParams.iBurst0 != Params.iBurst0)
    {
        ATOMIC_RW(xg_uBurstTimer, 0);
    }

    // did burst on time change?
    if(LastParams.iBurst1 != Params.iBurst1)
    {
        ATOMIC_RW(xg_uBurstTimer, 0);
    }

#ifdef COMPILE_WITH_PWM

    // Trigger Pulses for non-continuous mode on every change per menu
    if (LastParams.iPWMimpulses!=Params.iPWMimpulses)
    {

        // stop any PWM burst that may be going on
        if (Params.iPWMimpulses == 0)
        {
            ATOMIC_RW(g_iPWMimpulseCount, 0);
            PORTC &= ~ucSyncOut;
        }
#ifdef PWM_SHOW_PULSES // when >> count is changed<< by menu (by default: Button press or command 71)
        else
        {
            ATOMIC_RW(g_iPWMimpulseCount, Params.iPWMimpulses);
        }
#endif
    }

#ifdef COMPILE_WITH_PWM_OC1A
    if (LastParams.ucPWMpolarity!=Params.ucPWMpolarity)
    {
        if (Params.ucPWMpolarity)
        {
            // invert PWM
            TCCR1A |= (1<<COM1A0);
            PORTD &= ~(1<<PD5);
        }
        else
        {
            //invert
            TCCR1A &= ~(1<<COM1A0);
            PORTD |= (1<<PD5);
        }

        if (Params.iPWMimpulses)  // run one pulse to clear the PWM logic
        {
            ATOMIC_RW(g_ucLastPulse, 1);       // suppress the sync
            ATOMIC_RW(g_iPWMimpulseCount, 1);
        }
    }
#else
    if (LastParams.ucPWMpolarity!=Params.ucPWMpolarity)
    {
        if (Params.ucPWMpolarity)
        {
            // invert PWM
            TCCR1A |= (1<<COM1B0);
            PORTD &= ~(1<<PD4);
        }
        else
        {
            // don't invert PWM
            TCCR1A &= ~(1<<COM1B0);
            PORTD |= (1<<PD4);
        }

        if (Params.iPWMimpulses)  // run one pulse to clear the PWM logic
        {
            ATOMIC_RW(g_ucLastPulse, 1);       // suppress the sync
            ATOMIC_RW(g_iPWMimpulseCount, 1);
        }
    }
#endif


#endif

// Update Display in case of Changes

//  CAUTION menu_entry_t are in PROGMEM !!! so pgm_read_byte/word are needed

    pUpdateDisplayFunct =  (void*) pgm_read_word(&g_currentMenuItem->pUpdateDisplayFunction);

    if ((pUpdateDisplayFunct) && (g_ucLCDpresent))
    {
        pUpdateDisplayFunct(0, (void*) pgm_read_word(&g_currentMenuItem->pParam));
    };


    if (memcmp(&LastParams, &Params, sizeof(Params)))
    {
        memcpy(&LastParams, &Params, sizeof(Params));
        EEPROM_WRITE_CACHED();
    }
}

