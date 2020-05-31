/*
 * Copyright (c) 2007 by Hartmut Birr, Thoralt Franz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "main.h"
#include "encoder.h"
#include "dds-hw.h"

//----------------------------------------------------------------------------
// globals
//----------------------------------------------------------------------------
// the following timers are decremented once every 500 us until they reach 0

// timer to avoid button bounce
volatile uint16_t xg_uButtonTimer = 0;

// timer for wait_ms()
volatile uint16_t xg_uMillisecondTimer = 0;

// timer for panel text scrolling
volatile uint16_t xg_uScrollTimer = 0;

// timer for TRMSC measurement
volatile uint16_t xg_uTRMSC_Timer = 0;

// timer for permanent TRMSC display
volatile uint16_t xg_uPermanentTRMSC_Timer = 0;

// timer for EEPROM write
volatile uint16_t xg_uEEPROM_Timer = 0;

// timer for burst
volatile uint16_t xg_uBurstTimer = 0;

// burst state: 0 = off, 1 = on
volatile uint8_t g_ucBurstState = 0;

// burst waveform
volatile uint8_t g_ucSavedBurstWaveform = WAVE_SINE;

#ifdef ENCODER_NAVIGATION
// timer for automatic encoder menu mode reset
volatile uint16_t g_uEncoderMenuTimer;

// timer for encoder turn symbol
volatile uint16_t g_uTurnSymbolTimer = 0;
#endif

// variables for sweep
volatile double xg_dSweepFrequency;
volatile double g_dLastSweepFrequency;
volatile double g_dSweepFrequencyFactorUp;
volatile double g_dSweepFrequencyFactorDown;

volatile uint32_t xg_uiSweepStartFrequency;
volatile uint32_t xg_uiSweepEndFrequency;
volatile uint32_t xg_uiSweepFrequencyIncrement;

volatile int16_t g_iSweepIndex = 0;
volatile uint16_t g_uSweepTimer = 0;
volatile uint8_t g_ucSweepDirection = SWEEP_UP;
volatile uint8_t g_ucSweepSync = 0;
volatile double  g_dDistanceMarker = 1.0f;
volatile double  g_dFirstLogMarker = 0.0f;
volatile double  g_dLastLogMarker = 0.0f;
volatile double  g_dFirstLinMarkerLow = 0.0f;
volatile double  g_dFirstLinMarkerHigh = 0.0f;
volatile double  g_dMarkerIndex = 0;
volatile double  g_dMarkerHeight = 1.2f;
volatile uint8_t g_ucSweepMark = 0;
volatile uint16_t g_uiMarkerLevel;
volatile uint16_t g_uiNormalLevel;

#ifdef COMPILE_WITH_PWM

volatile int16_t g_iPWMimpulseCount = 0;

//----------------------------------------------------------------------------
// ISR(TIMER1_OVF_vect)
//
// Timer 1 overflow interrupt 
// when reaching TOP, the PWM outputs a rising slope (non-inverted output PD4/PD5; falling on the DDS output)
//
// -> --
// <- --
//----------------------------------------------------------------------------
#if defined(__AVR_ATmega32__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_AT90CAN128__)
ISR(TIMER1_OVF_vect)
#else
#error Please define your TIMER1 code
#endif
{
    if (Params.iPWMimpulses)
    {
//        PORTC ^= ucSyncOut; // test output

        if (g_iPWMimpulseCount)
        {
            g_iPWMimpulseCount--;
#ifdef COMPILE_WITH_PWM_OC1A
            TCCR1A |= (1<<COM1A1);
#else
            TCCR1A |= (1<<COM1B1);
#endif
        }
        else
        {
#ifdef COMPILE_WITH_PWM_OC1A
            TCCR1A &= ~(1<<COM1A1);
#else
            TCCR1A &= ~(1<<COM1B1);
#endif
        }
    }
    else // continuous mode PWM when 0
    {
#ifdef COMPILE_WITH_PWM_OC1A
        TCCR1A |= (1<<COM1A1);
#else
        TCCR1A |= (1<<COM1B1);
#endif
    }
}

#endif // COMPILE_WITH_PWM

//----------------------------------------------------------------------------
// ISR(TIMER2_COMP_vect)
//
// Timer 2 compare interrupt
//
// -> --
// <- --
//----------------------------------------------------------------------------
#if defined(__AVR_ATmega32__) || defined(__AVR_AT90CAN128__)
ISR(TIMER2_COMP_vect)
#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
ISR(TIMER2_COMPA_vect)
#else
#error Please define your TIMER2 code
#endif
{

    static uint8_t ucLastMarker = 0;
    static uint8_t ucLastSync = 0;
    static uint8_t ucLastBurst = 0;
    static uint32_t uiSweepFrequency;
    static uint32_t uiLastFrequency = 0;

    // count down button press timer
    if(xg_uButtonTimer) xg_uButtonTimer--;

    // count down button press timer
    if(xg_uMillisecondTimer) xg_uMillisecondTimer--;

    // count down panel scroll text timer
    if(xg_uScrollTimer) xg_uScrollTimer--;

    // count down panel scroll text timer
    if(xg_uTRMSC_Timer) xg_uTRMSC_Timer--;

    // count down panel scroll text timer
    if(xg_uPermanentTRMSC_Timer) xg_uPermanentTRMSC_Timer--;

    // count down EEPROM timer
    if(xg_uEEPROM_Timer) xg_uEEPROM_Timer--;

#ifdef ENCODER_NAVIGATION
    // count down turning symbol timer
    if(g_uTurnSymbolTimer) g_uTurnSymbolTimer--;

    // count down menu mode timer
    if(g_uEncoderMenuTimer) g_uEncoderMenuTimer--;
#endif

//vvvvvvvvvvvv 42.8 us max
    // handle burst
    if(pParams->iBurstOnOff)
    {
        ucLastBurst = 1;
        if(xg_uBurstTimer) xg_uBurstTimer--;
        else
        {
            if(0==g_ucBurstState)
            {
                // switch signal off
                g_ucSavedBurstWaveform = AD9833_SetWaveform(WAVE_OFF);
                xg_uBurstTimer = (pParams->iBurst0<<1) - 1;
                g_ucBurstState = 1;
                PORTC &= ~ucSyncOut;
            }
            else
            {
                // switch signal on
                AD9833_SetWaveform(g_ucSavedBurstWaveform);
                xg_uBurstTimer = (pParams->iBurst1<<1) - 1;
                g_ucBurstState = 0;
                PORTC |= ucSyncOut;
            }
        }
    }

    if ((pParams->iBurstOnOff == 0) && (ucLastBurst == 1))
    {
        PORTC &= ~ucSyncOut;
        ucLastBurst = 0;
    }
//^^^^^^^^^^^^ 42.8 us max

//vvvvvvvvvvvv 116 us max
    // handle sweep
#ifdef COMPILE_WITH_PWM
    if((pParams->ucSweepMenu!=SWEEP_OFF) && ( (pParams->ucWaveForm != WAVE_LOGIC) && (pParams->ucWaveForm != WAVE_PWM) ) )
#else
    if((pParams->ucSweepMenu!=SWEEP_OFF) && (pParams->ucWaveForm != WAVE_LOGIC))
#endif
    {
        // count down sweep prescaler
        if(g_uSweepTimer) g_uSweepTimer--;
        else
        {
            // reset sweep prescaler
            g_uSweepTimer = SWEEP_UPDATE_INTERVAL*2 - 1;

            switch(Params.ucSweepMode)
            {
                case SWEEP_LINEAR:
                {

                    switch(Params.ucSweepSlope)
                    {
                        case SWEEP_UP:
                            g_iSweepIndex += SWEEP_UPDATE_INTERVAL;
                            if(g_iSweepIndex>=pParams->iSweepTime)
                            {
                                g_iSweepIndex = 0;
                                g_ucSweepSync = 1;
                                g_dMarkerIndex = g_dFirstLinMarkerLow;
                            }
                            else
                            {
                                g_ucSweepSync = 0;
                            }
                            break;
                        case SWEEP_DOWN:
                            g_iSweepIndex -= SWEEP_UPDATE_INTERVAL;
                            if(g_iSweepIndex<=0)
                            {
                                g_iSweepIndex = pParams->iSweepTime;
                                g_ucSweepSync = 1;
                                g_dMarkerIndex = g_dFirstLinMarkerHigh;
                            }
                            else
                            {
                                g_ucSweepSync = 0;
                            }

                            break;
                        case SWEEP_UPDOWN:
                            if(SWEEP_UP == g_ucSweepDirection)
                            {
                                g_ucSweepSync = 1;
                                g_iSweepIndex += SWEEP_UPDATE_INTERVAL;
                                if(g_iSweepIndex>=pParams->iSweepTime)
                                {
                                    g_iSweepIndex = pParams->iSweepTime;
                                    g_ucSweepDirection = SWEEP_DOWN;
                                    g_dMarkerIndex = g_dFirstLinMarkerHigh;
                                }
                            }
                            else
                            {
                                g_ucSweepSync = 0;
                                g_iSweepIndex -= SWEEP_UPDATE_INTERVAL;
                                if(g_iSweepIndex<=0)
                                {
                                    g_iSweepIndex = 0;
                                    g_ucSweepDirection = SWEEP_UP;
                                    g_dMarkerIndex = g_dFirstLinMarkerLow;
                                }
                            }
                            break;
                    }

                    // calculate Frequency to be set at the end of ISTR
                    uiSweepFrequency = xg_uiSweepStartFrequency + g_iSweepIndex * xg_uiSweepFrequencyIncrement;


                    if ( (Params.ucSweepSlope == SWEEP_UP ) || ((Params.ucSweepSlope == SWEEP_UPDOWN ) && (SWEEP_UP == g_ucSweepDirection) ) )
                    {
                        if ( uiSweepFrequency >= g_dMarkerIndex )
                        {
                            g_dMarkerIndex *= ((pParams->ucSweepMarker==MARKER_OCTAVE) ? (double)2.0f : (double) 10.0f);    // ( ? : ) saves space
                            g_ucSweepMark = 1;
                        }
                        else
                        {
                            g_ucSweepMark = 0;
                        }
                    }
                    else
                    {
                        if ( uiSweepFrequency <= g_dMarkerIndex )
                        {
                            g_dMarkerIndex *= ((pParams->ucSweepMarker==MARKER_OCTAVE) ? (double)0.5f : (double) 0.1f);  // ( ? : ) saves space
                            g_ucSweepMark = 1;
                        }
                        else
                        {
                            g_ucSweepMark = 0;
                        }
                        break;
                    }

                }// switch(pParams->ucSweepMode == LINEAR)

                break;

                case SWEEP_LOG:
                    // note: the concept for this mode is keep interrupt time as short as possible by using just one double multiplication
                    //       if repeated too often, the "almost infinite" resolution of (double) is not exact, but good enough.
                    //       however, the start/end values for the frequencies are set for every sweep, so there shouldn't be a noticeable deviation.
                {
                    switch(pParams->ucSweepSlope)
                    {
                        case SWEEP_UP:
                            g_iSweepIndex += SWEEP_UPDATE_INTERVAL;
                            if(g_iSweepIndex>=pParams->iSweepTime)
                            {
                                g_iSweepIndex = 0;
                            }

                            if ( g_iSweepIndex == 0)
                            {
                                xg_dSweepFrequency = xg_uiSweepStartFrequency;
                                g_ucSweepSync = 1;
                                g_dMarkerIndex = g_dFirstLogMarker;
                            }
                            else
                            {
                                xg_dSweepFrequency *= g_dSweepFrequencyFactorUp;
                                g_ucSweepSync = 0;
                            }

                            if ( g_iSweepIndex >= g_dMarkerIndex )
                            {
                                g_dMarkerIndex += g_dDistanceMarker;
                                g_ucSweepMark = 1;
                            }
                            else
                            {
                                g_ucSweepMark = 0;
                            }
                            break;
                        case SWEEP_DOWN:
                            g_iSweepIndex -= SWEEP_UPDATE_INTERVAL;
                            if(g_iSweepIndex<=0) g_iSweepIndex = pParams->iSweepTime;
                            if ( g_iSweepIndex == pParams->iSweepTime)
                            {
                                xg_dSweepFrequency = xg_uiSweepEndFrequency;
                                g_ucSweepSync = 1;
                                g_dMarkerIndex = g_dLastLogMarker;
                            }
                            else
                            {
                                xg_dSweepFrequency *= g_dSweepFrequencyFactorDown;
                                g_ucSweepSync = 0;
                            }
                            if ( g_iSweepIndex <= g_dMarkerIndex )
                            {
                                g_dMarkerIndex -= g_dDistanceMarker;
                                g_ucSweepMark = 1;
                            }
                            else
                            {
                                g_ucSweepMark = 0;
                            }
                            break;
                        case SWEEP_UPDOWN:
                            if(SWEEP_UP == g_ucSweepDirection)
                            {
                                g_iSweepIndex += SWEEP_UPDATE_INTERVAL;
                                xg_dSweepFrequency *= g_dSweepFrequencyFactorUp;


                                if(g_iSweepIndex>=pParams->iSweepTime)
                                {
                                    g_iSweepIndex = pParams->iSweepTime;
                                    g_ucSweepDirection = SWEEP_DOWN;
                                    xg_dSweepFrequency = xg_uiSweepEndFrequency;
                                    g_ucSweepSync = 0;
                                    g_dMarkerIndex = g_dLastLogMarker;
                                }
                                else
                                {
                                    if ( g_iSweepIndex >= g_dMarkerIndex )
                                    {
                                        g_dMarkerIndex += g_dDistanceMarker;
                                        g_ucSweepMark = 1;
                                    }
                                    else
                                    {
                                        g_ucSweepMark = 0;
                                    }
                                }

                            }
                            else
                            {
                                g_iSweepIndex -= SWEEP_UPDATE_INTERVAL;
                                xg_dSweepFrequency *= g_dSweepFrequencyFactorDown;

                                if(g_iSweepIndex<=0)
                                {
                                    g_iSweepIndex = 0;
                                    g_ucSweepDirection = SWEEP_UP;
                                    xg_dSweepFrequency = xg_uiSweepStartFrequency;
                                    g_ucSweepSync = 1;
                                    g_dMarkerIndex = g_dFirstLogMarker;
                                }
                                else
                                {
                                    if ( g_iSweepIndex <= g_dMarkerIndex )
                                    {
                                        g_dMarkerIndex -= g_dDistanceMarker;
                                        g_ucSweepMark = 1;
                                    }
                                    else
                                    {
                                        g_ucSweepMark = 0;
                                    }
                                }
                            }
                            break;

                            // Sweep Frequency is finally set inside jobExecute()
                    }// switch(pParams->ucSweepSlope)

                    // for log mode now convert frequency
                    uiSweepFrequency = (unsigned long) xg_dSweepFrequency;

                } //switch(pParams->ucSweepMode == LOG)
                break;

            } //end of switch(pParams->ucSweepMode)


            // to avoid jitter for sync and markers, handle the sync pulse generation here in the ISR instead of JobExecute

            if (g_ucSweepSync != ucLastSync)
            {
                if (g_ucSweepSync)
                {
                    PORTC |= ucSyncOut;
                }
                else
                {
                    PORTC &= ~ucSyncOut;
                }
                ucLastSync = g_ucSweepSync;
            }

            if (uiSweepFrequency != uiLastFrequency)
            {
                // frequency is already calculated in Hz/64
                AD9833_SetFrequency(uiSweepFrequency);
                uiLastFrequency = uiSweepFrequency;
            }

            if ((pParams->ucSweepMarker != MARKER_OFF) && (g_ucSweepMark != ucLastMarker))
            {

                if (g_ucSweepMark)
                {
                    // set Output voltage to MarkerLevel
                    SetAttenuation(g_uiMarkerLevel);
                }
                else
                {
                    // set Output voltage to normal level
                    SetAttenuation(g_uiNormalLevel);
                }
                ucLastMarker = g_ucSweepMark;
            }

        }

    } //end of switch(pParams->ucSweepMenu)

//^^^^^^^^^^^^ 116 us max
//  depending on mode:
//  normal non-sweep        =   0.5 us max
//  linear sweep            =  63.0 us max
//  log sweep               = 104   us max
//  log sweep w/ markers    = 116   us max

//vvvvvvvvvvvv 12.3 us max
    jobEncoder();
//^^^^^^^^^^^^ 12.3 us max

}

#ifdef COMPILE_WITH_PWM
//----------------------------------------------------------------------------
// CalculatePWMTimerValue
//
// Calculate value for PWM timer using f_cpu as input frequency, the
// prescaler and dFrequency.
//
// -> dFrequency: Frequency to calculate the timer value for
//    uPrescaler: Prescaler to use for calculation
// <- calculated timer value
//----------------------------------------------------------------------------
uint16_t CalculatePWMTimerValue(double dFrequency, uint16_t uPrescaler)
{
    return ((uint16_t)((double)(F_CPU / uPrescaler) / dFrequency) - 1);
}

//----------------------------------------------------------------------------
// CalculatePWMFrequencyValue
//
// Calculate value for PWM frequency using f_cpu as input frequency, the
// prescaler and PWM timer value.
//
// -> iTimerValue: Timer value to calculate the frequency for
//    uPrescaler: Prescaler to use for calculation
// <- calculated frequency
//----------------------------------------------------------------------------
double CalculatePWMFrequencyValue(uint16_t iTimerValue, uint16_t uPrescaler)
{
    return ((double)(F_CPU / uPrescaler)) / (double)(iTimerValue + 1);
}

//----------------------------------------------------------------------------
// SetPWMPrescaler
//
// Sets the PWM prescaler to a value which fits best to a given frequency or
// just calculates the prescaler without setting it, depending on uPrescaler.
//
// If the frequency is < 1.0 Hz, it is set to 1.0 Hz.
//
// If uPrescaler is not NULL, the prescaler is not set in hardware, but copied
// into uPrescaler. This is used inside panel.c to calculate the prescaler.
//
// -> dFrequency: pointer to frequency to calculate the prescaler for
// <-
//----------------------------------------------------------------------------
void SetPWMPrescaler(double *dFrequency, uint16_t *uPrescaler)
{
    if(*dFrequency<5.0f)
    {
        // PWM frequencies below 1.0 Hz are not possible with 16 MHz,
        // prescaler = 256 and a 16 bit timer, therefore limit the
        // value to 1.0 Hz
        if(*dFrequency<1.0f)
        {
            *dFrequency = 1.0f;
        }

        if(uPrescaler==NULL)
        {
            // set prescaler to 256
            SET_TIMER1_PRESCALER_256();
            g_uPWMPrescaler = 256;
        }
        else
        {
            *uPrescaler = 256;
        }
    }
    else if(*dFrequency<32.0f)
    {
        if(uPrescaler==NULL)
        {
            // set prescaler to 64
            SET_TIMER1_PRESCALER_64();
            g_uPWMPrescaler = 64;
        }
        else
        {
            *uPrescaler = 64;
        }
    }
    else if(*dFrequency<250.0f)
    {
        if(uPrescaler==NULL)
        {
            // set prescaler to 8
            SET_TIMER1_PRESCALER_8();
            g_uPWMPrescaler = 8;
        }
        else
        {
            *uPrescaler = 8;
        }
    }
    else
    {
        if(uPrescaler==NULL)
        {
            // set prescaler to 1
            SET_TIMER1_PRESCALER_1();
            g_uPWMPrescaler = 1;
        }
        else
        {
            *uPrescaler = 1;
        }
    }
}
#endif


//----------------------------------------------------------------------------
// InitTimer
//
// Initialize Timer2 to interrupt every 500 us
//
// -> --
// <- --
//----------------------------------------------------------------------------
void InitTimer(void)
{
    uint8_t sreg = SREG;
    cli();


#if defined(__AVR_ATmega32__)

    // initialize Timer 2, period 500us, prescaler 32, CTC mode
    OCR2 = (F_CPU / 64000UL) - 1;
    TCNT2 = 0;
    TCCR2 = (1<<WGM21)|(1<<CS21)|(1<<CS20);
    TIMSK |= (1<<OCF2);

#elif defined(__AVR_AT90CAN128__)

    OCR2A = (F_CPU / 64000UL) - 1;
    TCNT2 = 0;
    TCCR2A = (1 << WGM21) | (1 << CS21) | (1 << CS20);
    TIMSK2 |= (1 << OCIE2A);

#elif defined(__AVR_ATmega1284P__)

#if (F_CPU == 20000000)

    // 20 MHZ special hardware -> dg1vs
    // initialize Timer 2, period 500us, prescaler 64, CTC mode
    OCR2A = (F_CPU / 128000UL) - 1;
    OCR2B = 0;
    TCNT2 = 0;
    TCCR2A = (1<<WGM21);
    TCCR2B = (1<<CS22);
    TIMSK2 |= (1 << OCIE2A);
#else
    // normal stuff -> 16 MHz
    // initialize Timer 2, period 500us, prescaler 32, CTC mode
    OCR2A = (F_CPU / 64000UL) - 1;
    OCR2B = 0;
    TCNT2 = 0;
    TCCR2A = (1<<WGM21);
    TCCR2B = (1<<CS21)|(1<<CS20);
    TIMSK2 |= (1 << OCIE2A);
#endif

#else
#error Please define your TIMER1 / TIMER2 code or check the avr manual
#endif

#ifdef COMPILE_WITH_PWM
// Timer 1 is the same for all ATmega cpu options
// clear OC1A/OC1B on compare match, set OC1A/OC1B at TOP
// Waveform Generation Mode = 14 (Fast PWM with ICR1)
// clock prescaler = 1

#ifdef COMPILE_WITH_PWM_OC1A
    TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11)|(0<<WGM10);
#else
    TCCR1A = (1<<COM1B1)|(1<<WGM11)|(0<<WGM10);
#endif

SET_TIMER1_PRESCALER_1();
    ICR1 = F_CPU / 1000UL; // TOP value for default 1000Hz

#ifdef COMPILE_WITH_PWM_OC1A
    OCR1A = F_CPU / 2000UL; // default 50%
#endif
    OCR1B = F_CPU / 2000UL; // default 50%

// initialize Timer 1 Overflow interrupt for PWM impulse counting (same for both options OC1B or OC1A)
    TIMSK |= (1<<TOIE1);

#endif

    SREG = sreg;
}

//----------------------------------------------------------------------------
// wait_ms()
//
// Waits a given number of milliseconds
//
// -> uMS = milliseconds to wait
// <- --
//----------------------------------------------------------------------------
void wait_ms(uint16_t uMS)
{
    uint16_t u;
    ATOMIC_RW(xg_uMillisecondTimer, (uMS<<1));
    do
    {
        ATOMIC_RW(u, xg_uMillisecondTimer);
    }
    while(u);
}
