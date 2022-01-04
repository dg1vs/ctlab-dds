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

#include <avr/interrupt.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "dds-hw.h"
#include "dds.h"
#include "main.h"
#include "timer.h"

// this variable reflects the current state of the two 4094 shift registers
// (relays, square wave offset correction transistor and attenuator)
uint16_t uCurrent4094State = 0;

// this variable holds the state of the AD9833 DDS chip
uint16_t uCurrent9833State = 0;

// this is the previously selected waveform
uint8_t ucPreviousWave = WAVE_SINE;

#ifdef COMPILE_WITH_PWM
// the current timer prescaler for PWM mode
uint16_t g_uPWMPrescaler;
#endif

//----------------------------------------------------------------------------
// GetADC
//
// Read the value of a given ADC channel; the channel is sampled 4 times and
// the result is divided by 4
//
// -> Channel (0...7)
// <- 10 bits measurement
//----------------------------------------------------------------------------
uint16_t GetADC(uint8_t Channel)
{
    uint8_t i;
    uint16_t Result = 0;

    ADMUX = (Channel & 0x07);

    for (i = 0; i < 4; i++)
    {
        ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADIF)|(1<<ADPS2)|(1<<ADPS1);
        while (!(ADCSRA & (1<<ADIF)))
            ;
        Result += ADC;
    }
    return (Result + 2) / 4;
}

//----------------------------------------------------------------------------
// SetOffsetVoltage
//
// Set the offset voltage to the desired value. If iVoltage==0, the offset
// NULL relay will be switched on.
//
// A positive value of iVoltage causes a negative offset voltage to be gene-
// rated. When using the offset output as reference voltage, this leads to a
// positive signal shift (and vice versa).
//
// Execution time: 60 us
//
// -> iVoltage = integer value ranging from -10000 mV ... +10000 mV
// <- --
//----------------------------------------------------------------------------
void SetOffsetVoltage(int16_t iVoltage)
{
    // check limits
    if(iVoltage>10000) iVoltage = 10000;
    if(iVoltage<-10000) iVoltage = -10000;

    if(0 == iVoltage)
    {
        // if 0 V, switch on "offset null" relay, clear offset LED
        SetLED(OFFSET_LED, 0);
        SetRelay(RELAY_OFFSET_NULL, 1);

        // output 0 V (not really necessary because relay switches output
        // to GND)
        ShiftOut1257(OFFSET_NULL_VOLTAGE);
    }
    else
    {
        // set offset LED, switch off "offset null" relay
        SetLED(OFFSET_LED, 1);
        SetRelay(RELAY_OFFSET_NULL, 0);

        // output voltage
        ShiftOut1257(OFFSET_NULL_VOLTAGE + iVoltage/5);
    }
}

//----------------------------------------------------------------------------
// ShiftOut1257
//
// This function shifts out the given value to the D/A converter LTC1257
// (offset voltage)
//
// Execution time: 29.8 us
//
// -> uValue = 12 bits offset voltage (D11:D0)
// <- --
//----------------------------------------------------------------------------
void ShiftOut1257(uint16_t uValue)
{
    uint16_t uMask;
    uint8_t i;
    uint8_t sreg = SREG;

    cli();

    uMask = 0x0800;
    PORTB |= LTC1257_STROBE;        // strobe high

    for(i = 0; i < 12; i++)
    {
        CLK_LO();
        if(uValue & uMask)
        {
            PORTB |= SPI_DATA;      // SDATA high
        }
        else
        {
            PORTB &= ~SPI_DATA;     // SDATA low
        }
        CLK_HI();
        uMask >>= 1;
    }

    PORTB &= ~LTC1257_STROBE;       // strobe low
    nop();
    PORTB |= LTC1257_STROBE;        // strobe high

    SREG = sreg;
}

//----------------------------------------------------------------------------
// ShiftOut4094
//
// Shifts out the given value to the two 4094 shift registers
//
// Execution time: 10 us
//
// -> uValue:
//      Highest three bits (D15:D13) control the relays:
//        D15: SW3 (offset null switch)
//        D14: SW1 (external audio in)
//        D13: SW2 (AC passive attenuator)
//        D12: Q1 (square switch)
//      Lowest 12 bits (D11:D0) set the attenuation value of U6 (AD7541)
// <- --
//----------------------------------------------------------------------------
void ShiftOut4094(uint16_t uValue)
{
    uint8_t i;
    uint8_t sreg = SREG;

    cli();

    // it might look strange using no loop, but this is the fastest
    // way of shifting out the data
    PORTB &= ~SR4094_STROBE;        // strobe low
    i = uValue >> 8;

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x80) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x40) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x20) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x10) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x08) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x04) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x02) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x01) PORTB |= SPI_DATA;
    CLK_HI();

    i = uValue;

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x80) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x40) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x20) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x10) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x08) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x04) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x02) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB &= ~SPI_DATA;
    CLK_LO();
    if(i & 0x01) PORTB |= SPI_DATA;
    CLK_HI();

    PORTB |= SR4094_STROBE;         // strobe high
    nop();
    PORTB &= ~SR4094_STROBE;        // strobe low

    SREG = sreg;
}

//----------------------------------------------------------------------------
// SetRelay
//
// This function sets the relay to the desired state, the other relays as
// well as the attenuation value for AD7541 are not changed
//
// -> uRelay:
//     RELAY_ATTENUATOR  (0): SW2 (AC passive attenuator)
//     RELAY_EXTERNAL_IN (1): SW1 (external audio in)
//     RELAY_OFFSET_NULL (2): SW3 (offset null switch)
//    uState:
//     0: off
//     1: on
// <- --
//----------------------------------------------------------------------------
void SetRelay(uint8_t uRelay, uint8_t uState)
{
    if(uState)
        uCurrent4094State |= (1<<(13+uRelay));
    else
        uCurrent4094State &= ~(1<<(13+uRelay));
    ShiftOut4094(uCurrent4094State);
}

//----------------------------------------------------------------------------
// GetRelay
//
// This function gets the relay state
//
// -> --
// <- state of the relay with the following bits set or reset:
//     RELAY_ATTENUATOR  (0): SW2 (AC passive attenuator)
//     RELAY_EXTERNAL_IN (1): SW1 (external audio in)
//     RELAY_OFFSET_NULL (2): SW3 (offset null switch)
//----------------------------------------------------------------------------
uint8_t GetRelay(void)
{
    return (uCurrent4094State>>13) & 0x07;
}

//----------------------------------------------------------------------------
// SetAttenuation
//
// This function sets attenuation of the AD7541 to the desired value,
// the state of the relays is not changed
//
// Execution time: 33 us
//
// -> u: D12:D0 = value (0...4095) to set for AD7541
// <- --
//----------------------------------------------------------------------------
void SetAttenuation(uint16_t u)
{
    uCurrent4094State &= 0xF000;
    uCurrent4094State |= (u & 0x0FFF);
    ShiftOut4094(uCurrent4094State);
}

//----------------------------------------------------------------------------
// ShiftOut9833
//
// Shifts out the given value to the AD9833 frequency generator
//
// Execution time: 10 us
//
// -> uValue = 16 bits data to be shifted out
// <- --
//----------------------------------------------------------------------------
void ShiftOut9833(uint16_t uValue)
{
    uint8_t i;
    uint8_t sreg = SREG;

    cli();

    CLK_HI();

    // sync
    PORTB |= AD9833_STROBE;         // strobe high
    PORTB &= ~AD9833_STROBE;        // strobe low during transfer

    i = uValue >> 8;

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x80) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x40) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x20) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x10) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x08) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x04) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x02) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x01) PORTB |= SPI_DATA;
    CLK_LO();

    i = uValue;

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x80) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x40) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x20) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x10) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x08) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x04) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x02) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB &= ~SPI_DATA;
    CLK_HI();
    if(i & 0x01) PORTB |= SPI_DATA;
    CLK_LO();

    PORTB |= AD9833_STROBE;         // strobe high

    SREG = sreg;
}

//----------------------------------------------------------------------------
// AD9833_Init
//
// Initialize (reset) the DDS chip, set sine mode, set default frequency
//
// -> --
// <- --
//----------------------------------------------------------------------------
void AD9833_Init(void)
{
    // do a reset
    ShiftOut9833(AD9833_RESET);
    ShiftOut9833(0);

    // set the "B28" bit to indicate we want to load 28 bit values during the
    // next transfers (will be splitted into two 14 bit transfers)
    // this is a control word write since the two highest bits are 00
    ShiftOut9833(AD9833_B28);
    uCurrent9833State = AD9833_B28;

    // in detail the chip is initialized as following:
    // - load 28 bit wide frequency values
    // - use the FREQ0 register for the phase accumulator
    // - use the PHASE0 register (not useful for this project, however)
    // - reset off, sleep off
    // - sine wave

    // set a default frequency (1 kHz)
    AD9833_SetFrequency(64000L);
}

//----------------------------------------------------------------------------
// SetLED
//
// switch LEDs on or off
//
// -> ucLED = ACTIVITY_LED, OFFSET_LED
//    ucState = 0: LED off
//            = 1: LED on
// <- --
//----------------------------------------------------------------------------
/*void SetLED(uint8_t ucLED, uint8_t ucState)
{
    if(ucState)
    {
        PORTD &= ~ucLED;
    }
    else
    {
        PORTD |= ucLED;
    }
}*/

//----------------------------------------------------------------------------
// AD9833_SetWaveform
//
// Sets the waveform for the DDS chip
//
// Execution time: 40 us
//
// -> u = WAVE_OFF      (0): wave generation off
//      = WAVE_SINE     (1): sine
//      = WAVE_TRIANGLE (2): triangle
//      = WAVE_SQUARE   (3): square wave
//      = WAVE_LOGIC    (4): logic
//      = WAVE_EXT_IN   (5): external input (compilation option)
//      = WAVE_PWM      (5): via external input (compilation option)
// <- previously selected waveform
//----------------------------------------------------------------------------
uint8_t AD9833_SetWaveform(uint8_t ucWave)
{
    uint8_t uc = ucPreviousWave;
    ucPreviousWave = ucWave;
    uint8_t sreg = SREG;

    cli();

    // use if/else instead of switch, this saves us 8 bytes :)
    if(ucWave==WAVE_SINE)
    {
        uCurrent9833State = AD9833_B28;
        SetRelay(RELAY_EXTERNAL_IN, 0);
        uCurrent4094State &= ~(1<<12);  // switch off Q1 (square switch)
    }
    else if(ucWave==WAVE_TRIANGLE)
    {
        uCurrent9833State = AD9833_B28 | AD9833_MODE;
        SetRelay(RELAY_EXTERNAL_IN, 0);
        uCurrent4094State &= ~(1<<12);  // switch off Q1 (square switch)
    }
    else if(ucWave==WAVE_SQUARE)
    {
        uCurrent9833State = AD9833_B28 | AD9833_OPTBITEN | AD9833_DIV2;
        SetRelay(RELAY_EXTERNAL_IN, 0);
        uCurrent4094State |= (1<<12);   // switch on Q1 (square switch)
    }
    else if(ucWave==WAVE_LOGIC)
    {
        uCurrent9833State = AD9833_B28 | AD9833_OPTBITEN | AD9833_DIV2;
        SetRelay(RELAY_EXTERNAL_IN, 0);
        uCurrent4094State |= (1<<12);   // switch on Q1 (square switch)
    }
#ifdef COMPILE_WITH_PWM
    // PWM is fed into External Input

    else if(ucWave==WAVE_PWM)
    {
        uCurrent9833State = AD9833_RESET;
        SetRelay(RELAY_EXTERNAL_IN, 1);
        uCurrent4094State &= ~(1<<12);  // switch off Q1 (square switch)
    }
#else
    else if(ucWave==WAVE_EXT_IN)
    {
        uCurrent9833State = AD9833_RESET;
        SetRelay(RELAY_EXTERNAL_IN, 1);
        uCurrent4094State &= ~(1<<12);  // switch off Q1 (square switch)
    }
#endif

    else
    {
        // WAVE_OFF, default
        uCurrent9833State = AD9833_RESET;
        SetRelay(RELAY_EXTERNAL_IN, 0);
        uCurrent4094State &= ~(1<<12);  // switch off Q1 (square switch)
    }

    // write AD9833 control register
    ShiftOut9833(uCurrent9833State);

    // set new shift register state (maybe modified Q1 state)
    ShiftOut4094(uCurrent4094State);

    SREG = sreg;
    return uc;
}

//----------------------------------------------------------------------------
// SetLevel
//
// Sets the output Level. If the level is <= 0.2 V, the passive attenuator
// (1/40) is switched on and the output is scaled accordingly.
//
// Execution time: 75 us (when not switching)
//
// -> fLevel = output level in Volts (eff)
// <- --
//----------------------------------------------------------------------------
void SetLevel(float fLevel)
{
    float dSwitchPoint;


    // 4000 = used DAC counts, 8.0 (0.2) = scale end
    // value, 2.0f = amplification of output buffer
    dSwitchPoint = 4.0f * Params.dOutGain / Params.dAttnFactor;


    // Adapt Level which is based on sine to current waveform
    switch (Params.ucWaveForm)
    {
        case WAVE_TRIANGLE:
            fLevel *= 1.2247448714f;
            break;
        case WAVE_SQUARE:
            fLevel *= 0.7071067812f;
            break;
        case WAVE_LOGIC:
#ifdef COMPILE_WITH_PWM
        case WAVE_PWM:
#endif
            fLevel *= 0.3535533906f;
            break;
    }

    // check against maximum value
    if(fLevel>8.191) fLevel = 8.191;

    // switch on passive attenuator if necessary
    if(fLevel<=dSwitchPoint)
    {
        // if relay is not set: set level to zero, switch relay, wait,
        // then set level to new value
        if(0==(GetRelay()&(1<<RELAY_ATTENUATOR)))
        {
            SetAttenuation(0);
            SetRelay(RELAY_ATTENUATOR, 1);
            wait_ms(20);
        }
        SetAttenuation((int)(fLevel * 4000.0f * Params.dLevelScaleLow / dSwitchPoint ));
    }
    else
    {
        // if relay is set: set level to zero, switch relay, wait,
        // then set level to new value
        if(GetRelay()&(1<<RELAY_ATTENUATOR))
        {
            SetAttenuation(0);
            SetRelay(RELAY_ATTENUATOR, 0);
            wait_ms(20);
        }
        SetAttenuation((int)(fLevel * 1000.0f * Params.dLevelScaleHigh / Params.dOutGain));
    }
}

//----------------------------------------------------------------------------
// AD9833_SetFrequency
//
// Sets the frequency for the DDS chip
//
// Execution time: 86 us (when compiled without PWM)
//
// -> u = 32 bits unsigned integer, D27:D0 = 28 bits frequency value
//    frequency is calculated as u = f * 64
//    example: for 10 kHz call AD9833_SetFrequency(640000L);
// <- --
//----------------------------------------------------------------------------
void AD9833_SetFrequency(uint32_t u)
{

#ifdef COMPILE_WITH_PWM
    uint16_t uTimer;
    uint32_t uTemp;
    // if PWM mode is selected, the frequency is set via ICR1
    if(Params.ucWaveForm==WAVE_PWM)
    {
        // to allow maximum PWM resolution, the best timer prescaler has
        // to be chosen
        SetPWMPrescaler(&Params.dFrequency, NULL);
        uTimer = CalculatePWMTimerValue(Params.dFrequency, g_uPWMPrescaler);

        // set timer top value
        ICR1 = uTimer;

        // calculate switch point
        // use 32 bits to avoid overflow
        uTemp = uTimer;
        uTemp *= (1000-Params.iPWMDuty);
        uTemp /= 1000;
        OCR1B = uTemp;
        OCR1A = uTemp;

        // calculate real frequency and write back to Params
        Params.dFrequency = CalculatePWMFrequencyValue(uTimer, g_uPWMPrescaler);
    }
    else
    {
#endif
        cli();

        // write control register first to synchronize low- and high data word
        // (use the last saved state of the controlregister)
        ShiftOut9833(uCurrent9833State);

        // split the value into two 14 bit transfers
        // the highest two bits are set to 01 (0x4000), this means we write
        // to "FREQ0 REG"
        ShiftOut9833(0x4000 | (u & 0x3FFF));
        ShiftOut9833(0x4000 | ((uint32_t)(u>>14)&0x3FFFL));

        sei();

#ifdef COMPILE_WITH_PWM
    }
#endif

}

void SetGain10(uint8_t ucState)
{
    if(ucState==0)
    {
        PORTC &= ~ucGain10;
    }
    else
    {
        PORTC |= ucGain10;
    }
}

void SetAttenuator200(uint8_t ucState)
{
    if(ucState==0)
    {
        PORTC &= ~ucAttenuator200;
    }
    else
    {
        PORTC |= ucAttenuator200;
    }
}
