/*
 * Copyright (c) 2007 by Hartmut Birr, Thoralt Franz, Jörg Wilke
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
#include <avr/pgmspace.h>

#include <stdio.h>
#include <math.h>

#include "main.h"
#include "dds.h"
#include "dds-hw.h"
#include "parser.h"
#include "timer.h"

const PROGMEM char Mnemonics [][4] =
{
    {'n','o','p', 2},   // No OPeration
    {'f','r','q', 0},   // FReQuency
    {'l','v','l', 1},   // LeVeL
    {'l','v','p', 2},   // LeVel Peak
    {'d','b','u', 3},   // Level in dBu
    {'w','a','v', 4},   // WAVeform
    {'b','s','t', 5},   // BurST
    {'i','n','l', 10},  // INput Level of TRMSC-Module, 0=Veff, 1=Vpp, 2=Veff in dB
    {'r','n','g', 19},  // RaNGe of TRMSC-Module
    {'d','c','o', 20},  // DC Offset
    {'b','s','p', 30},  // BurSt Pause (Off Time)
    {'b','s','b', 31},  // BurSt Busy (On Time)
    {'s','w','s', 50},  // SWeep Start frequency
    {'s','w','e', 51},  // SWeep End frequency
    {'s','w','t', 52},  // SWeep Time
    {'s','w','d', 53},  // SWeep Direction
    {'s','w','c', 54},  // SWeep Center frequency
    {'s','w','f', 55},  // SWeep span Factor (for center frequency)
    {'s','w','m', 56},  // SWeep Mode
    {'s','w','k', 57},  // SWeep marKer
    {'s','w','h', 58},  // SWeep markerHeight
    {'s','w','p', 59},  // SWeeP menu
    {'s','w','o', 60},  // SWeep marker mOde
#ifdef COMPILE_WITH_PWM
    {'d','c','y', 70},  // DutyCYcle for PWM mode
    {'d','c','c', 71},  // DutyCycle Pulse Count for PWM mode
    {'d','c','p', 72},  // DutyCycle Polarity for PWM mode
#endif // COMPILE_WITH_PWM
    {'d','s','p', 80},  // Display Select Panel, Sets Menu of Panel
    {'a','l','l', 99},  // measurement values
    {'o','p','t', 150}, // DDS EEPROM OPTions
    {'s','c','l', 200}, // SCaLe for ADCs
    {'w','e','n', 250}, // EEPROM Write ENable
    {'t','r','g', 249}, // (?)
    {'e','r','c', 251}, // ERror Count since last reset
    {'s','b','d', 252}, // SerBaud UBRR register with U2X=1
    {'t','s','t', 253}, // TeST, returns command without doing anything
    {'i','d','n', 254}, // IDeNtify
    {'s','t','r', 255}, // STatus Request
    {'v','a','l', 0},   // VALue command, VAL can be omitted (Abbreviation for other commands)
    { 0, 0, 0, 0}       // Terminator
};

#define PARAM_FLOAT    0
#define PARAM_INT       1
#define PARAM_BYTE      2
#define PARAM_STR       3
#ifdef COMPILE_WITH_UNNECESSARY_STUFF
#define PARAM_L_1E6     4
#endif

#define SCALE_NONE      0
#ifdef COMPILE_WITH_UNNECESSARY_STUFF
#define SCALE_A         1
#define SCALE_mA        2
#define SCALE_uA        3
#endif
#define SCALE_PROZ      4
#define SCALE_TEMP      5
#define SCALE_FREQ      6
#define SCALE_mV        7
#define SCALE_V         8

typedef struct _PARAMTABLE
{
    uint8_t SubCh; // Command ordinal
    union
    {
        float (*get_f_Function)(void);
        int32_t (*get_l_Function)(void);
        int16_t (*get_i_Function)(void);
        uint8_t (*get_b_Function)(void);
        char* (*get_s_Function)(void);
        void (*doFunction)(struct _PARAMTABLE*);
        struct
        {
            union
            {
                float* f;
                int32_t* l;
                int16_t* i;
                uint16_t* u;
                uint8_t* b;
                const char* s;
            } ram;
            union
            {
                float* f;
                int32_t* l;
                int16_t* i;
                uint16_t* u; // TODO missing pointer ???????
                uint8_t* b;
                const char* s;
            } eep;
        } s;
    } u;
    uint8_t type        : 3; // Type of function/variable (int, float etc.)
    uint8_t scale       : 4;
    uint8_t rw          : 1; // 0 = read command (returns values to the caller), 1 = write (values to ctlab)
    uint8_t fct         : 2; // 0 = variable access, 1 = function pointer, 2 = special function
} PARAMTABLE;

void GetAll(PARAMTABLE* ParamTable __attribute__((unused)))
{
    ParseGetParam(10);
    ParseGetParam(11);
    ParseGetParam(15);
}

void SwitchRange(uint8_t ucRange)
{
    switch (ucRange)
    {
        case RANGE_100mV: // AC 100mV // AC Preamp x10 = x10
            Params.dInputGainFactor = 0.1f * Params.dRMSScale100m;
            SetGain10(1);         // AC Preamp x10
            SetAttenuator200(0);  // AC Attenuator :200
            break;
        case RANGE_1V: // AC 1 V
            Params.dInputGainFactor = 1.0f * Params.dRMSScale1;
            SetGain10(0);         // AC Preamp x10
            SetAttenuator200(0);  // AC Attenuator :200
            break;
        case RANGE_10V: // AC 10 V  // Attenuator 200, AC Preamp x10 = /20
            Params.dInputGainFactor = 10.0f * Params.dRMSScale10;
            SetGain10(1);         // AC Preamp x10
            SetAttenuator200(1);  // AC Attenuator :200
            break;
        default:
        case RANGE_100V: // 100V     // Attenuator 200 = /200
            Params.dInputGainFactor = 100.0f * Params.dRMSScale100;
            SetGain10(0);         // AC Preamp x10
            SetAttenuator200(1);  // AC Attenuator :200
            break;
    };
}


float GetTRMSC_RMS(void)
{
    int16_t ADCValue;
    ADCValue = GetADC(2);
    if (ADCValue > 1020)
    {
        Status.OverVolt = 1;
    }
    else
    {
        Status.OverVolt = 0;
    };
    return Status.OverVolt? (float) -9999 : (float) ADCValue * (float) Params.dInputGainFactor;
}

float GetTRMSC_dBU(void)
{
    return (20 * log10(GetTRMSC_RMS()/774.597));
}

float GetTRMSC_Peak(void)
{
    int16_t ADCValue;
    ADCValue = GetADC(3);
    if (ADCValue > 1020)
    {
        Status.OverVolt = 1;
    }
    else
    {
        Status.OverVolt = 0;
    };
    return Status.OverVolt? (float) -9999 : (float) ADCValue * (float) Params.dInputGainFactor * 2.82842712;
}


void ReturnInput(PARAMTABLE* ParamTable __attribute__((unused)))
{
    printf_P(PSTR("%s\n"), g_cSerInpStr);
}


const PROGMEM PARAMTABLE SetParamTable[] =
{

    // Entries to be ordered by Subchannel
    /* FRQ */
    {.SubCh = 0,   .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_FREQ, .u.s = {.ram.f = &Params.dFrequency, .eep.f = (float*)-1}},
    /* LVL */
    {.SubCh = 1,   .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_mV,   .u.s = {.ram.f = &Params.dLevel, .eep.f = (float*)-1}},
    /* LVP */
    {.SubCh = 2,   .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_mV,   .u.s = {.ram.f = &Params.dPeakLevel, .eep.f = (float*)-1}},
    /* DBU */
    {.SubCh = 3,   .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_mV,   .u.s = {.ram.f = &Params.dBULevel, .eep.f = (float*)-1}},
    /* WAV */
    {.SubCh = 4,   .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucWaveForm, .eep.b = (uint8_t*) -1}},
    /* BST */
    {.SubCh = 5,   .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iBurstOnOff, .eep.i = (int16_t*) -1}},
    /* INL 0 */
    {.SubCh = 10,  .rw = 0, .fct = 1, .type = PARAM_FLOAT,  .scale = SCALE_V,    .u.get_f_Function = &GetTRMSC_RMS},
    /* INL 1 */
    {.SubCh = 11,  .rw = 0, .fct = 1, .type = PARAM_FLOAT,  .scale = SCALE_V,    .u.get_f_Function = &GetTRMSC_Peak},
    /* INL 2 */
    {.SubCh = 12,  .rw = 0, .fct = 1, .type = PARAM_FLOAT,  .scale = SCALE_V,    .u.get_f_Function = &GetTRMSC_dBU},
    /* RNG */
    {.SubCh = 19,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucRange, .eep.b = (uint8_t*)-1}},
    /* DCO */
    {.SubCh = 20,  .rw = 1, .fct = 0, .type = PARAM_INT,  .scale = SCALE_V,    .u.s = {.ram.i = &Params.iOffset, .eep.i = (int16_t*)-1}},
    /* BS0 */
    {.SubCh = 30,  .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iBurst0, .eep.i = (int16_t*) -1}},
    /* BS1 */
    {.SubCh = 31,  .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iBurst1, .eep.i = (int16_t*) -1}},
    /* SWS */
    {.SubCh = 50,  .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_FREQ, .u.s = {.ram.f = &Params.dSweepStart, .eep.f = (float*)-1}},
    /* SWE */
    {.SubCh = 51,  .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_FREQ, .u.s = {.ram.f = &Params.dSweepEnd, .eep.f = (float*)-1}},
    /* SWT */
    {.SubCh = 52,  .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iSweepTime, .eep.i = (int16_t*) -1}},
    /* SWD */
    {.SubCh = 53,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucSweepSlope, .eep.b = (uint8_t*)-1}},
    /* SWC */
    {.SubCh = 54,  .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_FREQ, .u.s = {.ram.f = &Params.dSweepCenter, .eep.f = (float*)-1}},
    /* SWF */
    {.SubCh = 55,  .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_FREQ, .u.s = {.ram.f = &Params.dSweepSpanFactor, .eep.f = (float*)-1}},
    /* SWM */
    {.SubCh = 56,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucSweepMode, .eep.b = (uint8_t*)-1}},
    /* SWK */
    {.SubCh = 57,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucSweepMarker, .eep.b = (uint8_t*)-1}},
    /* SWH */
    {.SubCh = 58,  .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dSweepMarkerHeight, .eep.f = (float*)-1}},
//* SWP */
    {.SubCh = 59,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucSweepMenu, .eep.b = (uint8_t*)-1}},
//* SWO */
    {.SubCh = 60,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucSweepMarkerMode, .eep.b = (uint8_t*)-1}},
#ifdef COMPILE_WITH_PWM
//* DCY */
    {.SubCh = 70,  .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iPWMDuty, .eep.i = (int16_t*)-1}},
//* DCC */
    {.SubCh = 71,  .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iPWMimpulses, .eep.i = (int16_t*)-1}},
//* DCC */
    {.SubCh = 72,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucPWMpolarity, .eep.b = (uint8_t*)-1}},
#endif // COMPILE_WITH_PWM
    /* DSP 0 */
    {.SubCh = 80,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucDisplayedMenu, .eep.b = (uint8_t*) -1}},
    /* DSP 7 */
    {.SubCh = 85,  .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iScrollSpeed, .eep.i = &Params.iScrollSpeed}},
    /* DSP 6 */
    {.SubCh = 86,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucPermanentTRMSC, .eep.b = &Params.ucPermanentTRMSC}},
    /* DSP 7 */
    {.SubCh = 87,  .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iPermanentTRMSC_Delay, .eep.i = &Params.iPermanentTRMSC_Delay}},
    /* DSP 8 */
    {.SubCh = 88,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucAutosave, .eep.b = &Params.ucAutosave}},
    /* DSP 9 */
    {.SubCh = 89,  .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucEncoderPrescaler, .eep.b = &Params.ucEncoderPrescaler}},
    /* OPT 0, Frequency */
    {.SubCh = 150, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_FREQ, .u.s = {.ram.f = &Params.dFrequency, .eep.f = &Params.dFrequency}},
    /* OPT 1, Init Level */
    {.SubCh = 151, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_mV,   .u.s = {.ram.f = &Params.dLevel, .eep.f = &Params.dLevel}},
    /* OPT 2, Init Logic High*/
    {.SubCh = 152, .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_V,    .u.s = {.ram.i = &Params.iLogicHi, .eep.i = &Params.iLogicHi}},
    /* OPT 4, Init Wave Form */
    {.SubCh = 154, .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucWaveForm, .eep.b = &Params.ucWaveForm}},
    /* OPT 5, Init Burst on Time */
    {.SubCh = 155, .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.i = &Params.iBurst0, .eep.i = &Params.iBurst0}},
    /* OPT 19, Init Logic Low*/
    {.SubCh = 169, .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_V,    .u.s = {.ram.i = &Params.iLogicLo, .eep.i = &Params.iLogicLo}},
    /* OPT 20, Init Offset */
    {.SubCh = 170, .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_V,    .u.s = {.ram.i = &Params.iOffset, .eep.i = &Params.iOffset}},
    /* SCL 0 */
    {.SubCh = 200, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dLevelScaleLow, .eep.f = &Params.dLevelScaleLow}},
    /* SCL 1 */
    {.SubCh = 201, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dLevelScaleHigh, .eep.f = &Params.dLevelScaleHigh}},
    /* SCL 2 */
    {.SubCh = 202, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dOutGain, .eep.f = &Params.dOutGain}},
    /* SCL 3 */
    {.SubCh = 203, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dAttnFactor, .eep.f = &Params.dAttnFactor}},
    /* SCL 10 */
    {.SubCh = 210, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dRMSScale100m, .eep.f = &Params.dRMSScale100m}},
    /* SCL 11 */
    {.SubCh = 211, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dRMSScale1, .eep.f = &Params.dRMSScale1}},
    /* SCL 12 */
    {.SubCh = 212, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dRMSScale10, .eep.f = &Params.dRMSScale10}},
    /* SCL 13 */
    {.SubCh = 213, .rw = 1, .fct = 0, .type = PARAM_FLOAT,  .scale = SCALE_NONE, .u.s = {.ram.f = &Params.dRMSScale100, .eep.f = &Params.dRMSScale100}},
    /* ERC */
    {.SubCh = 251, .rw = 1, .fct = 0, .type = PARAM_INT,     .scale = SCALE_NONE, .u.s = {.ram.b = &g_ucErrCount, .eep.b = (uint8_t*)-1}},
    /* SBD */
    {.SubCh = 252, .rw = 1, .fct = 0, .type = PARAM_BYTE,    .scale = SCALE_NONE, .u.s = {.ram.b = &Params.ucSerBaudReg, .eep.b = &Params.ucSerBaudReg}},
    /* TST */
    {.SubCh = 253, .rw = 0, .fct = 2, .type = PARAM_STR,     .scale = SCALE_NONE, .u.doFunction=ReturnInput},
    /* IDN */
    {.SubCh = 254, .rw = 0, .fct = 0, .type = PARAM_STR,     .scale = SCALE_NONE, .u.s = {.ram.s = g_cVersStrLong}}

};


uint8_t ParseFindParamData(PARAMTABLE* Data, uint8_t SubCh)
{
    int16_t pos, min, max;

    min = 0;
    max = sizeof(SetParamTable) / sizeof(SetParamTable[0]) - 1;

    while (min <= max)
    {
        pos = (min + max) / 2;
        memcpy_P(Data, &SetParamTable[pos], sizeof(PARAMTABLE));

        if (SubCh == Data->SubCh)
        {
            // gefunden
            return 1;
        }

        if (SubCh > Data->SubCh)
        {
            min = pos + 1;
        }
        else
        {
            max = pos - 1;
        }
    }
    return 0;
}


void ParseGetParam(uint8_t SubCh)
{
    char fmt[20];
    PARAMTABLE ParamData;
    uint8_t fract_len = 4;
    union
    {
        float f;
        int32_t l;
        int16_t i;
        uint8_t b;
        const char* s;
    } Data;

#ifdef COMPILE_WITH_UNNECESSARY_STUFF
    uint32_t div, fract_div;
    uint8_t i;
    uint8_t neg = 0;
#endif

    if (SubCh == 255)
    {
        SerPrompt(NoErr, Status.u8);
        return;
    }

    if (!ParseFindParamData(&ParamData, SubCh))
    {
//        CHECKPOINT;
        SerPrompt(ParamErr, 0);
        return;
    }

    if (ParamData.fct == 2)
    {
        // special function
        ParamData.u.doFunction(&ParamData);
        return;
    }

    if (ParamData.fct == 1)
    {
        // get function
        switch (ParamData.type)
        {
            case PARAM_FLOAT:
                Data.f = ParamData.u.get_f_Function();
                break;

#ifdef COMPILE_WITH_UNNECESSARY_STUFF
            case PARAM_L_1E6:
                Data.l = ParamData.u.get_l_Function();
                break;
#endif

            case PARAM_INT:
                Data.i = ParamData.u.get_i_Function();
                break;

            case PARAM_BYTE:
                Data.b = ParamData.u.get_b_Function();
                break;

            case PARAM_STR:
                Data.s = ParamData.u.get_s_Function();
                break;

        }
    }
    else
    {
        // variable
        switch(ParamData.type)
        {
            case PARAM_FLOAT:
                Data.f = *ParamData.u.s.ram.f;
                break;

            case PARAM_INT:
                Data.i = *ParamData.u.s.ram.i;
                break;

            case PARAM_BYTE:
                Data.b = *ParamData.u.s.ram.b;
                break;

            case PARAM_STR:
                Data.s = ParamData.u.s.ram.s;
                break;

#ifdef COMPILE_WITH_UNNECESSARY_STUFF
            case PARAM_L_1E6:
                Data.l = *ParamData.u.s.ram.l;
                break;
#endif
        }
    }

    // handle scaling of float variables
    if (ParamData.type == PARAM_FLOAT)
    {
        switch (ParamData.scale)
        {
            case SCALE_PROZ:
                Data.f *= 1e2;
                fract_len = 2;
                break;

#ifdef COMPILE_WITH_UNNECESSARY_STUFF
            case SCALE_A:
// ERROR                fract_len = 6 - RangeI;
                break;

            case SCALE_mA:
                Data.f *= 1e3;
                fract_len = 3;
                break;

            case SCALE_uA:
                Data.f *= 1e6;
                fract_len = 0;
                break;
#endif
            case SCALE_TEMP:
                fract_len = 1;
                break;
            case SCALE_FREQ:
                fract_len = 3;
                break;
            case SCALE_mV:
                fract_len = 1;
                break;
            case SCALE_V:
                fract_len = 3;
                break;
        }
    }

    // print the parameters
    switch(ParamData.type)
    {
        case PARAM_FLOAT:
            // create a format string, because avrgcc doesn't handle a variable length format specifier like this "%.*f"
            sprintf_P(fmt, PSTR("#%%d:%%d=%%.%df\n"), fract_len);
            printf(fmt, g_ucSlaveCh, SubCh, Data.f);
            break;

        case PARAM_INT:
            if (ParamData.scale == SCALE_V)
            {
                printf_P(PSTR("#%d:%d=%.3f\n"), g_ucSlaveCh, SubCh, Data.i/1000.0);
            }
            else
            {
                printf_P(PSTR("#%d:%d=%d\n"), g_ucSlaveCh, SubCh, Data.i);
            }
            break;

        case PARAM_BYTE:
            printf_P(PSTR("#%d:%d=%u\n"), g_ucSlaveCh, SubCh, Data.b);
            break;

        case PARAM_STR:
            printf_P(PSTR("#%d:%d=%s\n"), g_ucSlaveCh, SubCh, Data.s);
            break;

#ifdef COMPILE_WITH_UNNECESSARY_STUFF
        case PARAM_L_1E6:
            switch(ParamData.scale)
            {
                default:
                    div = 1000000;
                    break;

                case SCALE_A:
// ERROR                    fract_len = 6 - RangeI;
                    div = 1000000;
                    break;

                case SCALE_mA:
                    fract_len = 3;
                    div = 1000;
                    break;

                case SCALE_uA:
                    fract_len = 0;
                    div = 1;
                    break;
            }

            fract_div = 1;

            for(i = 0; i < fract_len; i++)
            {
                fract_div *= 10;
            }
            if (Data.l < 0)
            {
                Data.l = -Data.l;
                neg = 1;
            }
            if (fract_len)
            {
                sprintf_P(fmt, PSTR("#%%d:%%d=%s%%ld.%%0%dlu\n"), neg ? "-" : "", fract_len);
                printf(fmt, g_ucSlaveCh, SubCh, Data.l / div, (Data.l / (div / fract_div)) % fract_div);
            }
            else
            {
                printf_P(PSTR("#%d:%d=%s%ld\n"), g_ucSlaveCh, SubCh, neg ? "-" : "", Data.l / div);
            }
            break;
#endif
    }
}

void ParseSetParam(uint8_t SubCh, float Param)
{
    if (Status.Busy)
    {
        SerPrompt(BusyErr, 0);
        return;
    }
    if (SubCh == 250)
    {
        Status.EEUnlocked = 1;
    }
    else
    {
        PARAMTABLE Data;

        if (!ParseFindParamData(&Data, SubCh) || !Data.rw)
        {
            SerPrompt(ParamErr, 0);
            return;
        }

        if (Data.u.s.eep.f != (void*)-1 && !Status.EEUnlocked)
        {
            SerPrompt(LockedErr, 0);
            return;
        }

        switch(Data.type)
        {
#ifdef COMPILE_WITH_UNNECESSARY_STUFF
            case PARAM_L_1E6:
                switch(Data.scale)
                {
                    default:
                    case SCALE_A:
                        *Data.u.s.ram.l = Param * 1e6;
                        break;

                    case SCALE_mA:
                        *Data.u.s.ram.l = Param * 1e3;
                        break;

                    case SCALE_uA:
                        *Data.u.s.ram.l = Param;
                        break;
                }
                break;
#endif
            case PARAM_FLOAT:
                switch(Data.scale)
                {
                    case SCALE_PROZ:
                        *Data.u.s.ram.f = Param / 1e2;
                        break;

#ifdef COMPILE_WITH_UNNECESSARY_STUFF
                    case SCALE_mA:
                        *Data.u.s.ram.f = Param / 1e3;
                        break;

                    case SCALE_uA:
                        *Data.u.s.ram.f = Param / 1e6;
                        break;
#endif
                    default:
#ifdef COMPILE_WITH_UNNECESSARY_STUFF
                    case SCALE_A:
#endif
                        *Data.u.s.ram.f = Param;
                        break;

                }

                break;

            case PARAM_BYTE:
                *Data.u.s.ram.b = (uint8_t)(Param + 0.5);
                break;

            case PARAM_INT:
                if (Data.scale == SCALE_V)
                {
                    *Data.u.s.ram.i = (int16_t)(Param * 1000.0); //no rounding (!) to get exact negative values
                }
                else
                {
                    *Data.u.s.ram.i = (int16_t)(Param + 0.5);
                }
                break;

        }

        if (Data.u.s.eep.f != (float*)-1)
        {
            uint8_t size;
            switch(Data.type)
            {
                case PARAM_FLOAT:
                    size = sizeof(float);
                    break;

                case PARAM_BYTE:
                    size = sizeof(uint8_t);
                    break;

                case PARAM_INT:
                    size = sizeof(int16_t);
                    break;

#ifdef COMPILE_WITH_UNNECESSARY_STUFF
                case PARAM_L_1E6:
                    size = sizeof(int32_t);
                    break;
#endif
                default:
                    size = 0;
                    break;
            }

            eeprom_write_block(Data.u.s.ram.f, (void*)((uint16_t)Data.u.s.eep.f-(uint16_t)&Params + 2), size); // can use EEPROMPARAMS (2) here maybe
        }
        Status.EEUnlocked = 0;
    }

#ifdef COMPILE_WITH_PWM
    if (SubCh == 71) // any update (even if same value) shall trigger the PWM pulse count
    {
        ATOMIC_RW(g_iPWMimpulseCount, Params.iPWMimpulses);
    }
#endif // COMPILE_WITH_PWM

    SerPrompt(NoErr, Status.u8);
}


