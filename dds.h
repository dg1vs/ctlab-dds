/*
 * Copyright (c) 2007 by Hartmut Birr, Thoralt Franz, J�rg Wilke
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

#ifndef __DDS_H__
#define __DDS_H__

#include <inttypes.h>
#include <avr/eeprom.h>


#define VERSSTRSHORT_STD "DDS C1.0"     // 8 chars max  
#define VERSSTRLONG_STD  "1.0d[DDS C by HB, TF, JW and PSC]"

#define VERSSTRSHORT_PWM "DDSpC1.0"     // 8 chars max
#define VERSSTRLONG_PWM  "1.0d[DDSp C by HB, TF, JW and PSC]"


#define COMPILE_WITH_PWM 1         // default only OC1B / PD4  // requires hardware change // will replace ext. Audio-in with PWM menu/function
//#define COMPILE_WITH_PWM_OC1A 1  // option additionally to COMPILE_WITH_PWM / OC1A/PD5, TODO: auto detection of PWM on PD4/PD5
//define PWM_SHOW_PULSES 1         // show pulses when manually changing the value via encoder

//#define COMPILE_WITH_DISPLAY204 1
//#define ENCODER_NAVIGATION 1

#if defined(COMPILE_WITH_DISPLAY204) && defined(ENCODER_NAVIGATION)
#error DISPLAY204 does not work with ENCODER_NAVIGATION; untested and the encoder symbols are missing
#endif

#define STRICTSYNTAX

//----------------------------------------------------------------------------
// TRMSC ranges
//----------------------------------------------------------------------------
#define RANGE_100mV 0
#define RANGE_1V    1
#define RANGE_10V   2
#define RANGE_100V  3
#define RANGE_AUTO  4

//----------------------------------------------------------------------------
// permanent TRMSC constants
//----------------------------------------------------------------------------
#define PERMANENT_TRMSC_OFF     0
#define PERMANENT_TRMSC_RMS     1
#define PERMANENT_TRMSC_PEAK    2

//----------------------------------------------------------------------------
// waveform names
//----------------------------------------------------------------------------
#define WAVE_OFF            0
#define WAVE_SINE           1
#define WAVE_TRIANGLE       2
#define WAVE_SQUARE         3
#define WAVE_LOGIC          4

#ifdef COMPILE_WITH_PWM
#define WAVE_PWM            5
#else
#define WAVE_EXT_IN         5
#endif

//----------------------------------------------------------------------------
// sweep modes
//----------------------------------------------------------------------------
// sweep menu
#define SWEEP_OFF       0
#define SWEEP_ON        1

//sweep slopes
#define SWEEP_UP        0
#define SWEEP_DOWN      1
#define SWEEP_UPDOWN    2

//sweep mode
#define SWEEP_LOG       0
#define SWEEP_LINEAR    1

//marker
#define MARKER_OFF      0
#define MARKER_DECADE   1
#define MARKER_OCTAVE   2
//#define MARKER_ONEPONE    3

//marker mode
#define MARKER_ABSOLUTE 0
#define MARKER_RELATIVE 1

// time between sweep updates (in ms)
#define SWEEP_UPDATE_INTERVAL 2

//----------------------------------------------------------------------------
// Menu identifiers
//----------------------------------------------------------------------------
#define MENU_WAVEFORM   0
#define MENU_FREQUENCY  1
#define MENU_LEVEL_RMS  2
#define MENU_LEVEL_PEAK 3
#define MENU_TRMSC      4
#define MENU_BURST      5
#define MENU_OFFSET     6
#define MENU_SWEEP     20
#define MENU_FM        21
#define MENU_AM        22

#define SUBMENU_SAVE            23
#define SUBMENU_PERMANENTTIME   24
#define SUBMENU_SCROLLSPEED     25
#define SUBMENU_SETTINGSBACK    26
#define SUBMENU_TRMS_RANGE      27
#define SUBMENU_TRMS_RMS        28
#define SUBMENU_TRMS_PEAK       29
#define SUBMENU_TRMS_PERMANENT  30
#define SUBMENU_TRMS_BACK       31
#define SUBMENU_LOGIC_HIGH      32
#define SUBMENU_LOGIC_LOW       33
#define SUBMENU_LOGIC_BACK      34
#define SUBMENU_SWEEP_START     35
#define SUBMENU_SWEEP_END       36
#define SUBMENU_SWEEP_TIME      37
#define SUBMENU_SWEEP_BACK      38
#define SUBMENU_BURST0          39
#define SUBMENU_BURST1          40
#define SUBMENU_BURST_BACK      41
#define SUBMENU_SWEEP_MODE      42

#define SUBMENU_SWEEP_CENTER    43
#define SUBMENU_SWEEP_SPANFACTOR 44
#define SUBMENU_SWEEP_SLOPE     45
#define SUBMENU_SWEEP_MARKER    46
#define SUBMENU_SWEEP_MARKERMODE 47
#define SUBMENU_SWEEP_MARKERHGHT 48

#define SUBMENU_PWM_DUTY        49
#define SUBMENU_PWM_COUNT       50
#define SUBMENU_PWM_POLARITY    51
#define SUBMENU_PWM_HIGH        52
#define SUBMENU_PWM_LOW         53
#define SUBMENU_PWM_BACK        53

// time until menu mode is switched from encoder mode back to normal
// (in 0.5 ms units)
#define ENCODER_MENU_TIME   10000

//----------------------------------------------------------------------------
// level cross calculation
//----------------------------------------------------------------------------
#define LEVEL2PEAKLEVEL 0
#define PEAKLEVEL2LEVEL 1

//----------------------------------------------------------------------------
// sweep cross calculation
//----------------------------------------------------------------------------
#define STARTEND2CENTER 0
#define CENTER2STARTEND 1

//----------------------------------------------------------------------------
// maximum frequency
//----------------------------------------------------------------------------
#define MAX_FREQUENCY   300000.0f

//----------------------------------------------------------------------------
// EEPROM addresses
//----------------------------------------------------------------------------
// one word for validation
#define EEPROM_VALID    (void*)0
// complete PARAMS structure
#define EEPROM_PARAMS   (void*)2
// time to wait between EEPROM writes (60000 means 30 seconds)
#define MINIMUM_EEPROM_TIME 60000
#define EEPROM_WRITE_CACHED() {\
    ATOMIC_RW(xg_uEEPROM_Timer, MINIMUM_EEPROM_TIME); g_ucModifiedParams = 1;}

typedef union
{
    uint8_t u8;
    struct
    {
        uint8_t Unused      : 1;
        uint8_t FuseBlown   : 1;
        uint8_t OverVolt    : 1; // In our case TRMSC Overvoltage
        uint8_t OverTemp    : 1;
        uint8_t EEUnlocked  : 1;
        uint8_t CurrentMode : 1;
        uint8_t UserSRQ     : 1;
        uint8_t Busy        : 1;
    };
} STATUS;

typedef struct
{
    double dFrequency;         //* Frequency in Hertz, 3 digits fraction
    double dLevel;             //* Voltage effective in mV
    double dPeakLevel;         //* Peak2PeakLevel in mV
    double dBULevel;           //* Voltage effective in dBU
    int16_t iOffset;           //* in mV
    double dSweepStart,        //* for Start/End Sweep Mode
           dSweepEnd;          //* Both in Hertz
    double dSweepCenter,       //* for Center/Span Sweep Mode, SweepCenter in Hz
           dSweepSpanFactor;       //* Factor
    int16_t iSweepTime;        //* in ms
    int16_t iLogicHi, iLogicLo; //* logic levels in mV
    uint8_t ucWaveForm;
    int16_t iBurst1, iBurst0;  //* in ms
    int16_t iBurstOnOff;       //* 0 = Burst off; 2-maxint = Burst on, use burstonoff as 10 times ms burst off;
                               //* 1 = Burst on, use Parameters burst1 and burst0, set bost to 1 if not populated.
    uint8_t ucSerBaudReg;
    double dInputGainFactor;   //* Factor to multiply ADC3 for TRMS Value
    uint8_t ucRange;           //* Input Range for TRMSC
    uint8_t ucPermanentTRMSC;
    int16_t iPermanentTRMSC_Delay;
    int16_t iScrollSpeed;
    uint8_t ucSweepMenu;
    uint8_t ucSweepMarker;
    uint8_t ucSweepMarkerMode;
    double  dSweepMarkerHeight;
    uint8_t ucSweepMode;
    uint8_t ucSweepSlope;
    uint8_t ucVolt_dB;
    uint8_t ucDisplayedMenu;    //* Current Menu on Panel (for DSP-Command)
    uint8_t ucAutosave;
    uint8_t ucEncoderPrescaler;
    double  dLevelScaleLow;     //* Level correction  <200 mV
    double  dLevelScaleHigh;    //* Level correction  >=200 mV
    double  dRMSScale100m;      //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    double  dRMSScale1;         //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    double  dRMSScale10;        //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    double  dRMSScale100;       //* Input correction  100 mV  // 100 mV, 1V, 10V, 100V
    double  dOutGain;           //* Output Gain
    double  dAttnFactor;        //* Attenuation 1/40
    int16_t iPWMDuty;
    int16_t iPWMimpulses;
    uint8_t ucPWMpolarity;
}  PARAMS;

extern STATUS Status;
extern PARAMS Params;
extern PARAMS *pParams;
extern PARAMS LastParams;
extern PARAMS *pLastParams;

extern uint8_t g_ucSlaveCh;
extern uint8_t g_ucTrackCh;
extern uint8_t g_ucErrCount;

extern uint8_t g_ucLCDpresent;

extern double g_dTRMSC_RMS;
extern double g_dTRMSC_Peak;
extern double g_dLastTRMSC_RMS;
extern double g_dLastTRMSC_Peak;

extern const char g_cVersStrLong[];
extern char g_cSerInpStr[];

void ParamsUpdateAndDisplay(
    uint8_t ucMenuItem,
    void (*pUpdateDisplayFunction)(uint8_t ucForceDisplay, void *pParam),
    void *DisplayParam);

void SetActivityTimer(uint8_t Value);
void jobActivityTimer(void);

void jobPanel(void);
void SwitchRange(uint8_t ucRange);
double GetTRMSC_RMS(void);
double GetTRMSC_Peak(void);
double GetTRMSC_dBU(void);

extern const uint16_t g_uiTerzArray[];

#endif
