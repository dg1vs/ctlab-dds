/*
 * Copyright (c) 2007 by Thoralt Franz
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
#ifndef PANEL_H_
#define PANEL_H_

#include <inttypes.h>
#include "Lcd.h" // used for inlinne
#include "dds.h"

//----------------------------------------------------------------------------
// LCD formatting constants
//----------------------------------------------------------------------------
#define LCD_TYPE_LEVEL_DB           0
#define LCD_TYPE_LEVEL_VOLT         1
#define LCD_TYPE_OFFSET             2
#define LCD_TYPE_FREQUENCY          3
#define LCD_TYPE_TIME_MS            4
#define LCD_TYPE_DOUBLE             6
#define LCD_TYPE_TIME_100MS         7
#define LCD_TYPE_PERCENT            8
#define LCD_TYPE_COUNT              9

//----------------------------------------------------------------------------
// menu structure
//----------------------------------------------------------------------------
typedef struct
{
    // pointer to function which is called after encoder is turned
    void (*pEncoderFunction)(int16_t iEncoderValue, void *pParam);
    // pointer to function which is called after encoder is pushed
    void (*pEnterFunction)(void);
    // pointer to function which is called to update the parameter display
    void (*pUpdateDisplayFunction)(uint8_t ucForceDisplay, void *pParam);
    // menupointers to submenu, previous or next menuitems
    void *pSubmenu, *pPrevious, *pNext;
    // encoder acceleration values for this parameter
    int16_t iEI1, iEI2, iEI3, iEI4;
    // additional pointer parameter to pass to the encoder function
    void *pParam;
    // Menu identifier for DSP command
    uint8_t ucIdentifier;
    // name to show on LCD
    char cName[];
} menu_entry_t;

//----------------------------------------------------------------------------
// DEFINEs
//----------------------------------------------------------------------------
#define null (void*)0

// time to avoid button bounce (value of 250 = 125 ms)
#define BUTTON_TIME 250

//----------------------------------------------------------------------------
// prototypes for menu actions and menu items
//----------------------------------------------------------------------------
void FrequencyEncoderFunction(int16_t iEncoderPos, void *pParam);
void FrequencyUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void FrequencyEnterFunction(void);

void LevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void LevelEncoderFunction(int16_t iEncoderPos, void *pParam);
void SwitchDBVoltsFunction(void);
void PeakLevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);

void OffsetUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void OffsetEnterFunction(void);

void WaveformUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void WaveformEnterFunction(void);

void LogicLevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void LogicLevelEnterFunction(void);

#ifdef COMPILE_WITH_PWM
void PWMDutyUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void PWMDutyEnterFunction(void);
void PWMDutyCountEnterFunction(void);
void PWMDutyCountUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void PWMDutyPolarityUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void PWMDutyEncoderFunction(int16_t iEncoderPos, void *pParam);
#endif

void TRMSC_RangeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void TRMSC_RangeEncoderFunction(int16_t iEncoderPos, void *pParam);
void TRMSC_LevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void TRMSC_PermanentEncoderFunction(int16_t iEncoderPos, void *pParam);
void TRMSC_PermanentUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);

void Burst_OnOffEncoderFunction(int16_t iEncoderPos, void *pParam);
void Burst_OnOffUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void TimeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);

void IEncoderFunction(int16_t iEncoderPos, void *pParam);
void UCEncoderFunction(int16_t iEncoderPos, void *pParam);

void SweepModeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void SweepMenuUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void SweepMarkerUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void SweepSlopeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void SweepDoubleEncoderFunction(int16_t iEncoderPos, void *pParam);
void SweepDoubleUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
void SweepMarkerModeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);

void SettingsSaveUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam);
//void SettingsSaveEnterFunction(void);

extern const PROGMEM menu_entry_t MenuFrequency, MenuLevel, MenuPeakLevel, MenuOffset,
       MenuWaveform, MenuSweep, MenuBurst, MenuTRMSC, MenuSettings,

       SubmenuLogicHi, SubmenuLogicLo, SubmenuLogicBack,

       SubmenuSweepStart, SubmenuSweepEnd, SubmenuSweepCenter, SubmenuSweepSpanFactor, SubmenuSweepTime,
       SubmenuSweepSlope, SubmenuSweepMarker, SubmenuSweepMarkerMode, SubmenuSweepMarkerHeight, SubmenuSweepMode, SubmenuSweepBack,

       SubmenuBurstOnTime, SubmenuBurstOffTime, SubmenuBurstBack,

       SubmenuTRMSC_Range, SubmenuTRMSC_RMS, SubmenuTRMSC_Peak,
       SubmenuTRMSC_Permanent, SubmenuTRMSC_Back,

#ifdef COMPILE_WITH_PWM
       SubmenuPWMDuty, SubmenuPWMBack, SubmenuPWMFrequency, SubmenuPWMHi, SubmenuPWMLo, SubmenuPWMCount, SubmenuPWMPolarity,
#endif

       SubmenuSettingsSave, SubmenuSettingsPermanentTime,
       SubmenuSettingsScrollSpeed, SubmenuSettingsBack
       ;

extern menu_entry_t *g_currentMenuItem;
extern uint8_t g_ucMenuItemChanged;

#define FREQUENCY_CONT 0
#define FREQUENCY_TERZ 1
extern uint8_t g_ucFrequencyMode;



#ifdef COMPILE_WITH_DISPLAY204              /* PM 20*4 */
#define CURSOR_ARRAY_SIZE 8*8
#else                                       /* PM8 */
#define CURSOR_ARRAY_SIZE 6*8
#endif

extern const PROGMEM uint8_t cursor[CURSOR_ARRAY_SIZE];

#ifdef COMPILE_WITH_DISPLAY204              /* PM 20*4 */
#define EXTRA_EA_DIP_SIGN_TRMSC "\x01"
#define EXTRA_EA_DIP_SIGN_ENTER "\x10"
#define EXTRA_EA_DIP_SIGN_BACK  "\x16"

#define EXTRA_EA_DIP_SIGN_DOWN  "\x1B"
#define EXTRA_EA_DIP_SIGN_UP    "\x1A"
#else                                       /* PM8 */
#define EXTRA_EA_DIP_SIGN_TRMSC "\x04"
#define EXTRA_EA_DIP_SIGN_ENTER "\x05"
#define EXTRA_EA_DIP_SIGN_BACK  "\x05"
#endif


//----------------------------------------------------------------------------
// prototypes
//----------------------------------------------------------------------------
void jobPanel(void);
menu_entry_t* findMenuEntry(uint8_t ucIdentifier);
void Panel_SplashScreen(void);

#endif /*PANEL_H_*/
