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



#include "dds.h"
#include <avr/pgmspace.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "panel.h"
#include "encoder.h"
#include "lcd.h"
#include "dds-hw.h"
#include "timer.h"




#ifdef COMPILE_WITH_DISPLAY204

// can be slow due 20 spaces
#define PANEL_PRINT_CENTER(x)            Lcd_OverWrite_P((COLUMN_MAX - sizeof(x))/2+1, 1, sizeof(x)-1, PSTR((x)));

// LCDOverwrite_P operates in this case over 20 char, this is to much and it will erase the first part of the display line
// so we use the combination of Panel_ClearDataLine, which clears only the second part (all information are right aligned
// and LCDWrite_P and some calculation
#define PANEL_PRINT_CHOICE(x)            Panel_ClearDataLine(); \
                                          Lcd_Write_P(COLUMN_MAX + 1 - sizeof(x), 2, sizeof(x)-1, PSTR((x)));

#define PANEL_PRINT_CHOICE_WITH_ENTER(x) Panel_ClearDataLine(); \
                                          Lcd_Write_P(COLUMN_MAX + 1 - sizeof(x), 2, sizeof(x)-1, PSTR((x))); Panel_PrintEnterChoice();

#define PANEL_PRINT_FIRMWARE_UPDATE(x)

#else

#define PANEL_PRINT_CENTER(x)            Lcd_OverWrite_P((COLUMN_MAX - sizeof(x))/2+1, 0, sizeof(x)-1, PSTR((x)));
#define PANEL_PRINT_CHOICE(x)            Lcd_OverWrite_P(COLUMN_MAX + 1 - sizeof(x), 1, sizeof(x)-1, PSTR((x)));
#define PANEL_PRINT_CHOICE_WITH_ENTER(x) Lcd_OverWrite_P(COLUMN_MAX     - sizeof(x), 1, sizeof(x)-1, PSTR((x))); Panel_PrintEnterChoice();

#define PANEL_PRINT_FIRMWARE_UPDATE(x)   Lcd_OverWrite_P(0, 0, 8, PSTR("FIRMWARE")); LCDOverwrite_P(0, 1, 8, PSTR(" UPDATE "));

#endif

// new functions
static void Panel_PrintBack(void);
static void Panel_PrintEnterChoice(void);
static void Panel_PrintEnter(void);
static void Panel_ResetEnter(void);
static void Panel_ClearDataLine(void);
static void Panel_ClearMenuLine(void);


static void Panel_UpdateStatusLine(void);
static void Panel_ScrollMenuName(menu_entry_t *g_currentMenuItem);

// old functions
static void SettingsSaveEnterFunction(void);
static void Panel_OutputValue(double f, int16_t i, uint8_t uType);

 
const PROGMEM uint8_t cursor[CURSOR_ARRAY_SIZE] =
{

    #ifdef COMPILE_WITH_DISPLAY204
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0 terminated string
    0x00, 0x08, 0x15, 0x02, 0x08, 0x15, 0x02, 0x00,   // 1 TRMSC symbol EXTRA_EA_DIP_SIGN_TRMSC
    0x10, 0x18, 0x1c, 0x1e, 0x1f, 0x1f, 0x1f, 0x1f,   // 2 D oben   + S rechts oben
    0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1c, 0x18, 0x10,   // 3 D unten  + S rechts unten
    0x01, 0x03, 0x07, 0x0f, 0x1f, 0x1f, 0x1f, 0x1f,   // 4 S links oben
    0x1f, 0x1f, 0x1f, 0x1f, 0x0f, 0x07, 0x03, 0x01,   // 5 S links unten
    0x1f, 0x0f, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00,   // 6 S mitte
    0x00, 0x00, 0x00, 0x10, 0x18, 0x1c, 0x1e, 0x1f,   // 7 S mitte
    #else
    0x00, 0x0e, 0x1f, 0x11, 0x1f, 0x0e, 0x00, 0x00,   // 0 ENCODER_NAVIGATION
    0x00, 0x0e, 0x17, 0x1b, 0x1d, 0x0e, 0x00, 0x00,   // 1 ENCODER_NAVIGATION
    0x00, 0x0e, 0x1b, 0x1b, 0x1b, 0x0e, 0x00, 0x00,   // 2 ENCODER_NAVIGATION
    0x00, 0x0e, 0x1d, 0x1b, 0x17, 0x0e, 0x00, 0x00,   // 3 ENCODER_NAVIGATION
    0x00, 0x08, 0x15, 0x02, 0x08, 0x15, 0x02, 0x00,   // 4 TRMSC symbol EXTRA_EA_DIP_SIGN_TRMSC
    0x00, 0x04, 0x06, 0x07, 0x06, 0x04, 0x00, 0x00,   // 5 enter symbol EXTRA_EA_DIP_SIGN_ENTER
    #endif

};

//............................................................................
// main menu
//............................................................................
const PROGMEM menu_entry_t MenuFrequency =
{
    .cName = "Frequency",
    .pEncoderFunction = FrequencyEncoderFunction,
    .pEnterFunction = FrequencyEnterFunction,
    .pUpdateDisplayFunction = FrequencyUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&MenuSettings,
    .pNext = (void*)&MenuLevel,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dFrequency), .ucIdentifier = MENU_FREQUENCY
};

const PROGMEM menu_entry_t MenuLevel =
{
    .cName = "Level",
    .pEncoderFunction = LevelEncoderFunction,
    .pEnterFunction = SwitchDBVoltsFunction,
    .pUpdateDisplayFunction = LevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&MenuFrequency,
    .pNext = (void*)&MenuPeakLevel,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dLevel), .ucIdentifier = MENU_LEVEL_RMS
};

const PROGMEM menu_entry_t MenuPeakLevel =
{
    .cName = "PeakPeakLevel",
    .pEncoderFunction = LevelEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = PeakLevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&MenuLevel,
    .pNext = (void*)&MenuOffset,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dPeakLevel), .ucIdentifier = MENU_LEVEL_PEAK
};

const PROGMEM menu_entry_t MenuOffset =
{
    .cName = "Offset",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = OffsetEnterFunction,
    .pUpdateDisplayFunction = OffsetUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&MenuPeakLevel,
    .pNext = (void*)&MenuWaveform,
    .iEI1 = 200, .iEI2 = 50, .iEI3 = 10, .iEI4 = 5,
    .pParam = &(Params.iOffset), .ucIdentifier = MENU_OFFSET
};

const PROGMEM menu_entry_t MenuWaveform =
{
    .cName = "Waveform",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = WaveformEnterFunction,
    .pUpdateDisplayFunction = WaveformUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&MenuOffset,
    .pNext = (void*)&MenuSweep,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucWaveForm), .ucIdentifier = MENU_WAVEFORM
};

const PROGMEM menu_entry_t MenuSweep =
{
    .cName = "Sweep",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = SweepMenuUpdateDisplayFunction,
    .pSubmenu = (void*)&SubmenuSweepStart,
    .pPrevious = (void*)&MenuWaveform,
    .pNext = (void*)&MenuBurst,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucSweepMenu), .ucIdentifier = MENU_SWEEP
};

const PROGMEM menu_entry_t MenuBurst =
{
    .cName = "Burst",
    .pEncoderFunction = Burst_OnOffEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = Burst_OnOffUpdateDisplayFunction,
    .pSubmenu = (void*)&SubmenuBurstOnTime,
    .pPrevious = (void*)&MenuSweep,
    .pNext = (void*)&MenuTRMSC,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = null, .ucIdentifier = MENU_BURST
};

const PROGMEM menu_entry_t MenuTRMSC =
{
    .cName = "TRMSC",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&SubmenuTRMSC_RMS,
    .pPrevious = (void*)&MenuBurst,
    .pNext = (void*)&MenuSettings,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = null, .ucIdentifier = MENU_TRMSC
};

const PROGMEM menu_entry_t MenuSettings =
{
    .cName = "Settings",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&SubmenuSettingsSave,
    .pPrevious = (void*)&MenuTRMSC,
    .pNext = (void*)&MenuFrequency,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = null, .ucIdentifier = MENU_TRMSC
};

//............................................................................
// Submenu "Settings"
//............................................................................
const PROGMEM menu_entry_t SubmenuSettingsSave =
{
    .cName = "Save",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = SettingsSaveEnterFunction,
    .pUpdateDisplayFunction = SettingsSaveUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSettingsBack,
    .pNext = (void*)&SubmenuSettingsPermanentTime,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucAutosave),
    .ucIdentifier = SUBMENU_SAVE
};

const PROGMEM menu_entry_t SubmenuSettingsPermanentTime =
{
    .cName = "Permanent TRMSC Delay",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TimeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSettingsSave,
    .pNext = (void*)&SubmenuSettingsScrollSpeed,
    .iEI1 = 100, .iEI2 = 100, .iEI3 = 100, .iEI4 = 100,
    .pParam = &(Params.iPermanentTRMSC_Delay),
    .ucIdentifier = SUBMENU_PERMANENTTIME
};

const PROGMEM menu_entry_t SubmenuSettingsScrollSpeed =
{
    .cName = "Display Scroll Speed",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TimeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSettingsPermanentTime,
    .pNext = (void*)&SubmenuSettingsBack,
    .iEI1 = 100, .iEI2 = 100, .iEI3 = 25, .iEI4 = 25,
    .pParam = &(Params.iScrollSpeed),
    .ucIdentifier = SUBMENU_SCROLLSPEED
};

const PROGMEM menu_entry_t SubmenuSettingsBack =
{
    .cName = "    back",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&MenuSettings,
    .pPrevious = (void*)&SubmenuSettingsScrollSpeed,
    .pNext = (void*)&SubmenuSettingsSave,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1, .pParam = null,
    .ucIdentifier = SUBMENU_SETTINGSBACK
};


//............................................................................
// Submenu "TRMSC"
//............................................................................
const PROGMEM menu_entry_t SubmenuTRMSC_RMS =
{
    .cName = "RMS",
    .pEncoderFunction = null,
    .pEnterFunction = SwitchDBVoltsFunction,
    .pUpdateDisplayFunction = TRMSC_LevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuTRMSC_Back,
    .pNext = (void*)&SubmenuTRMSC_Peak,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(g_dTRMSC_RMS), .ucIdentifier = SUBMENU_TRMS_RMS
};

const PROGMEM menu_entry_t SubmenuTRMSC_Peak =
{
    .cName = "PeakPeak",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TRMSC_LevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuTRMSC_RMS,
    .pNext = (void*)&SubmenuTRMSC_Range,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(g_dTRMSC_Peak), .ucIdentifier = SUBMENU_TRMS_PEAK
};

const PROGMEM menu_entry_t SubmenuTRMSC_Range =
{
    .cName = "Range",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TRMSC_RangeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuTRMSC_Peak,
    .pNext = (void*)&SubmenuTRMSC_Permanent,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucRange),
    .ucIdentifier = SUBMENU_TRMS_RANGE
};

const PROGMEM menu_entry_t SubmenuTRMSC_Permanent =
{
    .cName = "Permanent Display",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TRMSC_PermanentUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuTRMSC_Range,
    .pNext = (void*)&SubmenuTRMSC_Back,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucPermanentTRMSC),
    .ucIdentifier = SUBMENU_TRMS_PERMANENT
};

const PROGMEM menu_entry_t SubmenuTRMSC_Back =
{
    .cName = "    back",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&MenuTRMSC,
    .pPrevious = (void*)&SubmenuTRMSC_Permanent,
    .pNext = (void*)&SubmenuTRMSC_RMS,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1, .pParam = null,
    .ucIdentifier = SUBMENU_TRMS_BACK
};

//............................................................................
// Submenu "Logic"
//............................................................................
const PROGMEM menu_entry_t SubmenuLogicHi =
{
    .cName = "Logic Hi",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = LogicLevelEnterFunction,
    .pUpdateDisplayFunction = LogicLevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuLogicBack,
    .pNext = (void*)&SubmenuLogicLo,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 5,
    .pParam = &(Params.iLogicHi), .ucIdentifier = SUBMENU_LOGIC_HIGH
};

const PROGMEM menu_entry_t SubmenuLogicLo =
{
    .cName = "Logic Lo",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = LogicLevelEnterFunction,
    .pUpdateDisplayFunction = LogicLevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuLogicHi,
    .pNext = (void*)&SubmenuLogicBack,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 5,
    .pParam = &(Params.iLogicLo), .ucIdentifier = SUBMENU_LOGIC_LOW
};

const PROGMEM menu_entry_t SubmenuLogicBack =
{
    .cName = "    back",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&MenuWaveform,
    .pPrevious = (void*)&SubmenuLogicLo,
    .pNext = (void*)&SubmenuLogicHi,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1, .pParam = null,
    .ucIdentifier = SUBMENU_LOGIC_BACK
};

#ifdef COMPILE_WITH_PWM
//............................................................................
// Submenu "PWM"
//............................................................................
const PROGMEM menu_entry_t SubmenuPWMDuty =
{
    .cName = "PWM Duty",
    .pEncoderFunction = PWMDutyEncoderFunction,
    .pEnterFunction = PWMDutyEnterFunction,
    .pUpdateDisplayFunction = PWMDutyUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuPWMBack,
    .pNext = (void*)&SubmenuPWMFrequency,
    .iEI1 = 8, .iEI2 = 4, .iEI3 = 2, .iEI4 = 1,
    .pParam = &(Params.iPWMDuty), .ucIdentifier = SUBMENU_PWM_DUTY
};

const PROGMEM menu_entry_t SubmenuPWMFrequency =
{
    .cName = "Frequency",
    .pEncoderFunction = FrequencyEncoderFunction,
    .pEnterFunction = FrequencyEnterFunction,
    .pUpdateDisplayFunction = FrequencyUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuPWMDuty,
    .pNext = (void*)&SubmenuPWMCount,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dFrequency), .ucIdentifier = MENU_FREQUENCY
};

const PROGMEM menu_entry_t SubmenuPWMCount =
{
    .cName = "PWM Pulse Count",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = PWMDutyCountEnterFunction,
    .pUpdateDisplayFunction = PWMDutyCountUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuPWMFrequency,
    .pNext = (void*)&SubmenuPWMPolarity,
    .iEI1 = 8, .iEI2 = 4, .iEI3 = 2, .iEI4 = 1,
    .pParam = &(Params.iPWMimpulses), .ucIdentifier = SUBMENU_PWM_COUNT
};

const PROGMEM menu_entry_t SubmenuPWMPolarity =
{
    .cName = "PWM Pulse Polarity",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = PWMDutyPolarityUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuPWMCount,
    .pNext = (void*)&SubmenuPWMHi,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucPWMpolarity), .ucIdentifier = SUBMENU_PWM_POLARITY
};

const PROGMEM menu_entry_t SubmenuPWMHi =
{
    .cName = "PWM Hi",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = LogicLevelEnterFunction,
    .pUpdateDisplayFunction = LogicLevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuPWMPolarity,
    .pNext = (void*)&SubmenuPWMLo,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 5,
    .pParam = &(Params.iLogicHi), .ucIdentifier = SUBMENU_PWM_HIGH
};

const PROGMEM menu_entry_t SubmenuPWMLo =
{
    .cName = "PWM Lo",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = LogicLevelEnterFunction,
    .pUpdateDisplayFunction = LogicLevelUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuPWMHi,
    .pNext = (void*)&SubmenuPWMBack,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 5,
    .pParam = &(Params.iLogicLo), .ucIdentifier = SUBMENU_PWM_LOW
};

const PROGMEM menu_entry_t SubmenuPWMBack =
{
    .cName = "    back",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&MenuWaveform,
    .pPrevious = (void*)&SubmenuPWMLo,
    .pNext = (void*)&SubmenuPWMDuty,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1, .pParam = null,
    .ucIdentifier = SUBMENU_PWM_BACK
};
#endif

//............................................................................
// Submenu "Sweep"
//............................................................................
const PROGMEM menu_entry_t SubmenuSweepStart =
{
    .cName = "Startfrq",
    .pEncoderFunction = FrequencyEncoderFunction,
    .pEnterFunction = FrequencyEnterFunction,
    .pUpdateDisplayFunction = FrequencyUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepBack,
    .pNext = (void*)&SubmenuSweepEnd,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dSweepStart), .ucIdentifier = SUBMENU_SWEEP_START
};

const PROGMEM menu_entry_t SubmenuSweepEnd =
{
    .cName = "Endfreq",
    .pEncoderFunction = FrequencyEncoderFunction,
    .pEnterFunction = FrequencyEnterFunction,
    .pUpdateDisplayFunction = FrequencyUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepStart,
    .pNext = (void*)&SubmenuSweepCenter,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dSweepEnd), .ucIdentifier = SUBMENU_SWEEP_END
};

const PROGMEM menu_entry_t SubmenuSweepCenter =
{
    .cName = "Centerfrq",
    .pEncoderFunction = FrequencyEncoderFunction,
    .pEnterFunction = FrequencyEnterFunction,
    .pUpdateDisplayFunction = FrequencyUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepEnd,
    .pNext = (void*)&SubmenuSweepSpanFactor,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dSweepCenter), .ucIdentifier = SUBMENU_SWEEP_CENTER
};

const PROGMEM menu_entry_t SubmenuSweepSpanFactor =
{
    .cName = "Frq Span",
    .pEncoderFunction = SweepDoubleEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = SweepDoubleUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepCenter,
    .pNext = (void*)&SubmenuSweepTime,
    .iEI1 = 10, .iEI2 = 10, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.dSweepSpanFactor), .ucIdentifier = SUBMENU_SWEEP_SPANFACTOR
};

const PROGMEM menu_entry_t SubmenuSweepTime =
{
    .cName = "Swp Time",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TimeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepSpanFactor,
    .pNext = (void*)&SubmenuSweepMode,
    .iEI1 = 1000, .iEI2 = 500, .iEI3 = 200,  .iEI4 = 100,
//      .iEI1 = SWEEP_UPDATE_INTERVAL*100, .iEI2 = SWEEP_UPDATE_INTERVAL*10,
//      .iEI3 = SWEEP_UPDATE_INTERVAL*2, .iEI4 = SWEEP_UPDATE_INTERVAL,
    .pParam = &(Params.iSweepTime), .ucIdentifier = SUBMENU_SWEEP_TIME
};

const PROGMEM menu_entry_t SubmenuSweepMode =
{
    .cName = "Swp Mode",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = SweepModeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepTime,
    .pNext = (void*)&SubmenuSweepSlope,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucSweepMode), .ucIdentifier = SUBMENU_SWEEP_MODE
};

const PROGMEM menu_entry_t SubmenuSweepSlope =
{
    .cName = "Sw Slope",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = SweepSlopeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepMode,
    .pNext = (void*)&SubmenuSweepMarker,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucSweepSlope), .ucIdentifier = SUBMENU_SWEEP_SLOPE
};

const PROGMEM menu_entry_t SubmenuSweepMarker =
{
    .cName = "Marker",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = SweepMarkerUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepSlope,
    .pNext = (void*)&SubmenuSweepMarkerMode,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucSweepMarker), .ucIdentifier = SUBMENU_SWEEP_MARKER
};

const PROGMEM menu_entry_t SubmenuSweepMarkerMode =
{
    .cName = "MarkerMode",
    .pEncoderFunction = UCEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = SweepMarkerModeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepMarker,
    .pNext = (void*)&SubmenuSweepMarkerHeight,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.ucSweepMarkerMode), .ucIdentifier = SUBMENU_SWEEP_MARKERMODE
};

const PROGMEM menu_entry_t SubmenuSweepMarkerHeight =
{
    .cName = "MarkerHeight",
    .pEncoderFunction = SweepDoubleEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = SweepDoubleUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuSweepMarkerMode,
    .pNext = (void*)&SubmenuSweepBack,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = &(Params.dSweepMarkerHeight), .ucIdentifier = SUBMENU_SWEEP_MARKERHGHT
};

const PROGMEM menu_entry_t SubmenuSweepBack =
{
    .cName = "    back",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&MenuSweep,
    .pPrevious = (void*)&SubmenuSweepMarkerHeight,
    .pNext = (void*)&SubmenuSweepStart,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1,
    .pParam = null, .ucIdentifier = SUBMENU_SWEEP_BACK
};

//............................................................................
// Submenu "Burst"
//............................................................................
const PROGMEM menu_entry_t SubmenuBurstOnTime =
{
    .cName = "On Time",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TimeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuBurstBack,
    .pNext = (void*)&SubmenuBurstOffTime,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.iBurst1), .ucIdentifier = SUBMENU_BURST1
};

const PROGMEM menu_entry_t SubmenuBurstOffTime =
{
    .cName = "Off Time",
    .pEncoderFunction = IEncoderFunction,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = TimeUpdateDisplayFunction,
    .pSubmenu = null,
    .pPrevious = (void*)&SubmenuBurstOnTime,
    .pNext = (void*)&SubmenuBurstBack,
    .iEI1 = 250, .iEI2 = 50, .iEI3 = 10, .iEI4 = 1,
    .pParam = &(Params.iBurst0), .ucIdentifier = SUBMENU_BURST0
};

const PROGMEM menu_entry_t SubmenuBurstBack =
{
    .cName = "    back",
    .pEncoderFunction = null,
    .pEnterFunction = null,
    .pUpdateDisplayFunction = null,
    .pSubmenu = (void*)&MenuBurst,
    .pPrevious = (void*)&SubmenuBurstOffTime,
    .pNext = (void*)&SubmenuBurstOnTime,
    .iEI1 = 1, .iEI2 = 1, .iEI3 = 1, .iEI4 = 1, .pParam = null,
    .ucIdentifier = SUBMENU_BURST_BACK
};

//----------------------------------------------------------------------------
// globals
//----------------------------------------------------------------------------

// this variable holds a pointer to the current menu item
menu_entry_t *g_currentMenuItem = (void*)&MenuFrequency;

// this flag is set to "1" everytime the menu changes, jobPanel() will then
// show the new menu item on LCD
uint8_t g_ucMenuItemChanged = 1;

// variable to hold the previous button state
uint8_t g_uLastButtons = 0;

// current scrolling positions
uint8_t g_ucScrollOffset = 0;

#ifdef ENCODER_NAVIGATION
// variable for the turning encoder menu marker
uint8_t g_ucEncoderMenuMarker = 0;
#endif

uint8_t g_ucFrequencyMode = FREQUENCY_CONT;

// do text scrolling
void Panel_ScrollMenuName(menu_entry_t *g_currentMenuItem)
{
    uint8_t i, j, ucScrollLength;
    uint16_t u;

    char s[COLUMN_MAX+1];


    ATOMIC_RW(u, xg_uScrollTimer);
    if ((0 == u) || (u > 0xF000))
    {
        // if text has been reset, the timer is counting downwards from 0xFFFF
        // -> wait longer if the text has just been updated, this causes
        // a small delay before scrolling begins
        if (u > 0xF000)
        {
            ATOMIC_RW(xg_uScrollTimer, (Params.iScrollSpeed<<1)*4);
        }
        else
        {
            ATOMIC_RW(xg_uScrollTimer, (Params.iScrollSpeed<<1)); //+
        }

#ifdef ENCODER_NAVIGATION
#ifdef COMPILE_WITH_DISPLAY204
        ucScrollLength = (0 == g_ucEncoderMenuMarker) ? (COLUMN_MAX/2) : ((COLUMN_MAX/2) - 1);
#else
        ucScrollLength = (0 == g_ucEncoderMenuMarker) ? (COLUMN_MAX) : ((COLUMN_MAX) - 1);
#endif
#else
#ifdef COMPILE_WITH_DISPLAY204
        ucScrollLength = COLUMN_MAX/2;
#else
        ucScrollLength = COLUMN_MAX;
#endif
#endif
        // do not scroll if length < 9 (or 8, if encoder scrolling is active)
        if (strlen_P(g_currentMenuItem->cName) <= ucScrollLength)
        {
            // output menu item
            Panel_ClearMenuLine();
#ifdef COMPILE_WITH_DISPLAY204

            Lcd_Write_P(0, 2, strlen_P(g_currentMenuItem->cName), g_currentMenuItem->cName);
#else
            Lcd_OverWrite_P(0, 0, strlen_P(g_currentMenuItem->cName), g_currentMenuItem->cName);
#endif
        }
        else
        {
            // create scrolling text for LCD
            j = g_ucScrollOffset;
            for (i = 0; i < ucScrollLength; i++)
            {
                if (j >= (strlen_P(g_currentMenuItem->cName) + ucScrollLength))
                    j = 0;

                if (j < strlen_P(g_currentMenuItem->cName))
                    s[i] = pgm_read_byte(g_currentMenuItem->cName + j);
                else
                    s[i] = 0x020;
                j++;
            }
#ifdef COMPILE_WITH_DISPLAY204
            Lcd_Write(0, 2, ucScrollLength, s);
#else
            Lcd_Write(0, 0, ucScrollLength, s);
#endif
            g_ucScrollOffset++;
            // add 8 spaces at the end
            if (g_ucScrollOffset >= (strlen_P(g_currentMenuItem->cName) + COLUMN_MAX))
                g_ucScrollOffset = 0;
        }
    }
}

//----------------------------------------------------------------------------
// jobPanel
//
// Read buttons and encoder input, change appropriate parameters, write to  Panel.
//
// This function is called repeatedly.
//
// -> --
// <- --
//----------------------------------------------------------------------------
void jobPanel(void)
{
    uint8_t uButtons;
    int16_t iEncoderPos;
    uint16_t u;
    menu_entry_t *m;
#ifdef ENCODER_NAVIGATION
    // TODO warum 9, Sinn
    char s[9];
#endif

    // variables to receive function pointers
    void (*EncoderFunction)(int16_t iEncoderValue, void *pParam);
    void (*EnterFunction)(void);
    void (*UpdateDisplayFunction)(uint8_t ucForceUpdate, void *pParam);

    /*  uncomment this to test encoder acceleration
        iEncoderPos = GetAndResetEncPos();
        SetEncoderIncrements(4, 3, 2, 1);
        if(iEncoderPos)
        {
            printf_P(PSTR("%8i\n"), iEncoderPos);
        };
        return;
    */

#ifdef ENCODER_NAVIGATION
    // reset menu mode to normal if necessary
    if(0==g_uEncoderMenuTimer) g_ucEncoderMenuMarker = 0;
#endif

    // check buttons (only if button bounce timer reached zero)
    ATOMIC_RW(u, xg_uButtonTimer);
    if(0==u)
    {
        uButtons = Lcd_GetButton();

#ifdef FIRMWARE_UPDATE
        // firmware update mode if all buttons are pressed
        if((uButtons&(BUTTON_ENTER|BUTTON_UP|BUTTON_DOWN))==(BUTTON_ENTER|BUTTON_UP|BUTTON_DOWN))
        {
            PANEL_PRINT_FIRMWARE_UPDATE();

            // jump to flashloader
            asm volatile("ldi r30,0x00\n");
            asm volatile("ldi r31,0x3F\n");
            asm volatile("ijmp\n");
        }
#endif

#ifdef COMPILE_WITH_DISPLAY204
        // BUTTON_SOFT_KEY_1? (and previous not BUTTON_SOFT_KEY_1)
        if ((uButtons & BUTTON_SOFT_KEY_1) && (!(g_uLastButtons & BUTTON_SOFT_KEY_1)))
        {
            ATOMIC_RW(xg_uButtonTimer, BUTTON_TIME);

            // switch off TRMSC for a short time
            ATOMIC_RW(xg_uPermanentTRMSC_Timer, (Params.iPermanentTRMSC_Delay<<1)); //+
            // reset to normal mode
            initParams();
        }
#endif

        // up? (and previous not up)
        if((uButtons&BUTTON_UP)&&(!(g_uLastButtons&BUTTON_UP)))
        {
            ATOMIC_RW(xg_uButtonTimer, BUTTON_TIME);

            // switch off TRMSC for a short time
            ATOMIC_RW(xg_uPermanentTRMSC_Timer,
                      (Params.iPermanentTRMSC_Delay<<1)); //+

            // if possible, go back to previous menu item
            m = (void*)pgm_read_word(&g_currentMenuItem->pPrevious);
            if(null!=m)
            {
                g_currentMenuItem = m;
                g_ucMenuItemChanged = 1;
            }

#ifdef ENCODER_NAVIGATION
            // switch on encoder menu mode
            g_ucEncoderMenuMarker = 1;
            g_uEncoderMenuTimer = ENCODER_MENU_TIME;
#endif
        }

        // down? (and previous not down)
        if((uButtons&BUTTON_DOWN)&&(!(g_uLastButtons&BUTTON_DOWN)))
        {
            ATOMIC_RW(xg_uButtonTimer, BUTTON_TIME);

            // switch off TRMSC for a short time
            ATOMIC_RW(xg_uPermanentTRMSC_Timer,
                      (Params.iPermanentTRMSC_Delay<<1)); //+

            // if possible, advance to next menu item
            m = (void*)pgm_read_word(&g_currentMenuItem->pNext);
            if(null!=m)
            {
                g_currentMenuItem = m;
                g_ucMenuItemChanged = 1;
            }

#ifdef ENCODER_NAVIGATION
            // switch on encoder menu mode
            g_ucEncoderMenuMarker = 1;
            g_uEncoderMenuTimer = ENCODER_MENU_TIME;
#endif
        }

        // enter? (and previous not enter)
        if((uButtons&BUTTON_ENTER)&&(!(g_uLastButtons&BUTTON_ENTER)))
        {
            ATOMIC_RW(xg_uButtonTimer, BUTTON_TIME);

            ATOMIC_RW(xg_uPermanentTRMSC_Timer,
                      (Params.iPermanentTRMSC_Delay<<1)); //+

            EnterFunction =
                (void*)pgm_read_word(&g_currentMenuItem->pEnterFunction);

#ifdef ENCODER_NAVIGATION
            // if encoder menu changing was on, reset it now and do
            // not start anything yet
            if(g_ucEncoderMenuMarker)
            {
                g_ucEncoderMenuMarker = 0;
            }
            else
#endif
            {
                // if a submenu was defined, switch to this submenu
                m = (void*)pgm_read_word(&g_currentMenuItem->pSubmenu);
                if(null!=m)
                {
                    g_currentMenuItem = m;
                    g_ucMenuItemChanged = 1;
                }
                else
                    // if an "Enter" function was defined, call it now
                {
                    if(null!=EnterFunction) EnterFunction();
                }
            }
        }

        // save current button state
        g_uLastButtons = uButtons;
    }

    // Update Parameter for DSP-command
    LastParams.ucDisplayedMenu = Params.ucDisplayedMenu =
                                     pgm_read_byte(&g_currentMenuItem->ucIdentifier); //0

    // get the encoder function and update function
    EncoderFunction =
        (void*)pgm_read_word(&g_currentMenuItem->pEncoderFunction);
    UpdateDisplayFunction =
        (void*)pgm_read_word(&g_currentMenuItem->pUpdateDisplayFunction);

    // display new menu item if necessary and set new encoder
    // acceleration values
    if(g_ucMenuItemChanged)
    {

        // reset the chars which shows the usage of enter key of the encoder
        Panel_ResetEnter();

        SetActivityTimer(75);
        g_ucMenuItemChanged = 0;

        // force output parameter name to LCD
        ATOMIC_RW(xg_uScrollTimer, 0xFFFF);
        g_ucScrollOffset = 0;

        // set new acceleration values
        SetEncoderAcceleration(pgm_read_word(&g_currentMenuItem->iEI1),
                               pgm_read_word(&g_currentMenuItem->iEI2),
                               pgm_read_word(&g_currentMenuItem->iEI3),
                               pgm_read_word(&g_currentMenuItem->iEI4));

        // also update parameter display
        if(null!=UpdateDisplayFunction)
            UpdateDisplayFunction(1,
                                  (void*)pgm_read_word(&g_currentMenuItem->pParam));

        // if there's no EncoderFunction, display triangle to indicate if there
        // is a submenu or output a blank line if not
        else
        {
            if(null!=(void*)pgm_read_word(&g_currentMenuItem->pSubmenu))
            {
                // here we choice between enter and back
                if(strcmp_P("    back",g_currentMenuItem->cName) == 0)
                {
                    Panel_ClearDataLine();
                    Panel_PrintBack();
                }
                else
                {
                    Panel_ClearDataLine();
                    Panel_PrintEnter();
                }
            }
            else
            {
                // not used yet see http://thoralt.ehecht.com/phpbb/viewtopic.php?f=2&t=439
                Lcd_ClearLine(0);
            }
        }
    }

    Panel_ScrollMenuName(g_currentMenuItem);

#ifdef ENCODER_NAVIGATION
    // show encoder turn symbol?
    if(g_ucEncoderMenuMarker&&(0==g_uTurnSymbolTimer))
    {
        g_uTurnSymbolTimer = 500;
        s[0] = g_ucEncoderMenuMarker - 1;
        if(++g_ucEncoderMenuMarker>4) g_ucEncoderMenuMarker = 1;
#ifdef COMPILE_WITH_DISPLAY204
        Lcd_Write( 9, 3, 1, s);
        Lcd_Write(10, 3, 1, s);
#else
        Lcd_Write( 7, 0, 1, s);
#endif
    }
#endif

    // call the function which updates the Display for the current parameter
    // LCD will only be written if parameter changed
    if(null!=UpdateDisplayFunction)
        UpdateDisplayFunction(0,
                              (void*)pgm_read_word(&g_currentMenuItem->pParam));

    // read encoder position
    // current parameter is changed if necessary
    iEncoderPos = GetAndResetEncPos();
    if(iEncoderPos)
    {
        // reset timer for permanent TRMSC display to allow parameter
        // display for a short time
        ATOMIC_RW(xg_uPermanentTRMSC_Timer,
                  (Params.iPermanentTRMSC_Delay<<1)); //+

#ifdef ENCODER_NAVIGATION
        // if in encoder menu mode, use the encoder to scroll through
        // menu
        if(g_ucEncoderMenuMarker)
        {
            if(iEncoderPos<0)
            {
                // if possible, go back to previous menu item
                m = (void*)pgm_read_word(&g_currentMenuItem->pPrevious);
                if(null!=m)
                {
                    g_currentMenuItem = m;
                    g_ucMenuItemChanged = 1;
                }
            }
            else
            {
                // if possible, advance to next menu item
                m = (void*)pgm_read_word(&g_currentMenuItem->pNext);
                if(null!=m)
                {
                    g_currentMenuItem = m;
                    g_ucMenuItemChanged = 1;
                }
            }
            g_uEncoderMenuTimer = ENCODER_MENU_TIME;
        }
        // use the encoder to change values
        else
#endif
        {
            // call encoder function if it is not null
            if(null!=EncoderFunction)
                EncoderFunction(iEncoderPos,
                                (void*)pgm_read_word(&g_currentMenuItem->pParam));
            CheckLimits();
            if(null!=UpdateDisplayFunction)
                UpdateDisplayFunction(0,
                                      (void*)pgm_read_word(&g_currentMenuItem->pParam));

        }
    }

    // display permanent TRMSC value if necessary
    ATOMIC_RW(u, xg_uPermanentTRMSC_Timer);
    if((0==u) && Params.ucPermanentTRMSC && ((g_currentMenuItem != (void*)&SubmenuTRMSC_RMS) && (g_currentMenuItem != (void*)&SubmenuTRMSC_Peak)))
    {
        if(PERMANENT_TRMSC_RMS==Params.ucPermanentTRMSC)
            TRMSC_LevelUpdateDisplayFunction(1, &g_dTRMSC_RMS);
        else TRMSC_LevelUpdateDisplayFunction(1, &g_dTRMSC_Peak);
    }
    Panel_UpdateStatusLine();
}


//----------------------------------------------------------------------------
// RoundPrecision
//
// Rounds a value to a multiple of a selectable precision
//
// -> *d: Pointer to number to be rounded
//    prec: Precision which is used to round (0.1, 1, 10, ...)
// <- The result is placed in input parameter *d
//----------------------------------------------------------------------------
void RoundPrecision(double *d, double prec)
{
    int16_t i;

    i = (*d / prec) + 0.5f;
    *d = i * prec;
}

//----------------------------------------------------------------------------
// TerzDecode
//
// Converts Terz frequency codes to double values
//
// -> code: Terz frequency code
// <- frequency value corresponding to code
//----------------------------------------------------------------------------
double TerzDecode(uint16_t code)
{
// Coding for frequencies: (ui_TerzArray[x]>>4 ) * 10 ^ ( int16_t)(ui_TerzArray[x] & 0x000F ) - 2)
// 3 MSB nibbles: frequency f
// LSB nibble: exponent e  (with offset 2)
// frequency = f*10^(e-2)

    return  (double) ( code >> 4 ) *  pow( 10 , ( (int8_t)( code & 0x000F ) - 2 )) ;
}

//----------------------------------------------------------------------------
// TerzDecode
//
// Find next Terz frequency
// -> *pdFrequency: frequency
// -> UpDown:       Up: > 0 , Down: <0, Conversion to Terz = 0
// <- *pdFrequency: frequency
//----------------------------------------------------------------------------


void FindTerz(double *pdFrequency, int16_t UpDown)
{
    uint8_t  i = 0;
    uint8_t  t = 0;
    uint16_t c;

    // based on current frequency, select next frequency from list
    while ( (c = pgm_read_word(&g_uiTerzArray[i])) != 0)
    {

        if ( UpDown == 0)
        {
            t = i;

            if ( TerzDecode(c) >= *pdFrequency )
                break;
        }
        else if ( UpDown > 0)
        {
            t = i;

            if ( TerzDecode(c) > *pdFrequency )
                break;
        }
        else
        {
            if ( TerzDecode(c) >= *pdFrequency )
                break;
            t = i;
        }

        i++;
    }

    *pdFrequency = TerzDecode(pgm_read_word(&g_uiTerzArray[t]));
}

//----------------------------------------------------------------------------
// FrequencyEncoderFunction
//
// Gets called when encoder is turned while frequency menu item (or sweep
// start/end frequency submenu item) was active
//
// -> iEncoderPos = encoder value difference since last call (positive or
//                  negative)
//    pParam: --
// <- --
//----------------------------------------------------------------------------
void FrequencyEncoderFunction(int16_t iEncoderPos, void *pParam)
{
    double *d = (double*)pParam;
    double dIncrement;

#ifdef COMPILE_WITH_PWM
    uint16_t c;
    uint16_t uPrescaler;
    double dTemp = *((double*)pParam);
#endif

    if (g_ucFrequencyMode == FREQUENCY_CONT)
    {
        // to make range transistions more smooth,
        // but wait for more memory
        if(iEncoderPos >= 0)
        {
            // scale the increment/decrement appropriately
            if(*d<100.0f)
                dIncrement = 0.01f;
            else if(*d<1000.0f)
                dIncrement = 0.1f;
            else if(*d<10000.0f)
                dIncrement = 1.0f;
            else if(*d<100000.0f)
                dIncrement = 10.0f;
            else
                dIncrement = 100.0f;
        }
        else
        {
            if(*d<=100.05f)
                dIncrement = 0.01f;
            else if(*d<=1000.5f)
                dIncrement = 0.1f;
            else if(*d<=10005.0f)
                dIncrement = 1.0f;
            else if(*d<=10050.0f)
                dIncrement = 10.0f;
            else
                dIncrement = 100.0f;
        }

        // calculate new desired frequency
        *d += (double)iEncoderPos * dIncrement;
        RoundPrecision(d, dIncrement);

    }
    else // Terz-Mode
    {
        FindTerz(d, iEncoderPos);
    }

// TODO: PWM frequency quantization handling in Terz Mode? Fine Tuning necessary ;-))


#ifdef COMPILE_WITH_PWM

    // when using PWM, frequency values have to be quantized, because timer
    // resolution for PWM output depends on prescaler and timer value
    if(Params.ucWaveForm==WAVE_PWM)
    {
        // get prescaler corresponding to d=frequency into uPrescaler
        SetPWMPrescaler(d, &uPrescaler);

        // get the timer value for frequency and calculated prescaler
        c = CalculatePWMTimerValue(*d, uPrescaler);

        // calculate back to frequency using timer value and prescaler
        // due to integer mathematics rounding appears which is responsible
        // for the quantization
        dTemp = CalculatePWMFrequencyValue(c, uPrescaler);

        // Did the user input an increasing value?
        if (iEncoderPos >= 0)
        {
            // if quantized frequency is not greater than previous value,
            // choose next higher frequency value (next smaller timer value)
            if(dTemp<=*d)
            {
                dTemp = CalculatePWMFrequencyValue(--c, uPrescaler);
            }
        }
        // Did the user input a decreasing value?
        else
        {
            // if quantized frequency is not less than previous value,
            // choose next smaller frequency value (next higher timer value)
            if(dTemp>=*d)
            {
                dTemp = CalculatePWMFrequencyValue(++c, uPrescaler);
            }
        }

        // save the quantized value
        *d = dTemp;
    }
#endif


}

//----------------------------------------------------------------------------
// SweepDoubleEncoderFunction
//
// Gets called when encoder is turned while frequency menu item (or sweep
// start/end frequency submenu item) was active
//
// -> iEncoderPos = encoder value difference since last call (positive or
//                  negative)
//    pParam: --
// <- --
//----------------------------------------------------------------------------
void SweepDoubleEncoderFunction(int16_t iEncoderPos, void *pParam)
{
    double *d = (double*)pParam;

    // scale the increment/decrement appropriately

    *d += (double)iEncoderPos * 0.01f;
}

//----------------------------------------------------------------------------
// FrequencyUpdateDisplayFunction
//
// Gets called periodically to update frequency display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void FrequencyUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    double *d = (double*)pParam;

    // check all frequency values for update because this function displays
    // main frequency, sweep start and sweep end frequency
    if((pLastParams->dFrequency!=pParams->dFrequency)||
            (pLastParams->dSweepStart!=pParams->dSweepStart)||
            (pLastParams->dSweepEnd!=pParams->dSweepEnd)||
            (pLastParams->dSweepCenter!=pParams->dSweepCenter)||
            ucForceUpdate)
    {
        // output parameter to Panel
        Panel_OutputValue(*d, 0, LCD_TYPE_FREQUENCY);
    }
}

//----------------------------------------------------------------------------
// FrequencyEnterFunction
//
// Gets called when encoder is pushed while waveform menu item was active
//
// -> --
// <- --
//----------------------------------------------------------------------------

void FrequencyEnterFunction()
{

    // toggle frequency selection between continuous selection and terz selections.
    if (g_ucFrequencyMode == FREQUENCY_CONT)
    {
        g_ucFrequencyMode = FREQUENCY_TERZ;


        FindTerz(&Params.dFrequency, 0);
        FindTerz(&Params.dSweepStart, 0);
        FindTerz(&Params.dSweepEnd, 0);
        // Params.dSweepCenter is intentionally not set to Terz, it will be calculated by CrossCalcSweep.
        // PWM frequency is identical to Params.dFrequency
    }
    else
    {
        g_ucFrequencyMode = FREQUENCY_CONT;
    }

}

//----------------------------------------------------------------------------
// SweepDoubleUpdateDisplayFunction
//
// Gets called periodically to update frequency display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void SweepDoubleUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    double *d = (double*)pParam;

    // check all frequency values for update because this function displays
    // main frequency, sweep start and sweep end frequency
    if( (LastParams.dSweepMarkerHeight!=Params.dSweepMarkerHeight)||
            (LastParams.dSweepSpanFactor!=Params.dSweepSpanFactor)||
            ucForceUpdate)
    {
        // output parameter to Panel
        Panel_OutputValue(*d, 0, LCD_TYPE_DOUBLE);
    }
}

//----------------------------------------------------------------------------
// LevelEncoderFunction
//
// Gets called when encoder is turned while level menu item was active
//
// -> iEncoderPos = encoder value difference since last call (positive or
//                  negative)
//    pParam: --
// <- --
//----------------------------------------------------------------------------
void LevelEncoderFunction(int16_t iEncoderPos, void *pParam )
{
    double *d = (double*)pParam;
    double dIncrement;


    if ( iEncoderPos >= 0)  // to make range transitions more smooth, but wait for more memory
    {
        // above 200 mV: use steps of 1 mV
        if( *d > 200.0f)
        {
            dIncrement = 1.0f;
        }
        // else use steps of 1/40 mV
        else
        {
            dIncrement = 0.025f;
        }

    }
    else
    {
        // above 200 mV: use steps of 1 mV
        if( *d >= 200.1f)
        {
            dIncrement = 1.0f;
        }
        // else use steps of 1/40 mV
        else
        {
            dIncrement = 0.025f;
        }
    }

    *d += (double)iEncoderPos * dIncrement;

    RoundPrecision(d, dIncrement);
}

//----------------------------------------------------------------------------
// LevelUpdateDisplayFunction
//
// Gets called periodically to update level display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void LevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    double *d = (double*)pParam;

    if((LastParams.dLevel!=*d) || ucForceUpdate )
    {
        // output parameter to Panel
        if(Params.ucVolt_dB)
            Panel_OutputValue(*d * 0.001f, 0, LCD_TYPE_LEVEL_DB);
        else
            Panel_OutputValue(*d * 0.001f, 0, LCD_TYPE_LEVEL_VOLT);
    }
}

//----------------------------------------------------------------------------
// PeakLevelUpdateDisplayFunction
//
// Gets called periodically to update level display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void PeakLevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    double *d = (double*)pParam;

    if((LastParams.dPeakLevel != *d) || ucForceUpdate )
    {
        Panel_OutputValue(*d * 0.001f, 0, LCD_TYPE_LEVEL_VOLT);
    }
}

//----------------------------------------------------------------------------
// SwitchDBVoltsFunction
//
// Gets called when encoder is pushed while level menu, TRMSC RMS, TRMSC peak
// or similar item was active
//
// -> --
// <- --
//----------------------------------------------------------------------------
void SwitchDBVoltsFunction()
{
    // change "dB" or "Volts" setting
    if(Params.ucVolt_dB)
        Params.ucVolt_dB = 0;
    else
        Params.ucVolt_dB = 1;

    // force update of the parameter
    void (*UpdateFunction)(uint8_t ucForceUpdate, void *pParam) =
        (void*)pgm_read_word(&g_currentMenuItem->pUpdateDisplayFunction);

    void (*pParameter) = (void*)pgm_read_word(&g_currentMenuItem->pParam);

    if((null != UpdateFunction) && (null != pParameter))
        UpdateFunction(1, pParameter);
}

//----------------------------------------------------------------------------
// OffsetUpdateDisplayFunction
//
// Gets called periodically to update offset display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void OffsetUpdateDisplayFunction(uint8_t ucForceUpdate,
                                 void *pParam __attribute__((unused)))
{
    if((LastParams.iOffset!=Params.iOffset)||ucForceUpdate)
    {
        // output parameter to Panel
        Panel_OutputValue(0, Params.iOffset, LCD_TYPE_OFFSET);
    }
}

//----------------------------------------------------------------------------
// OffsetEnterFunction
//
// Gets called when encoder is pushed while offset menu item was active
//
// -> --
// <- --
//----------------------------------------------------------------------------
void OffsetEnterFunction()
{
    static int16_t s_iOffsetToggleValue = 0.0f;

    // toggle offset and zero (only via encoder on panel)
    if (pParams->iOffset != 0)
    {
        s_iOffsetToggleValue = pParams->iOffset;
        pParams->iOffset = 0;
    }
    else
    {
        pParams->iOffset = s_iOffsetToggleValue;
    }

    // re-display the parameter by calling the encoder function with
    // argument "0"
    void (*EncoderFunction)(int16_t iEncoderValue) =
        (void*)pgm_read_word(&g_currentMenuItem->pEncoderFunction);
    if(null!=EncoderFunction) EncoderFunction(0);
}

//----------------------------------------------------------------------------
// WaveformUpdateDisplayFunction
//
// Gets called periodically to update waveform display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void WaveformUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if((LastParams.ucWaveForm!=Params.ucWaveForm)||ucForceUpdate)
    {
        // display new setting


        // due different position in the Big-display the cursor information has to be reset
        // TODO check memory consumption
        Panel_ResetEnter();

        switch(Params.ucWaveForm)
        {
            case WAVE_OFF:
                 PANEL_PRINT_CHOICE("Off");
                break;
            case WAVE_SINE:
                PANEL_PRINT_CHOICE("Sine");
                break;
            case WAVE_TRIANGLE:
                PANEL_PRINT_CHOICE("Triangle");
                break;
            case WAVE_SQUARE:
                PANEL_PRINT_CHOICE("Square");
                break;
            case WAVE_LOGIC:
                PANEL_PRINT_CHOICE_WITH_ENTER("Logic");
                break;
#ifdef COMPILE_WITH_PWM
            case WAVE_PWM:
                PANEL_PRINT_CHOICE_WITH_ENTER("PWM");
                break;
#else
            case WAVE_EXT_IN:
                PANEL_PRINT_CHOICE("Ext. In");
                break;
#endif
        }
    }
}

//----------------------------------------------------------------------------
// WaveformEnterFunction
//
// Gets called when encoder is pushed while waveform menu item was active
//
// -> --
// <- --
//----------------------------------------------------------------------------
void WaveformEnterFunction()
{
    // if "logic" is selected, go to logic settings menu item
    if(WAVE_LOGIC==Params.ucWaveForm)
    {
        g_currentMenuItem = (void*)&SubmenuLogicHi;
        g_ucMenuItemChanged = 1;
    }
#ifdef COMPILE_WITH_PWM
    else if(WAVE_PWM==Params.ucWaveForm)
    {
        g_currentMenuItem = (void*)&SubmenuPWMDuty;
        g_ucMenuItemChanged = 1;
    }
#endif
}

//----------------------------------------------------------------------------
// Burst_OnOffEncoderFunction
//
// Gets called when encoder is turned while burst on/off submenu item
// was active
//
// -> iEncoderPos = encoder value difference since last call (positive or
//                  negative)
//    pParam: --
// <- --
//----------------------------------------------------------------------------
void Burst_OnOffEncoderFunction(int16_t iEncoderPos,
                                void *pParam __attribute__((unused)))
{
    // change setting
    // special handling for manual change with encoder (as opposed to values sent via uart):

    // leave pParams->iBurstOnOff > 0 as is, if iEncoderPos positive
    if ((pParams->iBurstOnOff == 0) && (iEncoderPos > 0))
    {
        pParams->iBurstOnOff = 1;
    }

    // change pParams->iBurstOnOff to 0 if iEncoderPos negative
    if ((pParams->iBurstOnOff != 0) && (iEncoderPos < 0))
    {
        pParams->iBurstOnOff = 0;
    }

}

//----------------------------------------------------------------------------
// Burst_OnOffUpdateDisplayFunction
//
// Gets called periodically to update burst on/off display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void Burst_OnOffUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if((LastParams.iBurstOnOff!=Params.iBurstOnOff)||ucForceUpdate)
    {
        if(Params.iBurstOnOff)
        {
            PANEL_PRINT_CHOICE_WITH_ENTER("On");
        }
        else
        {
            PANEL_PRINT_CHOICE_WITH_ENTER("Off");
        }
    }
}


//----------------------------------------------------------------------------
// IEncoderFunction
//
// Gets called when encoder is turned while
//
// -> iEncoderPos = encoder value difference since last call (positive or
//                  negative)
//    pParam: pointer to Params.Burst0 or Params.Burst1 or Params.SweepTime
// <- --
//----------------------------------------------------------------------------
void IEncoderFunction(int16_t iEncoderPos, void *pParam)
{
    int16_t *i = (int16_t*)pParam;

    // change setting
    *i += iEncoderPos;
}

//----------------------------------------------------------------------------
// UCEncoderFunction
//
// Gets called when encoder is turned while in Menu
// for non-negative values
//----------------------------------------------------------------------------
void UCEncoderFunction(int16_t iEncoderPos, void *pParam)
{
    uint8_t *uc = (uint8_t*)pParam;

    // don't make the value "negative"
    if ((*uc == 0) && (iEncoderPos < 0))
    {
        return;
    }
    // change setting
    *uc += iEncoderPos;
}

//----------------------------------------------------------------------------
// TimeUpdateDisplayFunction
//
// Gets called periodically to update time display
//
//    pParam: pointer to Params.Burst0, Params.Burst1, Params.SweepTime,
//            Params.uPermanentTRMSC_Delay or Params.uScrollSpeed
// <- --
//----------------------------------------------------------------------------
void TimeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    int16_t *i = (int16_t*)pParam;

    if((g_currentMenuItem==&SubmenuSweepTime)&&
            ((LastParams.iSweepTime!=pParams->iSweepTime) || ucForceUpdate))
    {
        Panel_OutputValue(0, *i, LCD_TYPE_TIME_100MS);
    }
    else if((LastParams.iBurst0!=pParams->iBurst0)||
            (LastParams.iBurst1!=pParams->iBurst1)||
            (LastParams.iScrollSpeed!=pParams->iScrollSpeed)||
            (LastParams.iPermanentTRMSC_Delay!=pParams->iPermanentTRMSC_Delay)||
            ucForceUpdate)
    {
        Panel_OutputValue(0, *i, LCD_TYPE_TIME_MS);
    }
}

//----------------------------------------------------------------------------
// LogicLevelUpdateDisplayFunction
//
// Gets called periodically to update logic-hi and logic-lo display
//
// -> pParam: pointer to either LogicLo or LogicHi
// <- --
//----------------------------------------------------------------------------
void LogicLevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    int16_t *i = pParam;

    if((LastParams.iLogicHi!=Params.iLogicHi)||
            (LastParams.iLogicLo!=Params.iLogicLo)||ucForceUpdate)
    {
        // output parameter to Panel
        Panel_OutputValue(0, (double)*i, LCD_TYPE_OFFSET);
    }
}

//----------------------------------------------------------------------------
// LogicLevelEnterFunction
//
// Gets called when encoder is pushed while logic hi or logic lo menu item
// was active
//
// -> --
// <- --
//----------------------------------------------------------------------------
void LogicLevelEnterFunction()
{
    Params.iLogicHi = 5000;
    Params.iLogicLo = 0;
}

#ifdef COMPILE_WITH_PWM

//----------------------------------------------------------------------------
// PWMDutyUpdateDisplayFunction
//
// Gets called periodically to update PWM Duty display
//
// -> pParam: pointer to PWM Duty
// <- --
//----------------------------------------------------------------------------
void PWMDutyUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    uint16_t *i = pParam;

    if((LastParams.iPWMDuty!=Params.iPWMDuty)||ucForceUpdate)
    {
        // output parameter to Panel
        Panel_OutputValue(0, *i, LCD_TYPE_PERCENT);
    }
}

//----------------------------------------------------------------------------
// PWMDutyCountUpdateDisplayFunction
//
// Gets called periodically to update PWM Duty count display
//
// -> pParam: pointer to PWM Duty
// <- --
//----------------------------------------------------------------------------
void PWMDutyCountUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    uint16_t *i = pParam;

    if((LastParams.iPWMimpulses!=Params.iPWMimpulses)||ucForceUpdate)
    {
        // output parameter to Panel
        if (*i == 0)
        {
            PANEL_PRINT_CHOICE("continu.");
        }
        else
        {
            Panel_OutputValue(0, *i, LCD_TYPE_COUNT);
        }
    }
}

//----------------------------------------------------------------------------
// PWMDutyPolarityUpdateDisplayFunction
//
// Gets called periodically to update PWM Duty polarity display
//
// -> pParam: pointer to PWM Duty
// <- --
//----------------------------------------------------------------------------
void PWMDutyPolarityUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    uint16_t *i = pParam;

    if((LastParams.ucPWMpolarity!=Params.ucPWMpolarity)||ucForceUpdate)
    {
        // output parameter to Panel
        if (*i == 0)
        {
            PANEL_PRINT_CHOICE("non-inv.");
        }
        else
        {
            PANEL_PRINT_CHOICE("    inv.");
        }
    }
}

//----------------------------------------------------------------------------
// PWMDutyEncoderFunction
//
// Gets called when encoder is turned while PWM duty menu item was active
//
// -> iEncoderPos = encoder value difference since last call (positive or
//                  negative)
//    pParam: --
// <- --
//----------------------------------------------------------------------------
void PWMDutyEncoderFunction(int16_t iEncoderPos, void *pParam)
{
    uint16_t uOCR1B, uOldPWM;
    uint32_t uTemp;
    uint16_t *uPWM = (uint16_t*)pParam;

    uOldPWM = *uPWM;

    // change current PWM duty to new value
    *uPWM += iEncoderPos;
    if(*uPWM > 65000)
    {
        *uPWM = 1;  // in case of rollover to "negative", stay at 0.1% duty cycle
    }

    // calculate new switch point (use 32 bit variable to avoid overflow) to 0.1 fraction of percent
    uTemp = ICR1;
    uTemp *= *uPWM;
    uTemp /= 1000;

    uOCR1B = uTemp;

    // calculate back to quantized PWM duty
    uTemp *= 1000;
    uTemp /= (uint32_t)(ICR1);

    // Did the user input an increasing value?
    if(iEncoderPos >= 0)
    {
        // if quantized PWM duty is not greater than previous value,
        // choose next higher value (next smaller timer value)
        if(uTemp<=uOldPWM)
        {
            // get next higher value
            uOCR1B++;
        }
    }
    // Did the user input a decreasing value?
    else
    {
        // if quantized PWM duty is not less than previous value,
        // choose next smaller frequency value (next higher timer value)
        if(uTemp>=uOldPWM)
        {
            // get next lower value
            uOCR1B--;
        }
    }

    // calculate back to quantized PWM duty
    uTemp = uOCR1B;
    uTemp *= 1000;
    uTemp /= (uint32_t)(ICR1);

    // save the quantized value
    *uPWM = uTemp;
}

//----------------------------------------------------------------------------
// PWMDutyEnterFunction
//
// Gets called when encoder is pushed while PWM Duty menu item was active
//
// -> --
// <- --
//----------------------------------------------------------------------------
void PWMDutyEnterFunction()
{
    // set PWM Duty to 50 %
    Params.iPWMDuty = 500;
}

//----------------------------------------------------------------------------
// PWMDutyCountEnterFunction
//
// Gets called when encoder is pushed while PWM Duty Count menu item was active
//
// -> --
// <- --
//----------------------------------------------------------------------------
void PWMDutyCountEnterFunction()
{
    // re-trigger Pulse Burst
    
    ATOMIC_RW(g_iPWMimpulseCount, Params.iPWMimpulses);
}
#endif // COMPILE_WITH_PWM

//----------------------------------------------------------------------------
// TRMSC_RangeUpdateDisplayFunction
//
// Gets called periodically to update TRMSC range display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void TRMSC_RangeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if((LastParams.ucRange!=Params.ucRange)||ucForceUpdate)
    {
        // display new setting
        switch(Params.ucRange)
        {
            case RANGE_100mV:
                PANEL_PRINT_CHOICE("100 mV");
                break;
            case RANGE_1V:
                PANEL_PRINT_CHOICE("1 V");
                break;
            case RANGE_10V:
                PANEL_PRINT_CHOICE("10 V");
                break;
            case RANGE_100V:
                PANEL_PRINT_CHOICE("100 V");
                break;
            case RANGE_AUTO:
                PANEL_PRINT_CHOICE("auto");
                break;
        }
    }
}

//----------------------------------------------------------------------------
// TRMSC_PermanentUpdateDisplayFunction
//
// Gets called periodically to update TRMSC permanent display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void TRMSC_PermanentUpdateDisplayFunction(uint8_t ucForceUpdate,
        void *pParam __attribute__((unused)))
{
    if((LastParams.ucPermanentTRMSC!=Params.ucPermanentTRMSC)||ucForceUpdate)
    {
        // display new setting
        switch(Params.ucPermanentTRMSC)
        {
            case PERMANENT_TRMSC_OFF:
                PANEL_PRINT_CHOICE("Off");
                break;
            case PERMANENT_TRMSC_RMS:
                PANEL_PRINT_CHOICE("RMS");
                break;
            case PERMANENT_TRMSC_PEAK:
                PANEL_PRINT_CHOICE("Peak");
                break;
        }
    }
}

//----------------------------------------------------------------------------
// TRMSC_LevelUpdateDisplayFunction
//
// Gets called periodically to update TRMSC level display
//
// -> pParam: pointer to double RMS or peak
// <- --
//----------------------------------------------------------------------------
void TRMSC_LevelUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam)
{
    // TODO warum 16?????
    char s[16];
    double *d = pParam;
    double dB;

    if((g_dLastTRMSC_RMS!=g_dTRMSC_RMS)|| (g_dLastTRMSC_Peak!=g_dTRMSC_Peak)|| ucForceUpdate)
    {
        if(Status.OverVolt||(*d<0.0f))
        {
#ifdef COMPILE_WITH_DISPLAY204
            Lcd_OverWrite_P(10, 2, 8, PSTR("OVERLOAD"));
#else
            Lcd_OverWrite_P(0, 1, 8, PSTR("OVERLOAD"));
#endif
        }
        else
        {
            // output parameter to LCD
            if( Params.ucVolt_dB  &&
                    ( (g_currentMenuItem == (void*)&SubmenuTRMSC_RMS) ||
                      ((Params.ucPermanentTRMSC == PERMANENT_TRMSC_RMS) && (g_currentMenuItem != (void*)&SubmenuTRMSC_Peak)) ))
            {
                //   9.99: | +9.99dB|
                //  -9.99: | -9.99dB|
                //  22.22: | +22.2dB|
                // -22.22: | -22.2dB|
                dB = (20 * log10( ( (*d < 0.000025f) ? 0.000025f : *d) / 774.597));
                if ((dB >= 10.0f) || (dB <= -10.0f))
                {
                    // TODO check
                    sprintf_P(s, PSTR(EXTRA_EA_DIP_SIGN_TRMSC"%+2.1fdB"), dB);
                }
                else
                {
                    // TODO check
                    sprintf_P(s, PSTR(EXTRA_EA_DIP_SIGN_TRMSC"%+1.2fdB"), dB);
                }
            }
            else
            {
                if(*d<1000.0f)
                    sprintf_P(s, PSTR(EXTRA_EA_DIP_SIGN_TRMSC"%5.1fmV"), *d);
                else
                    sprintf_P(s, PSTR(EXTRA_EA_DIP_SIGN_TRMSC"%6.3fV"), *d/1000.0f);
            }
#ifdef COMPILE_WITH_DISPLAY204
            Lcd_Write(10, 2, 8, s);
#else
            Lcd_Write(0, 1, 8, s);
#endif
        }
        g_dLastTRMSC_RMS = g_dTRMSC_RMS;
        g_dLastTRMSC_Peak = g_dTRMSC_Peak;
    }
}

//----------------------------------------------------------------------------
// SweepModeUpdateDisplayFunction
//
// Gets called periodically to update sweep mode display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void SweepMenuUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if((LastParams.ucSweepMenu!=Params.ucSweepMenu)||ucForceUpdate)
    {
        // display new setting
        switch(Params.ucSweepMenu)
        {
            case SWEEP_OFF:
                PANEL_PRINT_CHOICE_WITH_ENTER("Off");
                break;
            case SWEEP_ON:
                PANEL_PRINT_CHOICE_WITH_ENTER("On");
                break;
        }
    }
}

//----------------------------------------------------------------------------
// SweepMarkerModeUpdateDisplayFunction
//
// Gets called periodically to update sweep mode display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void SweepMarkerModeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if((LastParams.ucSweepMarkerMode!=Params.ucSweepMarkerMode)||ucForceUpdate)
    {
        // display new setting
        switch(Params.ucSweepMarkerMode)
        {
            case MARKER_ABSOLUTE:
                PANEL_PRINT_CHOICE("absolute");
                break;
            case MARKER_RELATIVE:
                PANEL_PRINT_CHOICE("relative");
                break;
        }
    }
}

//----------------------------------------------------------------------------
// SweepSlopeUpdateDisplayFunction
//
// Gets called periodically to update sweep mode display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void SweepSlopeUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if((LastParams.ucSweepSlope!=Params.ucSweepSlope)||ucForceUpdate)
    {
        // display new setting
        switch(Params.ucSweepSlope)
        {
            case SWEEP_UP:
                PANEL_PRINT_CHOICE("Up");
                break;
            case SWEEP_DOWN:
                PANEL_PRINT_CHOICE("Down");
                break;
            case SWEEP_UPDOWN:
                PANEL_PRINT_CHOICE("Up+Down");
                break;
        }
    }
}

//----------------------------------------------------------------------------
// SweepModeUpdateDisplayFunction
//
// Gets called periodically to update sweep formula display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void SweepModeUpdateDisplayFunction(uint8_t ucForceUpdate,
                                    void *pParam __attribute__((unused)))
{
    if((LastParams.ucSweepMode!=Params.ucSweepMode)||ucForceUpdate)
    {
        // display new setting
        switch(Params.ucSweepMode)
        {
            case SWEEP_LINEAR:
                PANEL_PRINT_CHOICE("linear");
                break;
            case SWEEP_LOG:
                PANEL_PRINT_CHOICE("log");
                break;
        }
    }
}
//----------------------------------------------------------------------------
// SweepMarkerUpdateDisplayFunction
//
// Gets called periodically to update sweep formula display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void SweepMarkerUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if((LastParams.ucSweepMarker!=Params.ucSweepMarker)||ucForceUpdate)
    {
        // display new setting
        switch(Params.ucSweepMarker)
        {
            case MARKER_OFF:
                PANEL_PRINT_CHOICE("Off");
                break;
            case MARKER_DECADE:
                PANEL_PRINT_CHOICE("decade");
                break;
            case MARKER_OCTAVE:
                PANEL_PRINT_CHOICE("octave");
                break;
        }
    }
}

//----------------------------------------------------------------------------
// SettingsSaveUpdateDisplayFunction
//
// Gets called periodically to update TRMSC range display
//
// -> pParam: --
// <- --
//----------------------------------------------------------------------------
void SettingsSaveUpdateDisplayFunction(uint8_t ucForceUpdate, void *pParam __attribute__((unused)))
{
    if(ucForceUpdate||(Params.ucAutosave!=LastParams.ucAutosave))
    {
        // display new setting
        switch(Params.ucAutosave)
        {
            case 0:
                PANEL_PRINT_CHOICE_WITH_ENTER("now");
                break;
            case 1:
            default:
                PANEL_PRINT_CHOICE("auto 30s");
                break;
        }
    }
}

//----------------------------------------------------------------------------
// SettingsSaveEnterFunction
//
// Gets called when encoder is pushed while "Save" menu item was active
//
// -> --
// <- --
//----------------------------------------------------------------------------
static void SettingsSaveEnterFunction(void)
{
    if(0==Params.ucAutosave)
    {
        PANEL_PRINT_CENTER("Saving");
        wait_ms(1000);
        eeprom_write_block(&Params, EEPROM_PARAMS, sizeof(Params));
        PANEL_PRINT_CENTER("OK");
        g_ucMenuItemChanged = 1;
        wait_ms(3000);
        g_currentMenuItem = (void*)&MenuSettings;
    }
}


//----------------------------------------------------------------------------
// PrintLCD
//
// Output various values to the LCD.
//
// -> f = offset voltage
//    uType = LCD_TYPE_OFFSET: print offset voltage in mV, V
//          = LCD_TYPE_LEVEL_VOLT: print level voltage in mV, V
//          = LCD_TYPE_LEVEL_DB: print level voltage in dB
//          = LCD_TYPE_FREQUENCY: output frequency in Hz
//          = LCD_TYPE_TIME_MS: output time in milliseconds
// <- --
//----------------------------------------------------------------------------
static void Panel_OutputValue(double f, int16_t i, uint8_t uType)
{
    char s[10];
    double dB;

    // isolate fractional part of i without sign
    int16_t j = ((i<0)?-i:i) % 1000;

    // 2nd line
    switch(uType)
    {
        case LCD_TYPE_OFFSET:
//      if(f<-10.0f) f = -10.0f;
//      if(f>10.0f) f = 10.0f;
            //   0.000: |    0 mV|
            //   0.999: | +999 mV|
            //  -0.999: | -999 mV|
            //   0.099: |  +99 mV|
            //  -0.099: |  -99 mV|
            //   0.009: |   +9 mV|
            //  -0.009: |   -9 mV|
            //   9.999: | +9.999V|
            //  -9.999: | -9.999V|
            //  10.230: |+10.230V|
            // -10.230: |-10.230V|
            if(0==i)
                sprintf_P(s, PSTR("    0 mV"));
            else if((i<1000)&&(i>-1000))
                sprintf_P(s, PSTR("%+5d mV"), i);
            else
                sprintf_P(s, PSTR("%+3d.%03dV"), i/1000, j);
            break;

        case LCD_TYPE_LEVEL_VOLT:
//      if(f>10.0f) f = 10.0f;
//      if(f<0.000025f) f = 0.000025f;
            //  0.009125: | 9.125mV|
            //  0.099125: |99.125mV|
            //  0.199125: |199.13mV|
            //  0.200000: |   200mV|
            //  0.999000: |   999mV|
            //  9.999000: | 9.999 V|
            // 10.000000: |10.000 V|
            if(f<0.1f)
                sprintf_P(s, PSTR("%6.3fmV"), f*1000.0f);
            else if(f<=0.2f)
                sprintf_P(s, PSTR("%6.2fmV"), f*1000.0f);
            else if(f<1.0f)
                sprintf_P(s, PSTR("   %dmV"), (int)(f*1000.0f));
            else
                sprintf_P(s, PSTR("%7.3fV"), f);
            break;

        case LCD_TYPE_LEVEL_DB:
            if(f<0.000025f) f = 0.000025f;
//      if(f>10.0f) f = 10.0f;
            //   9.99: | +9.99dB|
            //  -9.99: | -9.99dB|
            //  22.22: |+22.22dB|
            // -22.22: |-22.22dB|
            dB = (20 * log10(f/0.774597));
            sprintf_P(s, PSTR("%+6.2fdB"), dB);
            break;

        case LCD_TYPE_FREQUENCY:
            if(f<0.015625f) f = 0.015625f;
            if(f>MAX_FREQUENCY) f = MAX_FREQUENCY;

            if(f<10.0f)
                sprintf_P(s, PSTR("%6.3fHz"), f);
            else if(f<100.0f)
                sprintf_P(s, PSTR("%6.2fHz"), f);
            else if(f<1000.0f)
                sprintf_P(s, PSTR("%6.1fHz"), f);
            else if(f<10000.0f)
                sprintf_P(s, PSTR("%5.3fkHz"), f/1000.0f);
            else if(f<100000.0f)
                sprintf_P(s, PSTR("%5.2fkHz"), f/1000.0f);
            else
                sprintf_P(s, PSTR("%5.1fkHz"), f/1000.0f);
            break;

        case LCD_TYPE_TIME_MS:
            if(i<0) i = 0;
            sprintf_P(s, PSTR("%5d ms"), i);
            break;

        case LCD_TYPE_TIME_100MS:
            if(i<0) i = 0;
            sprintf_P(s, PSTR("%4d.%1d s"), i/1000, j/100);
            break;

        case LCD_TYPE_DOUBLE:
            sprintf_P(s, PSTR("%3.2f    "), f);
            break;

#ifdef COMPILE_WITH_PWM
        case LCD_TYPE_PERCENT:
            sprintf_P(s, PSTR("%5.1f %% "), (double)i / 10.0f);
            break;

        case LCD_TYPE_COUNT:
        sprintf_P(s, PSTR("   %5d"), i);
        break;
#endif
    }

#ifdef COMPILE_WITH_DISPLAY204
    /*
    this works properly, since strings for
    * PANEL_PRINT_CENTER(x)
    * PANEL_PRINT_CHOICE(x)  and
    * PANEL_PRINT_CHOICE_WITH_ENTER(x)
    shorter than 8 char. otherwise check the following line
    */
    Lcd_Write(12, 2, 8, s);
#else
    Lcd_Write(0, 1, 8, s);
#endif

}


//----------------------------------------------------------------------------
// findMenuEntry
//
// -> ucIdentifier
//
// <- g_currentMenuItem
//----------------------------------------------------------------------------
menu_entry_t *findMenuEntry(uint8_t ucIdentifier)
{
    menu_entry_t *foundEntry;
    int iSafetyCntr = 0; // avoid endless loops if lists are not properly maintained or I made an error iterating through it ;-)

    if (pgm_read_byte(&g_currentMenuItem->ucIdentifier) == ucIdentifier)
    {
        return g_currentMenuItem;
    }
    else
    {
        foundEntry = (menu_entry_t *) pgm_read_word(&g_currentMenuItem->pNext);
        do
        {
            if (pgm_read_byte(&foundEntry->ucIdentifier) == ucIdentifier)
            {
                return foundEntry;
            }
            else
            {
                foundEntry = (menu_entry_t *)pgm_read_word(&foundEntry->pNext);
            }
            if (iSafetyCntr ++ > 100) break;
        }
        while (((menu_entry_t *)(pgm_read_word(&foundEntry->pNext)) != g_currentMenuItem) && ((menu_entry_t *)(pgm_read_word(&foundEntry->pNext)) != null));
    }
    return g_currentMenuItem;
}


extern const PROGMEM char g_cVersStrShort_P[]; // can't be moved to headerfile ;-(

void Panel_SplashScreen(void)
{
    char s[COLUMN_MAX+1];


#ifdef COMPILE_WITH_DISPLAY204
    // DDS in big
    Lcd_Write_P(0, 0, 12, PSTR("\x1f\x1f\x02\x20\x1f\x1f\x02\x20\x04\x1f\x1f\x20"));
    Lcd_Write_P(0, 1, 12, PSTR("\x1f\x20\x1f\x20\x1f\x20\x1f\x20\x05\x07\x20\x20"));
    Lcd_Write_P(0, 2, 12, PSTR("\x1f\x20\x1f\x20\x1f\x20\x1f\x20\x20\x06\x02\x20"));
    Lcd_Write_P(0, 3, 12, PSTR("\x1f\x1f\x03\x20\x1f\x1f\x03\x20\x1f\x1f\x03\x20"));

    // Infos in small
    sprintf_P(s, g_cVersStrShort_P);
    Lcd_Write(12, 1, 8, s);
    sprintf_P(s, PSTR("Adr  %-3d"), g_ucSlaveCh);
    Lcd_Write(12, 2, 8, s);

    Lcd_Write_P(12, 3, 3, PSTR("RC0"));
#else
    sprintf_P(s, g_cVersStrShort_P);
    Lcd_Write(0, 0, 8, s);
    sprintf_P(s, PSTR("Adr  %-3d"), g_ucSlaveCh);
    Lcd_Write(0, 1, 8, s);
#endif
}


// works properly only if jobPanel runs before jobExecute
static void Panel_UpdateStatusLine(void)
{
#ifdef COMPILE_WITH_DISPLAY204
    if (LastParams.iBurstOnOff != Params.iBurstOnOff)
    {
        if (Params.iBurstOnOff == 0)
            Lcd_Write_P(15, 0, 1, PSTR("\x20"));
        else
            Lcd_Write_P(15, 0, 1, PSTR("B"));

    }

    if (LastParams.ucSweepMenu != Params.ucSweepMenu)
    {
        if (Params.ucSweepMenu == SWEEP_OFF)
            Lcd_Write_P(17, 0, 1, PSTR("\x20"));
        else
            Lcd_Write_P(17, 0, 1, PSTR("S"));
    }

    if (LastParams.ucWaveForm != Params.ucWaveForm)

    {
        switch (Params.ucWaveForm)
        {
                // Darstellung entweder Ascii oder schicke Zeichen aus ea dip 204
            case WAVE_OFF:
                Lcd_Write_P(19, 0, 1, PSTR("O"));
                break;
            case WAVE_SINE:
                Lcd_Write_P(19, 0, 1, PSTR("\xce"));
                break;
            case WAVE_TRIANGLE:
                Lcd_Write_P(19, 0, 1, PSTR("\xc1"));
                break;
            case WAVE_SQUARE:
                Lcd_Write_P(19, 0, 1, PSTR("\xc2"));
                break;
            case WAVE_LOGIC:
                Lcd_Write_P(19, 0, 1, PSTR("L"));
                break;

#ifdef COMPILE_WITH_PWM
            case WAVE_PWM:
                Lcd_Write_P(19,0,1,PSTR("P"));
                break;
#else
            case WAVE_EXT_IN:
                Lcd_Write_P(19, 0, 1, PSTR("E"));
                break;
#endif
            default:
                Lcd_Write_P(19, 0, 1, PSTR("F"));
        }
    }
#else
#endif
}



void Panel_PrintEnterChoice(void)
{
#ifdef COMPILE_WITH_DISPLAY204
    Lcd_Write_P( 9, 3, 1, PSTR(EXTRA_EA_DIP_SIGN_ENTER));
    Lcd_Write_P(10, 3, 1, PSTR(EXTRA_EA_DIP_SIGN_ENTER));
#else
    // 2x8 hier wird das Enter ohne Überschreiben ausgeführt
    Lcd_Write_P(7, 1, 1, PSTR(EXTRA_EA_DIP_SIGN_ENTER));
#endif
}


void Panel_PrintEnter(void)
{
#ifdef COMPILE_WITH_DISPLAY204
    Lcd_Write_P( 9, 3, 1, PSTR(EXTRA_EA_DIP_SIGN_ENTER));
    Lcd_Write_P(10, 3, 1, PSTR(EXTRA_EA_DIP_SIGN_ENTER));
#else
    // 2x8 hier wird das Enter mit Überschreiben ausgeführt
    // TODO check
    Lcd_OverWrite_P(7, 1, 1, PSTR(EXTRA_EA_DIP_SIGN_ENTER));
#endif
}


static void Panel_PrintBack(void)
{
#ifdef COMPILE_WITH_DISPLAY204
    Lcd_Write_P( 9, 3, 1, PSTR(EXTRA_EA_DIP_SIGN_BACK));
    Lcd_Write_P(10, 3, 1, PSTR(EXTRA_EA_DIP_SIGN_BACK));
#else
    // 2x8 hier wird das Back (Enter) mit Überschreiben ausgeführt
    Lcd_OverWrite_P(7, 1, 1, PSTR(EXTRA_EA_DIP_SIGN_BACK));
#endif
}

//
static void Panel_ResetEnter(void)
{
#ifdef COMPILE_WITH_DISPLAY204
    Lcd_Write_P(9, 3, 2, PSTR("  "));
#else
    ;
#endif
}


static void Panel_ClearDataLine(void)
{
#ifdef COMPILE_WITH_DISPLAY204
    Lcd_Write_P(10, 2, 10, PSTR("          "));
#else
    ;
#endif
}

static void Panel_ClearMenuLine(void)
{
#ifdef COMPILE_WITH_DISPLAY204
    Lcd_Write_P(0, 2, 10, PSTR("          "));
#else
    ;
#endif
}
