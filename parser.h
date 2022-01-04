/*
 * Copyright (c) 2007 by Hartmut Birr
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

#ifndef __PARSE_H__
#define __PARSE_H__

#include <inttypes.h>
#include <avr/pgmspace.h>

//typedef struct
//{
//    const char* Mnemonic;
//    uint8_t dOffset;
//} MNEMONIC;

typedef enum
{
    NoErr = 0,
    UserReq,
    BusyErr,
    OvlErr,
    SyntaxErr,
    ParamErr,
    LockedErr,
    ChecksumErr,
    FuseErr,
    FaultErr,
    OvrflErr
} ERROR;

#define MNEM_STR 0
#define MNEM_VAL 3
extern const PROGMEM char Mnemonics [][4];

extern uint8_t g_ucSlaveCh;

void SerPrompt(ERROR, uint8_t);
void SerStr(char*);
void jobParseData(void);

void ParseGetParam(uint8_t);
void ParseSetParam(uint8_t, float);
void SetActivityTimer(uint8_t);

#endif
