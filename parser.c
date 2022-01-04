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

#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "parser.h"
#include "Uart.h"
#include "dds.h"

#define NDEBUG
#include "debug.h"

const PROGMEM char ErrStr[11][9] =
{
    "[OK]",
    "[SRQUSR]",
    "[BUSY]",
    "[OVERLD]",
    "[CMDERR]",
    "[PARERR]",
    "[LOCKED]",
    "[CHKSUM]",
    "[FUSE]",
    "[FAULT]",
    "[OVRFLW]"
};


char g_cSerInpStr[64];
uint8_t SerInpCount;

uint8_t CurrentCh = 255;
struct
{
    uint8_t Verbose : 1;
} ParserFlags;


//---------------------------------------------------------------------------------------------

void SerPrompt(ERROR Err, uint8_t Status)
{
    if (ParserFlags.Verbose || Err != NoErr)
    {
        printf_P(PSTR("#%d:255=%d %S\n"), g_ucSlaveCh, Err + Status, ErrStr[Err]);  // capital "%S" to print string from PROGMEM
    }
}


//---------------------------------------------------------------------------------------------
void SerStr(char* str)
{
    printf_P(PSTR("%s\n"), str);
}


//---------------------------------------------------------------------------------------------
void jobParseData(void)
{
    uint8_t count;
    static char c = 0;
    ERROR Error = NoErr;
    static ERROR OvflError = NoErr;
    uint8_t SubCh;
    char prev;

    count = Uart_GetRxCount();

    // maximum in chunks of 20 Bytes, in order not to stay too long here
    if (count > 20)
    {
        count = 20;
    }

    // if an OvrflError has occurred, read in following buffer content until \n or \r to clear overrun buffer garbage
    if (OvflError == OvrflErr)
    {
        while (count--)
        {
            Uart_GetRxData((uint8_t*)&c, 1);
            if  ((c == '\r') || (c == '\n'))
            {
                OvflError = NoErr;
                break;
            }
        }
        return;
    }

    while (count--)
    {
        prev = c;
        Uart_GetRxData((uint8_t*)&c, 1);

        if (c >= ' ' && (uint8_t)c <= 127)
        {
            g_cSerInpStr[SerInpCount++] = c;

            if (SerInpCount >= sizeof(g_cSerInpStr))
            {
                // buffer overrun, last character in buffer is not \n, \r, \b
                SerInpCount = 0;
                CHECKPOINT;
                OvflError = OvrflErr;
                SerPrompt(OvflError, 0);
                return;
            }
        }
        else if (c == '\b')
        {
            if (SerInpCount)
            {
                SerInpCount--;
            }
        }

        else if (c == '\n' && prev == '\r')
        {
            continue;
        }

        else if (c == '\r' || c == '\n')
        {
            // after \r or \n, we consider a new command complete
            char* s;
            char* pos;
            uint16_t value;
            float Param;

            // Zero-Terminate String in Buffer
            g_cSerInpStr[SerInpCount] = 0;

            count = 0; // stop while loop
            SerInpCount = 0;
            s = g_cSerInpStr;
            while (*s == ' ') s++;
            // now check for various options
            if (*s == 0)
            {
                // Leerer String, just skip, don't forward
                SerPrompt(NoErr, 0);
                break;
            }


#ifdef STRICTSYNTAX
            if (*s == '#')
            {
                s++;
                while (*s == ' ') s++;

                if ( (*s >= '0') && (*s <= '9') )
                {
                    s++;
                    while (*s == ' ') s++;
                    if (*s == ':')
                    {
                        // Syntax "#x:result /r/n" is OK
                        SerStr(g_cSerInpStr);  // forward the result (as it is)
                        Error = NoErr;      // don't report an error for a shortened result
                        continue;           // go on and handle next characters

                    }
                }

                Error = SyntaxErr;
                break;
            }
#else
            if (*s == '#')
            {
                SerStr(g_cSerInpStr);  // forward the result
                Error = NoErr;      // don't report an error for a shortened result
                continue;           // go on and handle next characters
            }

#endif // STRICTSYNTAX

            if (Error != NoErr)
            {
                CHECKPOINT;
                break;
            }
            pos = strchr(s, ':');
            if (pos)
            {
                // address is checked here, if present
                if (*s == '*')
                {
                    // Omni-Befehl, Weiterleiten (and process locally later, too)
                    SerStr(g_cSerInpStr);
                    s++;
                    while (*s == ' ') s++;
                    if (s != pos)
                    {
                        CHECKPOINT;
                        Error = SyntaxErr;
                        break;
                    }
                }
                else
                {
                    value = 0;
                    if (*s >= '0' && *s <= '9')     // actually, address can only be between 0...7
                    {
                        value += *s - '0';
                        s++;
                    }
                    else
                    {
                        CHECKPOINT;
                        Error = SyntaxErr;
                        break;
                    }
                    // check, if only spaces are between single digit address and colon ':'
                    while (*s == ' ') s++;
                    if (s != pos)
                    {
                        CHECKPOINT;
                        Error = SyntaxErr;
                        break;
                    }
                    CurrentCh = value;
                    if (CurrentCh != g_ucSlaveCh)
                    {
                        // Nicht für uns, weiterleiten
                        SerStr(g_cSerInpStr);
                        continue; // go on and handle next characters
                    }
                }
                s = pos + 1;
                while (*s == ' ') s++;
                if (*s == 0)
                {
                    CHECKPOINT;
                    Error = SyntaxErr;
                    break;
                }
            } // if (pos) ':'

#ifdef STRICTSYNTAX
            else // no (pos) ':'
            {
                // no Addressdesignator found --> incorrect syntax
                Error = SyntaxErr;
                break;
            }
#endif
            // Ist für uns, ab hier eigentliche Verarbeitung
            ParserFlags.Verbose = strchr(s, '!') != NULL || strchr(s, '?') != NULL;

            pos = strchr(s, '$');
            if (pos && isxdigit(pos[1]))
            {
                uint8_t Sum = 0;
                uint8_t calcSum = 0;
                uint8_t i;
                char* ss = g_cSerInpStr;

                // calculate the checksum from command data
                while (ss < pos)
                {
                    calcSum ^= *ss++;
                }

                i = isxdigit(pos[2]) ? 2 : 1;

                // retrieve checksum from command after '$'
                while (i)
                {
                    i--;
                    pos++;
                    Sum *= 16;
                    if (isdigit(*pos))
                    {
                        Sum += *pos - '0';
                    }
                    else
                    {
                        Sum += toupper(*pos) - 'A' + 10;
                    }
                }

                // check if calculated and command checksum are the same
                if (Sum != calcSum)
                {
                    Error = ChecksumErr;
                    printf_P(PSTR("#%d:Command was \"%s\" CS:%02x\n"), g_ucSlaveCh, g_cSerInpStr, calcSum);
                    g_ucErrCount++;
                    break;
                }
            }

            if (*s >= '0' && *s <= '9')
            {
                // Direkter SubCh-Aufruf
                value = 0;
                while (*s >= '0' && *s <= '9' && value <= 255)
                {
                    value *= 10;
                    value += *s - '0';
                    s++;
                }
                if (value > 255)
                {
                    Error = SyntaxErr;
                    break;
                }
                SubCh = value;
            }
            else
            {
                // Klartext übersetzen to SubCh-Code
                char cmd[4];
                uint8_t i;
                uint8_t str ;
                uint8_t offset = 0;


                cmd[3] = 0;
                for (i = 0; i < 3; i++, s++)
                {
                    if (*s >= 'A' && *s <= 'Z')
                    {
                        cmd[i] = *s + 'a' - 'A';
                    }
                    else if (*s >= 'a' && *s <= 'z')
                    {
                        cmd[i] = *s;
                    }
                    else
                    {
                        break;
                    }
                }
                if (i < 3)
                {
                    CHECKPOINT;
                    Error = SyntaxErr;
                    break;
                }
                value = 0;
                while (*s == ' ') s++;
                if (*s >= '0' && *s <= '9')
                {
                    while (*s >= '0' && *s <= '9' && value <= 255)
                    {
                        value *= 10;
                        value += *s - '0';
                        s++;
                    }
                    if (value > 255)
                    {
                        CHECKPOINT;
                        Error = SyntaxErr;
                        break;
                    }

                }

                i=0;
                while(1)
                {
                    str = pgm_read_byte(&Mnemonics[i][MNEM_STR]);

                    if (str == 0)
                        break;


                    if ( 0 == memcmp_P( cmd, &Mnemonics[i][MNEM_STR], 3) )
                    {
                        offset = pgm_read_byte(&Mnemonics[i][MNEM_VAL]);
                        break;
                    }
                    i++;
                }

                if ( str == 0 )
                {
                    CHECKPOINT;
                    Error = SyntaxErr;
                    break;
                }
                if (i == 0)
                {
                    // nop
                    CHECKPOINT;
                    SerPrompt(NoErr, 0);
                    break;
                }

                if (value + offset > 255)
                {
                    CHECKPOINT;
                    Error = SyntaxErr;
                    break;
                }

                SubCh = value + offset;
            }
            while (*s == ' ') s++;
            if (*s == '=')
            {
                s++;
                while (*s == ' ') s++;
                if (!(*s >= '0' && *s <= '9') && *s != '-')
                {
                    CHECKPOINT;
                    Error = ParamErr;
                    break;
                }
                Param = atof(s);
                ParseSetParam(SubCh, Param);
            }
            else
            {
                ParseGetParam(SubCh);
            }
            SetActivityTimer(50);
        }
    }
    if (Error != NoErr)
    {
        SerPrompt(Error, 0);
    }
}

