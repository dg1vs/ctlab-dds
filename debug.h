/*
 * Copyright (c) 2006, 2007 by Hartmut Birr
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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <inttypes.h>
#include <avr/pgmspace.h>

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


#define DbgPrint(...)   printf_P(__VA_ARGS__)


#ifdef DEBUG

    extern uint8_t g_ucSlaveCh;

#ifndef NDEBUG
#define DPRINT(...)     do { DbgPrint(PSTR("#%d:-1=(%s:%d) "), SlaveCh, __FILE__, __LINE__); DbgPrint(__VA_ARGS__); } while(0)
#define CHECKPOINT      do { DbgPrint(PSTR("#%d:-1=(%s:%d)\n"), SlaveCh, __FILE__, __LINE__); } while(0)
#else
#define DPRINT(...)
#define CHECKPOINT
#endif

#define DPRINT1(...)    do { DbgPrint(PSTR("#%d:-1=(%s:%d) "), SlaveCh, __FILE__, __LINE__); DbgPrint(__VA_ARGS__); } while(0)
#define CHECKPOINT1     do { DbgPrint(PSTR("#%d:-1(%s:%d)\n"), SlaveCh, __FILE__, __LINE__); } while(0)

#else

#define DPRINT(...)
#define CHECKPOINT

#define DPRINT1(...)
#define CHECKPOINT1
#endif

#ifdef __cplusplus
}
#endif

#endif
