/*
 * Config_Char.h
 *
 * Created: 05/01/2022 10:02:59
 *  Author: ks
 */ 


#ifndef __CONFIG_LCD_CHAR_H__
#define __CONFIG_LCD_CHAR_H__

#include <avr/pgmspace.h>
#include "Config.h"

//#define COMPILE_WITH_DISPLAY204 1 --< via Compiler Define

// The extra char space in the LCD is 8 char long, the DCG is using the complete space
// DDS has different sizes, nevertheless the hw is the same
#ifdef COMPILE_WITH_DISPLAY204              /* PM 20*4 */
	#define CURSOR_ARRAY_SIZE 8*8
#else                                       /* PM8 */
	#define CURSOR_ARRAY_SIZE 6*8
#endif

extern const PROGMEM uint8_t LCD_ExtraCharacter[CURSOR_ARRAY_SIZE];


#endif /* CONFIG_LCD_CHAR_H_ */