/*
 *
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <inttypes.h>
#include <avr/pgmspace.h>


#define VERSSTRSHORT_STD "DDS D1.0"     // 8 chars max
#define VERSSTRLONG_STD  "1.0d[DDS C by HB, TF, JW, PSC and KSC]"

#define VERSSTRSHORT_PWM "DDSpC1.0"     // 8 chars max
#define VERSSTRLONG_PWM  "1.0d[DDSp C by HB, TF, JW and PSC]"

//#define COMPILE_WITH_PWM 1         // default only OC1B / PD4  // requires hardware change // will replace ext. Audio-in with PWM menu/function
//#define COMPILE_WITH_PWM_OC1A 1  // option additionally to COMPILE_WITH_PWM / OC1A/PD5, TODO: auto detection of PWM on PD4/PD5
//define PWM_SHOW_PULSES 1         // show pulses when manually changing the value via encoder

//#define COMPILE_WITH_DISPLAY204 1
//#define ENCODER_NAVIGATION 1

#if defined(COMPILE_WITH_DISPLAY204) && defined(ENCODER_NAVIGATION)
	#error DISPLAY204 does not work with ENCODER_NAVIGATION; untested and the encoder symbols are missing
#endif

#define STRICTSYNTAX


#endif
