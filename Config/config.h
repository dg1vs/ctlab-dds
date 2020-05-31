/*
 *
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

// ******************************************************************************************************************
// *** user configurable parameters *********************************************************************************
// ******************************************************************************************************************
// The following defines decide which default values are loaded (12/16bit DAC/ADC hardware version)

//#define DEFAULT_PARAMS_12BIT
//#define DEFAULT_PARAMS_16BIT

// DCP module presence selection is actually not required since relay switching is done anyway.
// BTW: OPT 167 Bit2 is ignored.

// Extension board with two 16-Bit DACs to get rid off the sample & hold circuit
//#define DUAL_DAC        // disable this #define if you have just a standard DCG with single DAC

// for mandatory addressing on bus commands for improving bus reliability
//#define STRICTSYNTAX    // disable this #define if you want the command parser behavior of original pascal firmware

// ******************************************************************************************************************
// *** internal debugging settings **********************************************************************************
// ******************************************************************************************************************

//#define DEBUGSTDHW    // in case of debugging standard hardware on DUAL-DAC hardware
//#define PRELOAD_ARB_RAM   // for testing arbitrary function

// ******************************************************************************************************************
// *** version strings **********************************************************************************************
// ******************************************************************************************************************

//#define VERSSTRLONG_DUAL    "1.2 [DCG2d by HB + PSC]"
//#define VERSSTRLONG_SINGLE  "1.2 [DCG2 by HB + PSC]"

//#define VERSSTRSHORT_DUAL   "DCG2d1.2"
//#define VERSSTRSHORT_SINGLE "DCG2 1.2"

// **********************************************************************************************

// checking user settings


#endif
