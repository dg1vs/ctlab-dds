#ifndef MAIN_H_
#define MAIN_H_

#include <inttypes.h>

//----------------------------------------------------------------------------
// some useful shortcuts
//----------------------------------------------------------------------------
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define LO(x)   ((x)&0xFF)
#define HI(x)   (((x)>>8)&0xFF)
#define nop()   asm volatile("nop"::)
#define sleep() asm volatile("sleep"::)

//----------------------------------------------------------------------------
// prototypes
//----------------------------------------------------------------------------
void initParams(void);
void JobLoop(void);
void jobExecute(void);
void CheckLimits(void);
void CalculateSweepParameters(void);
void CrossCalcSweep(uint8_t mode);
void CrossCalcLevel(uint8_t mode);

#endif /*MAIN_H_*/


