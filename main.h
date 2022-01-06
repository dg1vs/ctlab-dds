#ifndef MAIN_H_
#define MAIN_H_

#include <inttypes.h>
#include "helper_macros.h"


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


