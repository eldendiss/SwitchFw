#include "Arduino.h"

#define TOP_FOR_STEP(percent_step)  ((uint16_t)((10000UL/(percent_step)) - 1)) // percent_step in basis points

#define PWM_MIN_TICKS    1   // enforce ~0.31% min on-time; adjust as needed
#define PWM_MAX_PERMIL   20  // cap to 2.0%

/**
 * @brief Initializes PWM
 * 
 * @return int nonzero on error
 */
int initPwm();

void setDuty(float dutyFraction);

void disablePwmOutput(void);
void enablePwmOutput(void);