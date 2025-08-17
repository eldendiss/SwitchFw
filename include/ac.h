#include "Arduino.h"

/**
 * @brief Initializes AC
 * 
 * @return int non-zero on error
 */
int initAc();

uint8_t getOcFault();

void armComparatorAfter_us(uint64_t delay_us, uint64_t usNow);
void acService(uint64_t usNow);

