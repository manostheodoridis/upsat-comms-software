#ifdef __CC1120X_SPI_H
#define __CC1120X_SPI_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void halRfWriteReg(uint16_t add, uint8_t val); 

#endif
