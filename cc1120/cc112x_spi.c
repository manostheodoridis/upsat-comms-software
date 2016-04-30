#include "cc112x_spi.h"
#include <stdint.h>

#define CC_EXT_ADD 0x2F00
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

void halRfWriteReg(uint16_t add, uint8_t val) {

    uint8_t aTxBuffer[4];
    uint8_t aRxBuffer[4] = {0, 0, 0, 0};
    uint8_t len = 0;

	if(add >= CC_EXT_ADD) {

	    len = 3;

	    aTxBuffer[0] = 0x2F;
	    aTxBuffer[1] = (uint8_t)(0x00FF & add);		//		extended address
	    aTxBuffer[2] = val;         //      send dummy so that i can read data

	} else {

	    len = 2;

	    aTxBuffer[0] = (uint8_t)(0x00FF & add);    		//		extended address
	    aTxBuffer[1] = val;         //      send dummy so that i can read data

	}

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET); 	//chip select LOw
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, len, 5000); //send and receive 3 bytes
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

        aTxBuffer[2] = val; 
//return aRxBuffer[2];  //if need be please change this part to return the whole buffer
}
