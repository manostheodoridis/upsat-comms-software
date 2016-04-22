//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "mxconstants.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_i2c.h"
// ----------------------------------------------------------------------------
//
// Standalone STM32F4 empty sample (trace via ITM).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

/* Buffer used for transmission */
uint8_t aTxBuffer[33];
/* Buffer used for reception */
uint8_t aRxBuffer[33];

//void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);

static uint8_t SPI2ReadExtended(uint8_t srExtndAdrr, uint8_t srData);
static uint8_t SPI1ReadExtended(uint8_t srExtndAdrr, uint8_t srData);


I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int
main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.
	/*
	 * Warning the above
	 *  HAL_Init();
		SystemClock_Config();
		are called in _initialize_hardware.c
*/
	initialize_hardware();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_UART5_Init();
	MX_USART3_UART_Init();

	SPI1ReadExtended(0X8F, 0X00);
	SPI2ReadExtended(0X8F, 0X00);
  // Infinite loop
  while (1)
    {
       // Add your code here.
    }
}

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();




  /*Configure GPIO pins : PA_CNTRL_Pin Tx_CS_N_Pin  */
  GPIO_InitStruct.Pin = PA_CNTRL_Pin|Tx_CS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO POWER AMPLIFIER PIN LOW=OFF  	*/
  HAL_GPIO_WritePin(GPIOA, PA_CNTRL_Pin, GPIO_PIN_RESET);
  /*Configure Tx_CS_N_Pin Output Level HIGH  		*/
  HAL_GPIO_WritePin(GPIOA, Tx_CS_N_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin : 2CS_N_Pin */
  GPIO_InitStruct.Pin = Rx_CS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Rx_CS_N_GPIO_Port, &GPIO_InitStruct);
  /*Configure Rx_CS_N_Pin  Output Level HIGH 		*/
  HAL_GPIO_WritePin(Rx_CS_N_GPIO_Port , Rx_CS_N_Pin , GPIO_PIN_SET);
}


/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  HAL_I2C_Init(&hi2c1);

}



/*HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
   Check the SPI handle allocation
  if(hspi == NULL)
  {
    return HAL_ERROR;
  }

   Check the parameters
  assert_param(IS_SPI_ALL_INSTANCE(hspi->Instance));
  assert_param(IS_SPI_MODE(hspi->Init.Mode));
  assert_param(IS_SPI_DIRECTION(hspi->Init.Direction));
  assert_param(IS_SPI_DATASIZE(hspi->Init.DataSize));
  assert_param(IS_SPI_NSS(hspi->Init.NSS));
  assert_param(IS_SPI_BAUDRATE_PRESCALER(hspi->Init.BaudRatePrescaler));
  assert_param(IS_SPI_FIRST_BIT(hspi->Init.FirstBit));
  assert_param(IS_SPI_TIMODE(hspi->Init.TIMode));
  if(hspi->Init.TIMode == SPI_TIMODE_DISABLE)
  {
    assert_param(IS_SPI_CPOL(hspi->Init.CLKPolarity));
    assert_param(IS_SPI_CPHA(hspi->Init.CLKPhase));
  }
#ifdef USE_SPI_CRC
  assert_param(IS_SPI_CRC_CALCULATION(hspi->Init.CRCCalculation));
  if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    assert_param(IS_SPI_CRC_POLYNOMIAL(hspi->Init.CRCPolynomial));
  }
#else
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
#endif

  if(hspi->State == HAL_SPI_STATE_RESET)
  {
     Allocate lock resource and initialize it
    hspi->Lock = HAL_UNLOCKED;

     Init the low level hardware : GPIO, CLOCK, NVIC...
    HAL_SPI_MspInit(hspi);
  }

  hspi->State = HAL_SPI_STATE_BUSY;

   Disable the selected SPI peripheral
  __HAL_SPI_DISABLE(hspi);

  ----------------------- SPIx CR1 & CR2 Configuration ---------------------
   Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
  Communication speed, First bit and CRC calculation state
  WRITE_REG(hspi->Instance->CR1, (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize |
                                  hspi->Init.CLKPolarity | hspi->Init.CLKPhase | (hspi->Init.NSS & SPI_CR1_SSM) |
                                  hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit  | hspi->Init.CRCCalculation) );

   Configure : NSS management
  WRITE_REG(hspi->Instance->CR2, (((hspi->Init.NSS >> 16U) & SPI_CR2_SSOE) | hspi->Init.TIMode));

#ifdef USE_SPI_CRC
  ---------------------------- SPIx CRCPOLY Configuration ------------------
   Configure : CRC Polynomial
  if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    WRITE_REG(hspi->Instance->CRCPR, hspi->Init.CRCPolynomial);
  }
#endif

#if defined(SPI_I2SCFGR_I2SMOD)
   Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset)
  CLEAR_BIT(hspi->Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif

  hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  hspi->State     = HAL_SPI_STATE_READY;

  return HAL_OK;
}*/
/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* UART5 init function */
void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart5);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

static uint8_t SPI2ReadExtended(uint8_t srExtndAdrr, uint8_t srData)

{

	//uint8_t aTxBuffer[3];
	//uint8_t aRxBuffer[4];

aTxBuffer[0]=0xAF;
aTxBuffer[1]=srExtndAdrr;    		//		extended address
aTxBuffer[2]=srData;         //      send dummy so that i can read data


aRxBuffer[0]=0x00;
aRxBuffer[1]=0x00;
aRxBuffer[2]=0x00;
aRxBuffer[3]=0x00;      //      one more for slot for contingency





HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
//delay10ms (1);
HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 3, 5000); //send and receive 3 bytes
//delay10ms (1);
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);

return aRxBuffer[2];  //if need be please change this part to return the whole buffer

}



static uint8_t SPI1ReadExtended(uint8_t srExtndAdrr, uint8_t srData)

{

	//uint8_t aTxBuffer[3];
	//uint8_t aRxBuffer[4];

aTxBuffer[0]=0xAF;
aTxBuffer[1]=srExtndAdrr;    		//		extended address
aTxBuffer[2]=srData;         //      send dummy so that i can read data


aRxBuffer[0]=0x00;
aRxBuffer[1]=0x00;
aRxBuffer[2]=0x00;
aRxBuffer[3]=0x00;      //      one more for slot for contingency





HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
//delay10ms (1);
HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 3, 5000); //send and receive 3 bytes
//delay10ms (1);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);


return aRxBuffer[2];  //if need be please change this part to return the whole buffer

}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
