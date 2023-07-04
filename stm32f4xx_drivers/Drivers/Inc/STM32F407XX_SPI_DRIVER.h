/*
 * STM32F407XX_SPI_DRIVER.h
 *
 *  Created on: 03-Jul-2023
 *      Author: Rohan
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "STM32F407XX.h"

/*
 * This is a configuration structure for a SPIx Peripheral
 */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SRClckConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * This is a handle structure for SPIx peripheral
 */
typedef struct{
	SPI_RegDef_t *pSPIx;		/*Holds the base address of the GPIO port to which the pin belongs to */
    SPI_Config_t SPI_Config;	/*This holds SPIconfiguration setting */
}SPI_Handle_t;



/*******************************************************************/
/************		API's SUPPORTED BY THIS DRIVER		************/
/*******************************************************************/

/*** API for Peripheral Clock Setup ***/
void SPI_PeriphClockControl (SPI_RegDef_t *pSPIx, uint8_t EnORDi);	//API in order to enable to disable peripheral clock of SPI

/*** API for Initialize and De-Initialize ***/
void SPI_Init (SPI_Handle_t *pSPIHandle ); 			//API in order to initialize SPI
void SPI_DeInit (SPI_RegDef_t *pSPIx); 				//API in order to de-initialize SPI

/*** API for Data Send and Receive ***/
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length ); 				//API in order to send data from SPI
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length ); 			//API in order to receive data from SPI

/*** API for IRQ Configuration and ISR Handling ***/
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnORDi); 		//API in order to configure interrupts of SPI
void SPI_IRQPrConfig (uint8_t IRQNumber, uint8_t IRQPriority); 								//API in order to configure priority interrupts of SPI
void SPI_IRQHandling (SPI_Handle_t *pHandle); 												//API in order to process the incoming interrupts of SPI






#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
