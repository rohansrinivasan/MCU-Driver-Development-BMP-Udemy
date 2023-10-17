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
	uint8_t SPI_SClckConfig;
	uint8_t SPI_DFF;			// data frame format
	uint8_t SPI_CPOL;			// clock polarity bits
	uint8_t SPI_CPHA;			// clock phase bits
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * This is a handle structure for SPIx peripheral
 */
typedef struct{
	SPI_RegDef_t 	*pSPIx;		/*Holds the base address of the GPIO port to which the pin belongs to */
    SPI_Config_t 	SPI_Config;	/*This holds SPIconfiguration setting */
    uint8_t 		*pTxBuffer;
    uint8_t 		*pRxBuffer;
    uint32_t 		TxLength;
    uint32_t 		RxLength;
    uint8_t 		TxState;
    uint8_t 		RxState;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI__DEVICE_MODE_SLAVE	0		//peripheral in slave mode by default after reset

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1	// Full Duplex
#define SPI_BUS_CONFIG_HD					2	// Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_TX_ONLY		3	// Simplex Mode Transmission Only
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY		4	// Simplex Mode Reception Only

/*
 * @SPI_SRClckSpeedkn
 */
#define SPI_SCLCK_SPEED_DIV2	0
#define SPI_SCLCK_SPEED_DIV4	1
#define SPI_SCLCK_SPEED_DIV8	2
#define SPI_SCLCK_SPEED_DIV16	3
#define SPI_SCLCK_SPEED_DIV32	4
#define SPI_SCLCK_SPEED_DIV64	5
#define SPI_SCLCK_SPEED_DIV128	6
#define SPI_SCLCK_SPEED_DIV256	7

/*
 * @SPI_SRClckSpeedkn
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1		// first clock transmission is first data capture edge
#define SPI_CPHA_LOW	0		// second clock transmission is first data capture edge

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0		//software slave management is activated by default


/*
 * @SPI related status flags definitions
 */
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG		(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG		(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)

/*
 * @SPI Application States
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Possible @SPI Application Event
 */
#define SPI_EVENT_TX_COMPLETE	1
#define SPI_EVENT_RX_COMPLETE	2
#define SPI_EVENT_OVR_ERR		3

/*******************************************************************/
/************		API's SUPPORTED BY THIS DRIVER		************/
/*******************************************************************/

/*** API for Peripheral Clock Setup ***/
void SPI_PeriphClockControl (SPI_RegDef_t *pSPIx, uint8_t EnORDi);	//API in order to enable to disable peripheral clock of SPI

/*** API for Initialize and De-Initialize ***/
void SPI_Init (SPI_Handle_t *pSPIHandle ); 			//API in order to initialize SPI
void SPI_DeInit (SPI_RegDef_t *pSPIx); 				//API in order to de-initialize SPI

/*** API for getting flag status ***/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*** API for Data Send and Receive ***/
/* Without Interrupts*/
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length ); 				//API in order to send data from SPI
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length ); 			//API in order to receive data from SPI

/*With Interrupts*/
/*** API for Data Send and Receive ***/
uint8_t SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length ); 				//API in order to send data from SPI
uint8_t SPI_ReceiveDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length ); 			//API in order to receive data from SPI

/*** API for IRQ Configuration and ISR Handling ***/
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t  IRQPriority, uint8_t EnORDi); 		//API in order to configure interrupts of SPI
void SPI_IRQPrConfig (uint8_t IRQNumber, uint8_t IRQPriority); 								//API in order to configure priority interrupts of SPI
void SPI_IRQHandling (SPI_Handle_t *pHandle); 												//API in order to process the incoming interrupts of SPI

/*** API for clearing OVR Flag ***/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);		//API to clear overrun flag

/*** API to close Tx and Rx ***/
void SPI_CloseTx (SPI_Handle_t *pHandle);		//API to close transmission
void SPI_CloseRx (SPI_Handle_t *pHandle);		//API to close reception

/*** API to enable SPI peripheral ***/
void SPI_PeriphControl(SPI_RegDef_t *pSPIx, uint8_t EnORDi);		//  API to enable or diable SPI peripheral

/*** API to configure SSI internally ***/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi);		//  API to configure SSI internally

/*** API to configure SSOE internally ***/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi);		//  API to configure SSOE internally

/*** API for Application Callback ***/
void SPIApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
