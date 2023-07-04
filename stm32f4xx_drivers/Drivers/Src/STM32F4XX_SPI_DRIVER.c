/*
 * STM32F4XX_SPI_DRIVER.c
 *
 *  Created on: 04-Jul-2023
 *      Author: Rohan
 */

#include "STM32F407XX_SPI_DRIVER.h"

/*********************************************************************/
/*** API for Peripheral Clock Setup ***/
/*********************************************************************
 * @Function Name			- SPI_PeriphClockControl
 *
 * @Description             - This function enables or disables peripheral clock for SPI
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base address of the SPI peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_PeriphClockControl (SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of SPI
	if (EnORDi ==  ENABLE){
		if (pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if (pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}else{
		if (pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3){
			SPI3_PCLK_DI();
		}else if (pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
}


/*********************************************************************/
/*** API for Initialize and De-Initialize ***/
/*********************************************************************
 * @Function Name					- SPI_Init
 *
 * @Description             		- This function initializes SPI
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pSPIHandle]	- Handling of SPI Port
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_Init (SPI_Handle_t *pSPIHandle ){
	; 			//API in order to initialize SPI
}

/*********************************************************************
 * @Function Name			- SPI_DeInit
 *
 * @Description             - This function de-initializes SPI
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base address of the SPI peripheral
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_DeInit (SPI_RegDef_t *pSPIx){
	//API in order to de-initialize SPI
	if (pSPIx == SPI1){
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	}else if (pSPIx == SPI4){
		SPI4_REG_RESET();
	}
}


/*********************************************************************/
/*** API for Sending and Receiving Data ***/
/*********************************************************************
 * @Function Name			- SPI_SendData
 *
 * @Description             - This function sends data from SPI
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base address of the SPI peripheral
 *  [uint8_t *pTxBuffer]    - Pointer pointing to transmission buffer
 *  [uint32_t length]    	- Size of data transfer
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length ){
	; 				//API in order to send data from SPI
}

/*********************************************************************
 * @Function Name			- SPI_ReceiveData
 *
 * @Description             - This function receives data from SPI
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base address of the SPI peripheral
 *  [uint8_t *pRxBuffer]    - Pointer pointing to reception buffer
 *  [uint32_t length]    	- Size of data transfer
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length ){
	; 			//API in order to receive data from SPI
}


/*********************************************************************/
/*** API for IRQ Configuration and ISR Handling ***/
/*********************************************************************
 * @Function Name			- SPI_IRQInterruptConfig
 *
 * @Description             - This function provides configuration of interrupts for SPI
 *
 * @Input Parameter(s)
 *  [uint8_t IRQNumber]		- Interrupt request number
 *  [uint8_t IRQPriority]   - Priority of interrupt request
 *  [uint8_t EnORDi]		- Enable or Disable Macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnORDi){
	; 		//API in order to configure interrupts for SPI
}

/*********************************************************************
 * @Function Name			- SPI_IRQProrityConfig
 *
 * @Description             - This function provides configuration of priority of interrupts for SPI
 *
 * @Input Parameter(s)
 *  [uint8_t IRQNumber]		- Interrupt request number
 *  [uint8_t IRQPriority]   - Priority of interrupt request
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_IRQPrConfig (uint8_t IRQNumber, uint8_t IRQPriority){
	; 								//API in order to configure priority interrupts of SPI
}

/*********************************************************************
 * @Function Name			- SPI_IRQHandling
 *
 * @Description             - This function handles the incoming interrupts for SPI
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pHandle]		- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_IRQHandling (SPI_Handle_t *pHandle){
	; 												//API in order to process the incoming interrupts of SPI
}
