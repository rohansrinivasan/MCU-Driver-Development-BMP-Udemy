/*
 * STM32F4XX_SPI_DRIVER.c
 *
 *  Created on: 04-Jul-2023
 *      Author: Rohan
 */

#include "STM32F407XX_SPI_DRIVER.h"

//IRQ Handling Function Definitions
static void SPI_TXE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle){
	if ((pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))){	// to check DFF bit in CR1
		// 16 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLength--;
		pSPIHandle->TxLength--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLength--;
		pSPIHandle->pTxBuffer++;
	}
	//To close communication is TxLength is zero
	//and inform application that Tx is over
	if (!pSPIHandle->TxLength){
		SPI_CloseTx(pSPIHandle);
		SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);		//To inform the application for callback  of Tx
	}
}

static void SPI_RXE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle){
	if ((pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))){	// to check DFF bit in CR1
		// 16 bit DFF
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength--;
		pSPIHandle->RxLength--;
		(uint16_t*)pSPIHandle->pRxBuffer--;
		(uint16_t*)pSPIHandle->pRxBuffer--;
	}else{
		// 8 bit DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength--;
		pSPIHandle->pRxBuffer--;
	}
	//To close RXNE interrupt and depict reception is complete c
	if (!pSPIHandle->RxLength){
		SPI_CloseRx(pSPIHandle);
		SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);		//To inform the application for callback  of Rx
	}
}

static void SPI_OVR_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle){
	//TO clear OVR flag-
	//1st Give Read access to SPI_DR register
	//2nd Give Read Access to SPI_SR Register
	uint8_t temp;
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX){
		pSPIHandle->pSPIx->DR;		//1st
		pSPIHandle->pSPIx->SR;		//2nd
	}
	(void)temp;

	//Informing application to close OVR interrupt
	SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

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
	//Enabling peripheral clock control for SPI
	SPI_PeriphClockControl(pSPIHandle->pSPIx, ENABLE);
	//API in order to initialize SPI
	uint32_t tempreg = 0;

	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR; // configuring device mode
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){		//configuring bus configs
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);		// BIDI Data Mode is SET
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		tempreg |= (1 << SPI_CR1_BIDIMODE);		// BIDI Data Mode is RESET
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);		// BIDI Data Mode is SET
		tempreg |= (1 << SPI_CR1_RXONLY);		//RX Only bit is SET
	}
	//configuring SPI Serial Clock Speed
	tempreg |= pSPIHandle->SPI_Config.SPI_SClckConfig << SPI_CR1_BR;

	//Configuring DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	//Configuring CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//Configure CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	//Configure SSM
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
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
 * @Note              		- Contains sub function @SPI_GetFlagStatus to set or reset status register for Tx
 */


void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length ){
	//API in order to send data from SPI
	while (length > 0){
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );	// waiting until TXE is set

		if ((pSPIx->CR1 & ( 1 << SPI_CR1_DFF))){	// to check DFF bit in CR1
			// 16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}else{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}
	}
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
	//API in order to receive data from SPI
	while (length > 0){
		while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );	// waiting until RXE is set

		if ((pSPIx->CR1 & ( 1 << SPI_CR1_DFF))){	// to check DFF bit in CR1
			// 16 bit DFF
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length--;
			length--;
			(uint16_t*)pRxBuffer++;
		}else{
			// 8 bit DFF
			pSPIx->DR = *pRxBuffer;
			length--;
			pRxBuffer++;
		}
	}

}

/*********************************************************************/
/*** API for Sending and Receiving Data with Interrupts ***/
/*********************************************************************
 * @Function Name				- SPI_SendDataIT
 *
 * @Description             	- This function sends data from SPI with interrupt handling
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pSPIHandle]	- Handling of SPI port
 *  [uint8_t *pTxBuffer]    	- Pointer pointing to transmission buffer
 *  [uint32_t length]    		- Size of data transfer
 *
 * @Return Type            		- Integer
 *
 * @Note              			- Contains sub function @SPI_GetFlagStatus to set or reset status register for Tx
 */
uint8_t SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length ){

	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		//API in order to send data from SPI with interrupts
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLength = length;

		//Making SPI state as busy in Transmission
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enabling TXEIE control bit to get interrupt when TXE flag is set in Status Register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
	//ISR code for data handling

}

/*********************************************************************
 * @Function Name				- SPI_ReceiveDataIT
 *
 * @Description             	- This function receives data from SPI
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pSPIHandle]	- Handling of SPI port
 *  [uint8_t *pRxBuffer]    	- Pointer pointing to reception buffer
 *  [uint32_t length]    		- Size of data transfer
 *
 * @Return Type            		- Integer
 *
 * @Note              			- None
 */
uint8_t SPI_ReceiveDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length ){
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		//API in order to send data from SPI with interrupts
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLength = length;

		//Making SPI state as busy in Transmission
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Enabling TXEIE control bit to get interrupt when RXE flag is set in Status Register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
	//ISR code for data handling

}

/*********************************************************************/
/*** API for Peripheral Control ***/
/*********************************************************************
 * @Function Name			- SPI_PeriphControl
 *
 * @Description             - This function enables or disables peripheral for SPI
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base address of the SPI peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_PeriphControl (SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of SPI
	if (EnORDi ==  ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );
	}else{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}
}

/*********************************************************************/
/*** API for Configuring SSI Internally ***/
/*********************************************************************
 * @Function Name			- SPI_SSIConfig
 *
 * @Description             - This function configures SSI
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base address of the SPI peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_SSIConfig (SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of SPI
	if (EnORDi ==  ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );
	}else{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
	}
}


/*********************************************************************/
/*** API for Configuring SSOE Internally ***/
/*********************************************************************
 * @Function Name			- SPI_SSOEConfig
 *
 * @Description             - This function configures SSOE
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base address of the SPI peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_SSOEConfig (SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of SPI
	if (EnORDi ==  ENABLE){
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );
	}else{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE );
	}
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
	 //API in order to configure interrupts for SPI

	if(EnORDi == ENABLE){
		if (IRQNumber <= 31){
			//Program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			//Program ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			//Program ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else{
		if (IRQNumber <= 31){
			//Program ICER0 Register
			*NVIC_ICER0 |= (1 << (IRQNumber));
		}else if(IRQNumber > 31 && IRQNumber < 64){
			//Program ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			//Program ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
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
	//API in order to configure priority interrupts of SPI
	uint8_t IPRx = IRQNumber / 4;													//finding out IPR Register
	uint8_t IPRx_Section = IRQNumber % 4; 											//finding Bit Field of IPR Register
	uint8_t shift_amount = ( 8 * IPRx_Section ) + ( 8 - NO_OF_PR_BITS_IMPLEMENTED); //left shift from non-implemented bits to upper bits

	*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << shift_amount );
}

/*********************************************************************
 * @Function Name			- SPI_IRQHandling
 *
 * @Description             - This function handles the incoming interrupts for SPI
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pHandle]	- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_IRQHandling (SPI_Handle_t *pHandle){
	uint8_t temp1, temp2; 												//API in order to process the incoming interrupts of SPI

	//Checking for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	//Handling TXE
	if (temp1 && temp2){
		SPI_TXE_INTERRUPT_HANDLE(pHandle);
	}
	//Checking for RXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	// Handling RXNE
	if (temp1 && temp2){
		SPI_RXE_INTERRUPT_HANDLE(pHandle);
	}
	//Check for Overrun Error (OVR Flag)
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2){
			SPI_OVR_INTERRUPT_HANDLE(pHandle);
		}

}


/*********************************************************************
 * @Function Name			- SPI_CloseTx
 *
 * @Description             - This function closes the transmission
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pHandle]	- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_CloseTx (SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLength = 0;
	pSPIHandle->TxState = SPI_READY;
}



/*********************************************************************
 * @Function Name			- SPI_CloseRx
 *
 * @Description             - This function closes reception
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pHandle]	- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_CloseRx (SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLength = 0;
	pSPIHandle->RxState = SPI_READY;
}

/*********************************************************************
 * @Function Name			- SPI_ClearOVRFlag
 *
 * @Description             - This function clears the overrun flag
 *
 * @Input Parameter(s)
 *  [SPI_RegDef_t *pSPIx]	- Base Address of SPI peripheral
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void SPI_ClearOVRFlag (SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*********************************************************************
 * @Function Name			- SPIApplicationEventCallback
 *
 * @Description             - Function for callback of application after interrupt is over
 *
 * @Input Parameter(s)
 *  [SPI_Handle_t *pSPIHandle]	- Handle for SPI peripheral
 *  [uint8_t AppEvent			- Integer for event call
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
__attribute__((weak)) void SPIApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent){
	// This is a weak implementation, and application can override this function.
	// we do weak implementation by using GCC attribute keyword 'weak'
}
