/*
 * STM32F407XX_I2C_DRIVER.c
 *
 *  Created on: Sep 16, 2023
 *      Author: Rohan
 */

#include "STM32F407XX_I2C_DRIVER.h"

uint32_t RCC_GetPLLOutputClock(){
    return;
}

uint32_t RCC_GetPCLK1Value(void){
    //Value of APB1 Clock to be used for I2C Peripheral
       uint32_t pclk1, SystemClk;
       uint8_t clksrc, AHBpre_factor, AHBprescalar, APBpre_factor, APBprescalar;

       uint16_t AHB_PreScalar[8] = {2,4,8,16,32,64,128,256,512};
       uint16_t APB_PreScalar[4] = {2,4,8,16};

       clksrc = ((RCC->RCC_CFGR >> 2 & 0x3));   // Reading bits 3 and 2 for SWS (system clock switch) Status. 
                                                //Done by right shifting bits 3 and 2 by 2 bit fields and masking all other bits

        if(clksrc == 0){
            SystemClk = 16000000;       // If SWS is 0x00 i.e. 0, then clock is HSI i.e. 16Mhz (specific for STM32F407G-Disc)
        }
        if(clksrc == 1){
            SystemClk = 8000000;       // If SWS is 0x01 i.e. 1, then clock is HEI i.e. 8Mhz (specific for STM32F407G-Disc)
        }
        if(clksrc == 2){
            SystemClk = RCC_GetPLLOutputClock();       // If SWS is 0x10 i.e. 2, then clock is PLL (not used mostly)
        }

       //For AHB
        AHBprescalar = ((RCC->RCC_CFGR >> 4 & 0xF));   // Reading bits 4 to 7 to determine AHB prescalar. 
                                                    //Done by right shifting bit 4 to 7 by 4 bit fields and masking all other bits
        if(AHBprescalar < 8){
            AHBpre_factor = 1;       // if Value of AHB prescalar factor is less than 8 then its not divided
        }else{
            AHBpre_factor = AHB_PreScalar[AHBprescalar - 8];
        }

        //For APB
        APBprescalar = ((RCC->RCC_CFGR >> 4 & 0x7));   // Reading bits 10 to 12 to determine APB prescalar. 
                                                    //Done by right shifting bit 10 to 12 by 10 bit fields and masking all other bits
        if(APBprescalar < 10){
            APBpre_factor = 1;       // if Value of APB prescalar factor is less than 8 then its not divided
        }else{
            APBpre_factor = APB_PreScalar[APBprescalar - 10];
        }

        pclk1 = ((SystemClk / AHBpre_factor) / APBpre_factor);
        return pclk1;   
}

/*********************************************************************/
/*** API for Peripheral Clock Setup ***/
/*********************************************************************
 * @Function Name			- I2C_PeriphClockControl
 *
 * @Description             - This function enables or disables peripheral clock for I2C
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base address of the I2C peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_PeriphClockControl (I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of I2C
	if (EnORDi ==  ENABLE){
		if (pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else{
		if (pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}


/*********************************************************************/
/*** API for Initialize and De-Initialize ***/
/*********************************************************************
 * @Function Name					- I2C_Init
 *
 * @Description             		- This function initializes I2C
 *
 * @Input Parameter(s)
 *  [I2C_Handle_t *pI2CHandle]	- Handling of I2C Port
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_Init (I2C_Handle_t *pI2CHandle ){
	//Enabling peripheral clock control for I2C
	I2C_PeriphClockControl(pI2CHandle->pI2Cx, ENABLE);
	//API in order to initialize I2C
	uint32_t tempreg = 0;

	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK; // configuring device mode
    pI2CHandle->pI2Cx->CR1 = tempreg; 
	
    //Configuring FREQ field of CR2
    tempreg = 0;
    tempreg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F); 

	//configuring I2C device own address (OAR)
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;   //Shift by 1 to not include ADD0 bit
    tempreg |= (1 << 14);       //Reserved bit 14 needs to set as 1 by the software (reference manual)
    pI2CHandle->pI2Cx->OAR1 = tempreg;
    
	//Configuring Clock Control Register
    uint16_t CCR_Value = 0;
    tempreg = 0;
	if ( pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
        //Standard Mode (Tlow = Thigh)
        CCR_Value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempreg |= (CCR_Value & 0xFFF);     //Mask first 12 bits to read CCR
    }else{
        //Fast Mode
        tempreg |= (1 << 15);
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTYCYCLE_2){
            //(Tlow = 2Thigh)(Duty Cycle 0)
            CCR_Value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }else{
            //(Tlow = 1.7Thigh)(Duty Cycle 1)
            CCR_Value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        tempreg |= (CCR_Value & 0xFFF);  
    }
    pI2CHandle->pI2Cx->CCR = tempreg;

}


/*********************************************************************
 * @Function Name			- I2C_DeInit
 *
 * @Description             - This function de-initializes I2C
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base address of the I2C peripheral
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_DeInit (I2C_RegDef_t *pI2Cx){
	//API in order to de-initialize I2C
	if (pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C2){
		I2C2_REG_RESET();
	}else if (pI2Cx == I2C3){
		I2C3_REG_RESET();
	}else if (pI2Cx == I2C4){
		I2C4_REG_RESET();
	}
}

/*********************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************/
/*** API for Sending and Receiving Data ***/
/*********************************************************************
 * @Function Name			- I2C_SendData
 *
 * @Description             - This function sends data from I2C
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base address of the I2C peripheral
 *  [uint8_t *pTxBuffer]    - Pointer pointing to transmission buffer
 *  [uint32_t length]    	- Size of data transfer
 *
 * @Return Type            	- None
 *
 * @Note              		- Contains sub function @I2C_GetFlagStatus to set or reset status register for Tx
 */


void I2C_SendData (I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t length ){
	//API in order to send data from I2C
	while (length > 0){
		while( I2C_GetFlagStatus(pI2Cx, I2C_TXE_FLAG) == FLAG_RESET );	// waiting until TXE is set

		if ((pI2Cx->CR1 & ( 1 << I2C_CR1_DFF))){	// to check DFF bit in CR1
			// 16 bit DFF
			pI2Cx->DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}else{
			// 8 bit DFF
			pI2Cx->DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}
	}
}



/*********************************************************************
 * @Function Name			- I2C_ReceiveData
 *
 * @Description             - This function receives data from I2C
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base address of the I2C peripheral
 *  [uint8_t *pRxBuffer]    - Pointer pointing to reception buffer
 *  [uint32_t length]    	- Size of data transfer
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_ReceiveData (I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t length ){
	//API in order to receive data from I2C
	while (length > 0){
		while( I2C_GetFlagStatus(pI2Cx, I2C_RXNE_FLAG) == FLAG_RESET );	// waiting until RXE is set

		if ((pI2Cx->CR1 & ( 1 << I2C_CR1_DFF))){	// to check DFF bit in CR1
			// 16 bit DFF
			*((uint16_t*)pRxBuffer) = pI2Cx->DR;
			length--;
			length--;
			(uint16_t*)pRxBuffer++;
		}else{
			// 8 bit DFF
			pI2Cx->DR = *pRxBuffer;
			length--;
			pRxBuffer++;
		}
	}

}

/*********************************************************************/
/*** API for Sending and Receiving Data with Interrupts ***/
/*********************************************************************
 * @Function Name				- I2C_SendDataIT
 *
 * @Description             	- This function sends data from I2C with interrupt handling
 *
 * @Input Parameter(s)
 *  [I2C_Handle_t *pI2CHandle]	- Handling of I2C port
 *  [uint8_t *pTxBuffer]    	- Pointer pointing to transmission buffer
 *  [uint32_t length]    		- Size of data transfer
 *
 * @Return Type            		- Integer
 *
 * @Note              			- Contains sub function @I2C_GetFlagStatus to set or reset status register for Tx
 */
uint8_t I2C_SendDataIT (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length ){

	uint8_t state = pI2CHandle->TxState;
	if(state != I2C_BUSY_IN_TX){
		//API in order to send data from I2C with interrupts
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLength = length;

		//Making I2C state as busy in Transmission
		pI2CHandle->TxState = I2C_BUSY_IN_TX;

		//Enabling TXEIE control bit to get interrupt when TXE flag is set in Status Register
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_TXEIE);
	}

	return state;
	//ISR code for data handling

}

/*********************************************************************
 * @Function Name				- I2C_ReceiveDataIT
 *
 * @Description             	- This function receives data from I2C
 *
 * @Input Parameter(s)
 *  [I2C_Handle_t *pI2CHandle]	- Handling of I2C port
 *  [uint8_t *pRxBuffer]    	- Pointer pointing to reception buffer
 *  [uint32_t length]    		- Size of data transfer
 *
 * @Return Type            		- Integer
 *
 * @Note              			- None
 */
uint8_t I2C_ReceiveDataIT (I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length ){
	uint8_t state = pI2CHandle->RxState;
	if(state != I2C_BUSY_IN_RX){
		//API in order to send data from I2C with interrupts
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLength = length;

		//Making I2C state as busy in Transmission
		pI2CHandle->RxState = I2C_BUSY_IN_RX;

		//Enabling TXEIE control bit to get interrupt when RXE flag is set in Status Register
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_RXNEIE);
	}

	return state;
	//ISR code for data handling

}

/*********************************************************************/
/*** API for Peripheral Control ***/
/*********************************************************************
 * @Function Name			- I2C_PeriphControl
 *
 * @Description             - This function enables or disables peripheral for I2C
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base address of the I2C peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_PeriphControl (I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of I2C
	if (EnORDi ==  ENABLE){
		pI2Cx->CR1 |= ( 1 << I2C_CR1_SPE );
	}else{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_SPE );
	}
}

/*********************************************************************/
/*** API for Configuring SSI Internally ***/
/*********************************************************************
 * @Function Name			- I2C_SSIConfig
 *
 * @Description             - This function configures SSI
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base address of the I2C peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_SSIConfig (I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of I2C
	if (EnORDi ==  ENABLE){
		pI2Cx->CR1 |= ( 1 << I2C_CR1_SSI );
	}else{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_SSI );
	}
}


/*********************************************************************/
/*** API for Configuring SSOE Internally ***/
/*********************************************************************
 * @Function Name			- I2C_SSOEConfig
 *
 * @Description             - This function configures SSOE
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base address of the I2C peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_SSOEConfig (I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	//API in order to enable to disable peripheral clock of I2C
	if (EnORDi ==  ENABLE){
		pI2Cx->CR2 |= ( 1 << I2C_CR2_SSOE );
	}else{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_SSOE );
	}
}



/*********************************************************************/
/*** API for IRQ Configuration and ISR Handling ***/
/*********************************************************************
 * @Function Name			- I2C_IRQInterruptConfig
 *
 * @Description             - This function provides configuration of interrupts for I2C
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
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnORDi){
	 //API in order to configure interrupts for I2C

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
 * @Function Name			- I2C_IRQProrityConfig
 *
 * @Description             - This function provides configuration of priority of interrupts for I2C
 *
 * @Input Parameter(s)
 *  [uint8_t IRQNumber]		- Interrupt request number
 *  [uint8_t IRQPriority]   - Priority of interrupt request
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_IRQPrConfig (uint8_t IRQNumber, uint8_t IRQPriority){
	//API in order to configure priority interrupts of I2C
	uint8_t IPRx = IRQNumber / 4;													//finding out IPR Register
	uint8_t IPRx_Section = IRQNumber % 4; 											//finding Bit Field of IPR Register
	uint8_t shift_amount = ( 8 * IPRx_Section ) + ( 8 - NO_OF_PR_BITS_IMPLEMENTED); //left shift from non-implemented bits to upper bits

	*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << shift_amount );
}

/*********************************************************************
 * @Function Name			- I2C_IRQHandling
 *
 * @Description             - This function handles the incoming interrupts for I2C
 *
 * @Input Parameter(s)
 *  [I2C_Handle_t *pHandle]	- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_IRQHandling (I2C_Handle_t *pHandle){
	uint8_t temp1, temp2; 												//API in order to process the incoming interrupts of I2C

	//Checking for TXE flag
	temp1 = pHandle->pI2Cx->SR & (1 << I2C_SR_TXE);
	temp2 = pHandle->pI2Cx->CR2 & (1 << I2C_CR2_TXEIE);
	//Handling TXE
	if (temp1 && temp2){
		I2C_TXE_INTERRUPT_HANDLE(pHandle);
	}
	//Checking for RXE flag
	temp1 = pHandle->pI2Cx->SR & (1 << I2C_SR_RXNE);
	temp2 = pHandle->pI2Cx->CR2 & (1 << I2C_CR2_RXNEIE);
	// Handling RXNE
	if (temp1 && temp2){
		I2C_RXE_INTERRUPT_HANDLE(pHandle);
	}
	//Check for Overrun Error (OVR Flag)
	temp1 = pHandle->pI2Cx->SR & (1 << I2C_SR_OVR);
	temp2 = pHandle->pI2Cx->CR2 & (1 << I2C_CR2_ERRIE);
	if (temp1 && temp2){
			I2C_OVR_INTERRUPT_HANDLE(pHandle);
		}

}


/*********************************************************************
 * @Function Name			- I2C_CloseTx
 *
 * @Description             - This function closes the transmission
 *
 * @Input Parameter(s)
 *  [I2C_Handle_t *pHandle]	- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_CloseTx (I2C_Handle_t *pI2CHandle){
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_TXEIE);
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLength = 0;
	pI2CHandle->TxState = I2C_READY;
}



/*********************************************************************
 * @Function Name			- I2C_CloseRx
 *
 * @Description             - This function closes reception
 *
 * @Input Parameter(s)
 *  [I2C_Handle_t *pHandle]	- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_CloseRx (I2C_Handle_t *pI2CHandle){
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_RXNEIE);
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLength = 0;
	pI2CHandle->RxState = I2C_READY;
}

/*********************************************************************
 * @Function Name			- I2C_ClearOVRFlag
 *
 * @Description             - This function clears the overrun flag
 *
 * @Input Parameter(s)
 *  [I2C_RegDef_t *pI2Cx]	- Base Address of I2C peripheral
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void I2C_ClearOVRFlag (I2C_RegDef_t *pI2Cx){
	uint8_t temp;
	temp = pI2Cx->DR;
	temp = pI2Cx->SR;
	(void)temp;
}

/*********************************************************************
 * @Function Name			- I2CApplicationEventCallback
 *
 * @Description             - Function for callback of application after interrupt is over
 *
 * @Input Parameter(s)
 *  [I2C_Handle_t *pI2CHandle]	- Handle for I2C peripheral
 *  [uint8_t AppEvent			- Integer for event call
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
__attribute__((weak)) void I2CApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent){
	// This is a weak implementation, and application can override this function.
	// we do weak implementation by using GCC attribute keyword 'weak'
}
