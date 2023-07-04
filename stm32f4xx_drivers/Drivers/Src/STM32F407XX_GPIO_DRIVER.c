/*
 * STM32F407XX_GPIO_DRIVER.C
 *
 *  Created on: 16-Jun-2023
 *      Author: Rohan
 */


#include "STM32F407XX_GPIO_DRIVER.h"

/*********************************************************************/
/*** API for Peripheral Clock Setup ***/
/*********************************************************************
 * @Function Name			- GPIO_PeriphClockControl
 *
 * @Description             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_RegDef_t *pGPIOx]	- Base address of the GPIO peripheral
 *  [uint8_t EnORDi]        - ENABLE or DISABLE macros
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_PeriphClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnORDi){
	if (EnORDi ==  ENABLE){
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}else if (pGPIOx == GPIOJ){
			GPIOJ_PCLK_EN();
		}else if (pGPIOx == GPIOK){
			GPIOK_PCLK_EN();
		}
	}else{
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}else if (pGPIOx == GPIOJ){
			GPIOJ_PCLK_DI();
		}else if (pGPIOx == GPIOK){
			GPIOK_PCLK_DI();
		}
	}
}

/*********************************************************************/
/*** API for Initialize and De-Initialize ***/
/*********************************************************************
 * @Function Name					- GPIO_Init
 *
 * @Description             		- This function initializes the given GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_Handle_t *pGPIOHandle]	- Handling of GPIO port
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_Init (GPIO_Handle_t *pGPIOHandle ){
	/****** Configuring Mode of GPIO Pin ******/
	uint32_t temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing the register
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting the register value

	}else{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FE){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//Configure the Falling Trigger Selection Register
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//Clear the Rising Trigger Selection Register
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RE){
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//Configure the Rising Trigger Selection Register
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//Configure the Falling Trigger Selection Register
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_REFET){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//Configure the Falling Trigger Selection Register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//Configure the Rising Trigger Selection Register
		}
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = port_code << (temp2 * 4);

		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//Enabling EXTI Interrupt delivery using IMR
	}
/****** Configuring Speed of GPIO Pin ******/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

/****** Configuring Pull-Up Pull-Down Setting of GPIO Pin ******/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPDControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

/****** Configuring Pull-Up Pull-Down Setting of GPIO Pin ******/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPDControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

/****** Configuring Output Type of GPIO Pin ******/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);;
	pGPIOHandle->pGPIOx->OTYPER &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
/****** Configuring Alternate Functionality of GPIO Pin ******/
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1 = 0, temp2 = 0;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~ (0xF << (4 * temp2));
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFnMode << (4 * temp2));
	}
}


/*********************************************************************
 * @Function Name			- GPIO_DeInit
 *
 * @Description             - This function de-initialises the given GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_RegDef_t *pGPIOx]	- Base address of the GPIO peripheral
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG){
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}else if (pGPIOx == GPIOJ){
		GPIOJ_REG_RESET();
	}else if (pGPIOx == GPIOK){
		GPIOK_REG_RESET();
	}
}



/*********************************************************************/
/*** API for Data Read and Write ***/
/*********************************************************************
 * @Function Name			- GPIO_ReadFromInputPin
 *
 * @Description             - This function reads from input pin for the given GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_RegDef_t *pGPIOx]	- Base address of the GPIO peripheral
 *  [uint8_t PinNumber]     - Pin Number of the GPIO Port
 *
 * @Return Type            	- Integer 8 bit
 *
 * @Note              		- None
 */
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}


/*********************************************************************
 * @Function Name			- GPIO_ReadFromInputPort
 *
 * @Description             - This function reads input of the given GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_RegDef_t *pGPIOx]	- Base address of the GPIO peripheral
 *
 * @Return Type            	- Integer 16 bit
 *
 * @Note              		- None
 */
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}


/*********************************************************************
 * @Function Name			- GPIO_WriteToOutputPin
 *
 * @Description             - This function writes to the output pin for the given GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_RegDef_t *pGPIOx]	- Base address of the GPIO peripheral
 *  [uint8_t PinNumber]     - Pin Number
 *  [uint8_t Value]			- Enable or Disable value
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if (Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);	//Write 1 to output data register at the bit field corresponding to the pin number
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);	//Write 0 to output data register at the bit field corresponding to the pin number
	}

}


/*********************************************************************
 * @Function Name			- GPIO_WriteToOutputPort
 *
 * @Description             - This function writes to the output of the GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_RegDef_t *pGPIOx]	- Base address of the GPIO peripheral
 *  [uint16_t Value]        - Enable or Disable Value
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;	//Write 'Value' to entire output data port

}


/*********************************************************************
 * @Function Name			- GPIO_ToggleOutputPin
 *
 * @Description             - This function toggles output pin for the given GPIO port
 *
 * @Input Parameter(s)
 *  [GPIO_RegDef_t *pGPIOx]	- Base address of the GPIO peripheral
 *  [uint8_t PinNumber]     - Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);

}


/*********************************************************************/
/*** API for IRQ Configuration and ISR Handling ***/
/*********************************************************************
 * @Function Name			- GPIO_IRQInterruptConfig
 *
 * @Description             - This function provides configuration of interrupts for the given GPIO port
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
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnORDi){

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
 * @Function Name			- GPIO_IRQProrityConfig
 *
 * @Description             - This function provides configuration of priority of interrupts for the given GPIO port
 *
 * @Input Parameter(s)
 *  [uint8_t IRQNumber]		- Interrupt request number
 *  [uint8_t IRQPriority]   - Priority of interrupt request
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t IPRx = IRQNumber / 4;													//finding out IPR Register
	uint8_t IPRx_Section = IRQNumber % 4; 											//finding Bit Field of IPR Register
	uint8_t shift_amount = ( 8 * IPRx_Section ) + ( 8 - NO_OF_PR_BITS_IMPLEMENTED); //left shift from non-implemented bits to upper bits

	*(NVIC_PR_BASEADDR + (IPRx * 4)) |= (IRQPriority << shift_amount );
}


/*********************************************************************
 * @Function Name			- GPIO_IRQHandling
 *
 * @Description             - This function handles the incoming interrupts for the given GPIO port
 *
 * @Input Parameter(s)
 *  [uint8_t PinNumber]		- Pin Number
 *
 * @Return Type            	- None
 *
 * @Note              		- None
 */
void GPIO_IRQHandling (uint8_t PinNumber){

	if (EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber); //clear the EXTI PR register
	}

}


