/*
 * STM32F407XX_GPIO_DRIVER.h
 *
 *  Created on: 16-Jun-2023
 *      Author: Rohan
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "STM32F407XX.h"

/*
 * This is a configuration structure for a GPIO Pin
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				//possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				//possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPUPDControl;		//possible values from @GPIO_PIN_PULLUP_PULLDOWN
	uint8_t GPIO_PinOPType;				//possible values from @GPIO_PIN_OUTPUT_TYPE
	uint8_t GPIO_PinAltFnMode;
}GPIO_PinConfig_t;

/*
 * This is a handle structure for a GPIO Pin
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx;	/*Holds the base address of the GPIO port to which the pin belongs to */
    GPIO_PinConfig_t GPIO_PinConfig;	/*This holds GPIO pin configuration setting */
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBER
 * GPIO pin all possible numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * @GPIO_PIN_MODES
 * GPIO pin all possible modes
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_INT_FE		4	//GPIO Interrupt Falling Edge
#define GPIO_MODE_INT_RE		5	//GPIO Interrupt Rising Edge
#define GPIO_MODE_INT_REFET		6	//GPIO Interrupt Rising Edge Falling Edge Trigger


/*
 * @GPIO_PIN_OUTPUT_TYPE
 * GPIO pin all possible output type
 */
#define GPIO_OTYPE_PP	0 	//Push-Pull Output Type
#define GPIO_OTYPE_OD	1	//Open Drain Output Type


/*
 * @GPIO_PIN_SPEED
 * GPIO pin all possible output speeds
 */
#define GPIO_OSPEED_LOW		0 	//Low Speed
#define GPIO_OSPEED_MED		1	//Medium Speed
#define GPIO_OSPEED_HI		2	//High Speed
#define GPIO_OSPEED_VHI		3	//Very High Speed


/*
 * @GPIO_PIN_PULLUP_PULLDOWN
 * GPIO pin all possible pull-up pull-down register
 */
#define GPIO_PIN_NOPUPD		0 	//No Pull-Up Pull-Down
#define GPIO_PIN_PU			1	//Pull-Up
#define GPIO_PIN_PD			2	//Pull-Down



/*******************************************************************/
/************		API's SUPPORTED BY THIS DRIVER		************/
/*******************************************************************/

/*** API for Peripheral Clock Setup ***/
void GPIO_PeriphClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnORDi);	//API in order to enable to disable peripheral clock of the given GPIO port

/*** API for Initialize and De-Initialize ***/
void GPIO_Init (GPIO_Handle_t *pGPIOHandle ); 			//API in order to initialize the given GPIO port
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx); 				//API in order to de-initialize the given GPIO port

/*** API for Data Read and Write ***/
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); 					//API in order to read from input pin of the given GPIO port
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx); 									//API in order to read from the input port of the given GPIO port
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value); 		//API in order to write to the output pin of the given GPIO port
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value); 						//API in order to write to the output port of the given GPIO port
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); 						//API in order to toggle output pin of the given GPIO port

/*** API for IRQ Configuration and ISR Handling ***/
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnORDi); 		//API in order to configure interrupts of the given GPIO port
void GPIO_IRQPrConfig (uint8_t IRQNumber, uint8_t IRQPriority); 							//API in order to configure priority interrupts of the given GPIO port
void GPIO_IRQHandling (uint8_t PinNumber); 													//API in order to process the incoming interrupts the given GPIO port






#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
