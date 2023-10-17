/*
 * STM32407XX_I2C_DRIVER.h
 *
 *  Created on: Sep 16, 2023
 *      Author: Rohan
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "STM32F407XX.h"
/*
 * This is a configuration structure for a I2Cx Peripheral
 */
typedef struct{
	uint32_t 	I2C_SCLSpeed;
	uint8_t 	I2C_DeviceAddress;
	uint8_t 	I2C_ACKControl;
	uint16_t 	I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * This is a handle structure for I2Cx peripheral
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;		/*Holds the base address of the GPIO port to which the pin belongs to */
    I2C_Config_t I2C_Config;	/*This holds I2C configuration setting */
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000		// Standard Mode of 100 KHz Speed
#define I2C_SCL_SPEED_FM2K	200000		// Fast Mode of 200 KHz Speed
#define I2C_SCL_SPEED_FM4K	400000		// Fast Mode of 400 KHz Speed

/*
 * @I2C_ACK
 */
#define I2C_ACK_ENABLE	1
#define I2C_ACK_DISABLE	0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTYCYCLE_2		0
#define I2C_FM_DUTYCYCLE_16_9	19


/*******************************************************************/
/************		API's SUPPORTED BY THIS DRIVER		************/
/*******************************************************************/

/*** API for Peripheral Clock Setup ***/
void I2C_PeriphClockControl (I2C_RegDef_t *pI2Cx, uint8_t EnORDi);	//API in order to enable to disable peripheral clock of I2C

/*** API for Initialize and De-Initialize ***/
void I2C_Init (I2C_Handle_t *pI2CHandle ); 			//API in order to initialize I2C
void I2C_DeInit (I2C_RegDef_t *pI2Cx); 				//API in order to de-initialize I2C

/*** API for getting flag status ***/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*** API for Data Send and Receive ***/
void I2C_MasterSendData (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t SLaveAddr); 				//API in order to send data from I2C
//void I2C_ReceiveData (I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t length ); 			//API in order to receive data from I2C

/*** API for IRQ Configuration and ISR Handling ***/
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnORDi); 		//API in order to configure interrupts of I2C
void I2C_IRQPrConfig (uint8_t IRQNumber, uint8_t IRQPriority); 								//API in order to configure priority interrupts of I2C
//void I2C_IRQHandling (I2C_Handle_t *pHandle); 												//API in order to process the incoming interrupts of I2C

/*** API to enable I2C peripheral ***/
void I2C_PeriphControl(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);		//  API to enable or diable I2C peripheral

/*** API to configure SSI internally ***/
void I2C_SSIConfig(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);		//  API to configure SSI internally

/*** API to configure SSOE internally ***/
void I2C_SSOEConfig(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);		//  API to configure SSOE internally




#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
