/*
 * 04SPI_Tx_Testing.c
 *
 *  Created on: 07-Jul-2023
 *      Author: Rohan
 */


#include "STM32F407XX.h"
#include <string.h>
/*
 * SPI2 Pin Modes on GPIO Pins Alternate Functionality
 * a
 * PB12 --> SPI2_NSS
 * PB13 --> SPI2_SCLK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * Alternate Function Mode: 5
 *
 */

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFnMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_NOPUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_VHI;

//	//Activate NSS Line
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
//	GPIO_Init(&SPIPins);

	//Activate SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

//	//Activate MISO Line
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	//Activate MOSI Line
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);



}

void SPI2_Inits(void){
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SClckConfig = SPI_SCLCK_SPEED_DIV2; //generated serial clock of 8Mhz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&SPI2Handle);


}
int main(void){
	char user_data[] = "Hello World";

	SPI2_GPIOInits();	//Function to initialize GPIO pins to behave as SPI2 pins

	SPI2_Inits();		//Function to initialize SPI2 peripheral parameters

	SPI_SSIConfig(SPI2, ENABLE); 	//Function to make NSS Signal internally high as
								    //when SSI is enabled, NSS is automatically pulled to ground
									//which gives enables MODF error and resets master to 0. So no signal is sent

	SPI_PeriphControl(SPI2, ENABLE);	//Function to enable the SPI2 peripheral

	SPI_SendData(SPI2,(uint8_t *)user_data, strlen(user_data));

//	while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

	SPI_PeriphControl(SPI2, DISABLE);	//Function to disable SPI2 peripheral after data is sent

	while (1);

	return 0;

}
