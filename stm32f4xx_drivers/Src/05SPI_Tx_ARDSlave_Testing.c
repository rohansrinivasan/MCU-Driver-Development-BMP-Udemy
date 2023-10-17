/*
 * 05SPI_Tx_ARDSlave_Testing.c
 *
 *  Created on: Jul 14, 2023
 *      Author: Rohan
 */


#include "STM32F407XX.h"
#include <string.h>
#include "STM32F407XX_GPIO_DRIVER.h"
#include "STM32F407XX_SPI_DRIVER.h"

#define BUTTON_PRESSED HIGH

void delay(void){
	for(uint32_t i = 0; i < 200000; i++);
}

/*
 * SPI2 Pin Modes on GPIO Pins
 *
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
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

	GPIO_PeriphClockControl(GPIOB, ENABLE);

	//Activate NSS Line
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	GPIO_Init(&SPIPins);

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
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SClckConfig = SPI_SCLCK_SPEED_DIV8; //generated serial clock of 2Mhz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);


}

//Function to enable button for Tx
void GPIO_ButtonInit(void){

	GPIO_Handle_t GPIO_BTN;
	GPIO_BTN.pGPIOx = GPIOA;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_VHI;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_NOPUPD;

	GPIO_Init(&GPIO_BTN);

}

int main(void){
	char user_data[] = "Hi Barbie!";

	GPIO_ButtonInit();

	SPI2_GPIOInits();	//Function to initialize GPIO pins to behave as SPI2 pins

	SPI2_Inits();		//Function to initialize SPI2 peripheral parameters

	SPI_SSOEConfig(SPI2, ENABLE);	//Function to initialize SSOE

	while(1){
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));		//Initializes Tx only when button is pressed

		delay();	//To avoid de-bouncing

		SPI_PeriphControl(SPI2, ENABLE);	//Function to enable the SPI2 peripheral

		uint8_t datalen = strlen(user_data);
		SPI_SendData(SPI2, &datalen, 1);	// to send length information

		SPI_SendData(SPI2,(uint8_t*)user_data, strlen(user_data));	// to send the message

		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		SPI_PeriphControl(SPI2, DISABLE);	//Function to disable SPI2 peripheral after data is sent
	}

	return 0;

}
