/*
 * 02LEDToggleButton.c
 *
 *  Created on: 26-Jun-2023
 *      Author: Rohan
 */


#include "STM32F407XX.h"
#include "STM32F407XX_GPIO_DRIVER.h"


#define BUTTON_PRESSED HIGH

void delay(void){
	for(uint32_t i = 0; i < 200000; i++);
}
int main(void){

	GPIO_Handle_t GPIO_LED, GPIO_BUTTON;
	GPIO_LED.pGPIOx = GPIOD;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_VHI;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_NOPUPD;

	GPIO_PeriphClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_LED);

	GPIO_BUTTON.pGPIOx = GPIOA;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_VHI;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_NOPUPD;

	GPIO_PeriphClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_BUTTON);
	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BUTTON_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
		}
	}
	return 0;

}
