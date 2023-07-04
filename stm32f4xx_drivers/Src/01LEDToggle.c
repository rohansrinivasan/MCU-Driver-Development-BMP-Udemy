/*
 * 01LEDToggle.c
 *
 *  Created on: 22-Jun-2023
 *      Author: Rohan
 */


#include "STM32F407XX.h"
#include "STM32F407XX_GPIO_DRIVER.h"

void delay(void){
	for(uint32_t i = 0; i < 200000; i++);
}
int main(void){

	GPIO_Handle_t GPIO_LED;
	GPIO_LED.pGPIOx = GPIOD;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_VHI;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_NOPUPD;

	GPIO_PeriphClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_LED);
	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
		delay();
	}
	return 0;

}
