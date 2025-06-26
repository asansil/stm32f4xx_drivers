/*
 * 001led_toggle.c
 *
 *  Created on: Jun 24, 2025
 *      Author: asans
 */

#include "stm32f407xx.h"


void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void){

	// Green led
	GPIO_Handle_t GpioGLed;

	GpioGLed.pGPIOx = GPIOD;
	GpioGLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioGLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioGLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GpioGLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioGLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Orange led
	GPIO_Handle_t GpioOLed;

	GpioOLed.pGPIOx = GPIOD;
	GpioOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GpioOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioGLed);
	GPIO_Init(&GpioOLed);


	//GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, ENABLE);
	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		//GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		delay();
	}
	return 0;
}

