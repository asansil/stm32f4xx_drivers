/*
 * 005button_interrupt.c
 *
 *  Created on: Jun 25, 2025
 *      Author: asans
 */


#include "stm32f407xx.h"

#define HIGH			1
#define BTN_PRESSED		HIGH


void delay(void){
	for(uint32_t i = 0; i < 500000; i++);
}


int main(void){

	// Green led
	GPIO_Handle_t GpioGLed;

	GpioGLed.pGPIOx = GPIOD;
	GpioGLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioGLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioGLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioGLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioGLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// User button
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioGLed);

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);


	return 0;
}

void EXTI0_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}
