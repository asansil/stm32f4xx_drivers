/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Jun 23, 2025
 *      Author: asans
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*Configuration structure for a GPIO pin*/
typedef struct{
	uint8_t GPIO_PinNumber;			//GPIO pin number (possible values from @GPIO_PIN_NUMBERS)
	uint8_t GPIO_PinMode;			//GPIO mode (possible values from @GPIO_PIN_MODES)
	uint8_t GPIO_PinSpeed;			//GPIO speed (possible values from @GPIO_PIN_SPEED)
	uint8_t GPIO_PinPuPdControl;	//GPIO pullup/pulldown control (possible values from @GPIO_PIN_PUPD)
	uint8_t GPIO_PinOPType;			//GPIO output type
	uint8_t GPIO_PinAltFunMode;		//GPIO alternate function mode

}GPIO_PinConfig_t;


/*Handle structure for a GPIO pin*/
typedef struct{
	GPIO_RegDef_t *pGPIOx;				//Base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//Pin configuration settings
}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0   	0
#define GPIO_PIN_NO_1   	1
#define GPIO_PIN_NO_2   	2
#define GPIO_PIN_NO_3  		3
#define GPIO_PIN_NO_4   	4
#define GPIO_PIN_NO_5   	5
#define GPIO_PIN_NO_6   	6
#define GPIO_PIN_NO_7   	7
#define GPIO_PIN_NO_8   	8
#define GPIO_PIN_NO_9   	9
#define GPIO_PIN_NO_10  	10
#define GPIO_PIN_NO_11  	11
#define GPIO_PIN_NO_12  	12
#define GPIO_PIN_NO_13  	13
#define GPIO_PIN_NO_14  	14
#define GPIO_PIN_NO_15  	15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0b00
#define GPIO_MODE_OUT	 	0b01
#define GPIO_MODE_ALTFN		0b10
#define GPIO_MODE_ANALOG	0b11
#define GPIO_MODE_IT_FT		4		//Interrupt falling edge
#define GPIO_MODE_IT_RT		5		//Interrupt raising edge
#define GPIO_MODE_IT_RFT	6		//Interrupt raising falling edge

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 	0	//Push-pull
#define GPIO_OP_TYPE_OD 	1	//Open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0b00	//Low speed
#define GPIO_SPEED_MEDIUM	0b01	//Medium speed
#define GPIO_SPEED_HIGH		0b10	//High speed
#define GPIO_SPEED_VHIGH	0b11	//Very high speed

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pullup/pulldown configurations
 */
#define GPIO_NO_PUPD		0b00	//Low speed
#define GPIO_PU				0b01	//Medium speed
#define GPIO_PD				0b10	//High speed


/*****************************************************************************************
 * 								APIs supported by this driver
 * 					For more information check the function definitions
 *****************************************************************************************/
/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Init and deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
