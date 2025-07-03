/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Jun 23, 2025
 *      Author: asans
 */

#include <stm32f407xx_gpio_driver.h>

/*************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- Enables or disables the peripheral clock for the given GPIO port
 *
 * @param[in]  		- Base address of the GPIO peripheral
 * @param[in]  		- ENABLE or DISABLE macros
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {
	/*
	 * For when GPIO registers are spaced evenly (faster)
	 */
	uint8_t port = ((uint32_t) pGPIOx - GPIOA_BASEADDR) / 0x400;
	if (EnOrDi == ENABLE) {
		RCC->AHB1ENR |= (1 << port);
	} else {
		RCC->AHB1ENR &= ~(1 << port);
	}

	/*
	 * For when GPIO registers are not spaced evenly
	 */
	/*
	 if(EnOrDi == ENABLE){
	 if(pGPIOx == GPIOA){
	 GPIOA_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOB){
	 GPIOB_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOC){
	 GPIOC_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOD){
	 GPIOD_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOE){
	 GPIOE_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOF){
	 GPIOF_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOG){
	 GPIOG_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOH){
	 GPIOH_PCLK_EN();
	 }
	 else if(pGPIOx == GPIOI){
	 GPIOI_PCLK_EN();
	 }
	 }
	 else{
	 if(pGPIOx == GPIOA){
	 GPIOA_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOB){
	 GPIOB_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOC){
	 GPIOC_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOD){
	 GPIOD_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOE){
	 GPIOE_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOF){
	 GPIOF_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOG){
	 GPIOG_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOH){
	 GPIOH_PCLK_DI();
	 }
	 else if(pGPIOx == GPIOI){
	 GPIOI_PCLK_DI();
	 }
	 }
	 */

}

/*************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Sets the GPIO pin configuration (pin number, mode, speed, pullup/pulldown control, output type, alternate function mode)
 *
 * @param[in]  		- GPIO_Handle_t structure which contains the configuration info
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp32 = 0;	// temp32 register

	// Enable the pheripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of the GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {	// Non interrupt mode
		temp32 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp32; //setting

	} else {	// Interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the corresponding RTSR bit
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// 1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the corresponding FTSR bit
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1. Configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		/*get port if GPIO registers are spaced evenly*/
		uint8_t port = ((uint32_t) pGPIOHandle->pGPIOx - GPIOA_BASEADDR)
				/ 0x400;

		/*get port if GPIO registers are not spaced evenly*/
		//uint8_t port = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		uint8_t extiRegister = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t pinOffset = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)
				* 4;

		temp32 = (port & 0xF) << pinOffset;	//Note: the port bits are limited in length for security
		SYSCFG_PCLK_EN();	// Enable sysconfig
		SYSCFG->EXTICR[extiRegister] &= ~(0xF << pinOffset);	// clearing
		SYSCFG->EXTICR[extiRegister] |= temp32;	// Setting

		// 3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Configure the speed
	temp32 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp32; //setting

	// 3. Configure the pullup/pulldown settings
	temp32 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp32; //setting

	// 4. Configure the output type
	temp32 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp32; //setting

	// 5. Configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 8;	//high or low
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8;	//register pin offset

		pGPIOHandle->pGPIOx->AFRL[temp1] &= ~(0b1111 << (4 * temp2));// clearing
		pGPIOHandle->pGPIOx->AFRL[temp1] |=
				((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4 * temp2));//setting
	}

}

/*************************************************************
 * @fn				- GPIO_Deinit
 *
 * @brief			- Resets the GPIO port configuration
 *
 * @param[in]  		- GPIO_RegDef_t structure which contains the GPIO port address
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	/*
	 * For when GPIO registers are spaced evenly (faster)
	 */
	uint8_t port = ((uint32_t) pGPIOx - GPIOA_BASEADDR) / 0x400;
	RCC->AHB1RSTR |= (1 << port);	//reset
	RCC->AHB1RSTR &= ~(1 << port);	//exit reset state

	/*
	 * For when GPIO registers are not spaced evenly
	 */
	/*
	 if(pGPIOx == GPIOA){
	 GPIOA_REG_RESET();
	 }
	 else if(pGPIOx == GPIOB){
	 GPIOB_REG_RESET();
	 }
	 else if(pGPIOx == GPIOC){
	 GPIOC_REG_RESET();
	 }
	 else if(pGPIOx == GPIOD){
	 GPIOD_REG_RESET();
	 }
	 else if(pGPIOx == GPIOE){
	 GPIOE_REG_RESET();
	 }
	 else if(pGPIOx == GPIOF){
	 GPIOF_REG_RESET();
	 }
	 else if(pGPIOx == GPIOG){
	 GPIOG_REG_RESET();
	 }
	 else if(pGPIOx == GPIOH){
	 GPIOH_REG_RESET();
	 }
	 else if(pGPIOx == GPIOI){
	 GPIOI_REG_RESET();
	 }
	 */

}

/*************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- Reads data from a GPIO pin
 *
 * @param[in]  		- GPIO_RegDef_t structure which defines the GPIOx register
 * @param[in]  		- The pin number to read
 *
 * @return 			- Value stored in the Input Data Register for the selected pin (0 or 1)
 *
 * @Note			- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;
}

/*************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Reads data from a GPIO port
 *
 * @param[in]  		- GPIO_RegDef_t structure which defines the GPIOx register
 *
 * @return 			- Value stored in the Input Data Register
 *
 * @Note			- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value = (uint16_t) ((pGPIOx->IDR) & 0xFFFF);
	return value;
}

/*************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- Writes a value to the desired GPIO pin
 *
 * @param[in]  		- GPIO_RegDef_t structure which defines the GPIOx register
 * @param[in]  		- Selected pin number
 * @param[in]  		- Value to write
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (0x1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}
}

/*************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- Writes a value to the desired GPIO port
 *
 * @param[in]  		- GPIO_RegDef_t structure which defines the GPIOx register
 * @param[in]  		- Value to write
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/*************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- Toggles the value of a GPIO pin
 *
 * @param[in]  		- GPIO_RegDef_t structure which defines the GPIOx register
 * @param[in]  		- Pin number to toggle
 *
 * @return 			-
 *
 * @Note			- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/************************************************************* TODO
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[in]  		-
 * @param[in]  		-
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
	/*
	 * Smart option knowing all NVIC registers are evenly spaced and next to each other
	 */
	if (IRQNumber > 81)
		return; // Protection (max 81 IRQ numbers)

	uint8_t NVIC_reg_idx = IRQNumber / 32;
	uint8_t bit_pos = IRQNumber % 32;

	if (EnOrDi == ENABLE) {
		*(NVIC_ISER0 + NVIC_reg_idx) |= (1 << bit_pos);
	} else {
		*(NVIC_ICER0 + NVIC_reg_idx) |= (1 << bit_pos);
	}

	/*
	 * Tedious option
	 */
	/*uint8_t temp1 = IRQNumber / 32;
	 if(EnOrDi == ENABLE){
	 switch(temp1){
	 case 0:
	 *NVIC_ISER0 |= (1<< IRQNumber);
	 break;
	 case 1:
	 *NVIC_ISER1 |= (1<< IRQNumber % 32);
	 break;
	 case 2:
	 *NVIC_ISER2 |= (1<< IRQNumber % 32);
	 break;
	 case 3:
	 *NVIC_ISER3 |= (1<< IRQNumber % 32);
	 break;
	 case 4:
	 *NVIC_ISER4 |= (1<< IRQNumber % 32);
	 break;
	 case 5:
	 *NVIC_ISER5 |= (1<< IRQNumber % 32);
	 break;
	 case 6:
	 *NVIC_ISER6 |= (1<< IRQNumber % 32);
	 break;
	 case 7:
	 *NVIC_ISER7 |= (1<< IRQNumber % 32);
	 break;
	 }
	 } else {
	 //Repeat for ICER
	 }*/
}

/************************************************************* TODO
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]  		-
 * @param[in]  		- IRQ priority from @NVIC_IRQ_PRI
 *
 * @return 			-
 *
 * @Note			- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;	//Find IPR register
	uint8_t iprx_section = IRQNumber % 4;	//Find IPR register section

	/*
	 * Register priority value fields are eight bits wide, and non-implemented low-order bits read as zero and ignore writes.
	 */
	uint8_t shift_amount = iprx_section * 8 + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) &= ~(0xF << shift_amount);//Clear XXX: Maybe have to change if @NO_PR_BITS_IMPLEMENTED changes
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);	//Set
}

/*************************************************************	TODO
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	//Clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}
}
