/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 27, 2025
 *      Author: asans
 */

#include "stm32f407xx_spi_driver.h"


static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_IT_Handle(SPI_Handle_t *pSPIHandle);



/************************************************************* TODO
 * @fn				- SPI_PeriClockControl
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/************************************************************* TODO
 * @fn				- SPI_Init
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* 1. Configure the SPI_CR1 register */
	uint32_t tempReg = 0;

	// 1.1. Configure the device mode
	tempReg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// 1.2. Configure the bus configuration
	switch (pSPIHandle->SPIConfig.SPI_BusConfig) {
	case SPI_BUS_CONFIG_FD:
		//BIDIMODE cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		break;
	case SPI_BUS_CONFIG_HD:
		//BIDIMODE set
		tempReg |= (1 << SPI_CR1_BIDIMODE);
		break;
	case SPI_BUS_CONFIG_SIMPLEX_RXONLY:
		//BIDIMODE cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY set
		tempReg |= (1 << SPI_CR1_RXONLY);
		break;
	}

	// 1.3. Configure the baud rate
	tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 1.4. Configure the dataframe format
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 1.5. Configure clock polarity
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 1.6. Configure clock phase
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 1.7. Configure software slave management
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 &= ~0x7FFF;	//Clear
	pSPIHandle->pSPIx->CR1 |= tempReg;	//Set


}

/*************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}



/*************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	} else{
		return FLAG_RESET;
	}
}

/*************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- This is a blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while(Len > 0){
		// 1. Wait until the TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16 bit DFF
			// 2.1. Load the data to the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);	//XXX: maybe corrupting reserved bits?
			Len -= 2;
			pTxBuffer ++;
			pTxBuffer ++;
		} else{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;	//XXX: maybe corrupting reserved bits?
			Len --;
			pTxBuffer ++;
		}

	}
}

/*************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while(Len > 0){
		// 1. Wait until the RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16 bit DFF
			// 2.1. Load the data from DR to RxBuffer address
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len -= 2;
			pRxBuffer ++;
			pRxBuffer ++;
		} else{
			// 8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len --;
			pRxBuffer ++;
		}
	}
}



/*************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}



/*************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}



/************************************************************* TODO
 * @fn				- SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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
 * @fn				- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;	//Find IPR register
	uint8_t iprx_section = IRQNumber % 4;	//Find IPR register section

	/*
	 * Register priority value fields are eight bits wide, and non-implemented low-order bits read as zero and ignore writes.
	 */
	uint8_t shift_amount = iprx_section * 8 + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) &= ~(0xF << shift_amount);//Clear XXX: Maybe have to change if @NO_PR_BITS_IMPLEMENTED changes
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);	//Set
}






/*************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_STATE_BUSY_TX) {
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2. Mark the SPI state as busy in transmission so that no other
		//	 code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_STATE_BUSY_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}



/*************************************************************
 * @fn				- SPI_ReceiveDataIT
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_STATE_BUSY_RX) {
		//1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		//2. Mark the SPI state as busy in transmission so that no other
		//	 code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_STATE_BUSY_RX;
		//3. Enable the RXEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}



/*************************************************************	TODO
 * @fn				- SPI_IRQHandling
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
	uint8_t temp1, temp2;

	//check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2){
		//handle TXE
		SPI_TXE_IT_Handle(pSPIHandle);
	}

	//check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2){
		//handle RXNE
		SPI_RXNE_IT_Handle(pSPIHandle);
	}

	//check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2){
		//handle OVR error
		SPI_OVR_IT_Handle(pSPIHandle);
	}
}



/*---- Some helper function implementations----*/
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle){
	//Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16 bit DFF
		// 2.1. Load the data to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		pSPIHandle->pTxBuffer ++;
		pSPIHandle->pTxBuffer ++;
	} else{
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen --;
		pSPIHandle->pTxBuffer ++;
	}

	if(! pSPIHandle->TxLen){
		SPI_CloseTransmission(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle){
	//Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16 bit DFF
		// 2.1. Load the data from DR to RxBuffer address
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	} else{
		// 8 bit DFF
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen --;
		pSPIHandle->pRxBuffer ++;
	}

	if(! pSPIHandle->RxLen){
		SPI_CloseReception(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void SPI_OVR_IT_Handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_STATE_BUSY_TX)
	{
		//Read both DR and SR
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_AppEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}




/*************************************************************
 * @fn				- SPI_CloseTransmission
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//Close SPI transmission and inform
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);	//Disable TX interrupts
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_STATE_READY;
}




/*************************************************************
 * @fn				- SPI_CloseReception
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//Close SPI transmission and inform
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);	//Disable RX interrupts
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_STATE_READY;
}



/*************************************************************
 * @fn				- SPI_ClearOVRFlag
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}




__weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	//This is a weak implementation . the user application may override this function.
}
