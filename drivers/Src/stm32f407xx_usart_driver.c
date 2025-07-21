/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jul 21, 2025
 *      Author: asans
 */

#include "stm32f407xx_usart_driver.h"


/*************************************************************
 * @fn				-USART_PeriClockControl
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_DI();
		}else if (pUSARTx == USART2)
		{
			USART2_PCCK_DI();
		}else if (pUSARTx == USART3)
		{
			USART3_PCCK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_DI();
		}
	}

}



/*************************************************************
 * @fn				-USART_PeripheralControl
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}

}



/*************************************************************
 * @fn				-USART_GetFlagStatus
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }

   return RESET;
}


/*************************************************************
 * @fn				-USART_ClearFlag
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);

}


/*************************************************************
 * @fn				-USART_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


/*************************************************************
 * @fn				-USART_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]  		-
 *
 * @return 			-
 *
 * @Note			- none
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

