/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Jun 30, 2025
 *      Author: asans
 */


/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 5
 */

#include <string.h>
#include "stm32f407xx.h"


void SPI2_GPIO_Inits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}


void SPI2_Inits(void){
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;	//16MHz/32=500KHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);

}


void GPIO_Buttn_Init(){
	// User button
		GPIO_Handle_t GpioBtn;

		GpioBtn.pGPIOx = GPIOA;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GpioBtn);
}

void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void){
	char user_data[] = "Hello world";

	GPIO_Buttn_Init();
	SPI2_GPIO_Inits();
	SPI2_Inits();

	/*
	 * Enable SSOE so that the NSS pin is automatically controlled by hardware.
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		//Wait for the button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		//For button de-bouncing
		delay();

		//Enable the SP2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// First send length info
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);
		// Send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//Confirm SPI not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) == FLAG_SET);
		//Disable the SP2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
