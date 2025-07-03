/*
 * 008spi_cmd_handling.c
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

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();

//Command codes
#define CMD_LED_CTRL			0X50
#define CMD_SENSOR_READ			0X51
#define CMD_LED_READ			0X52
#define CMD_COMMAND_PRINT		0X53
#define CMD_ID_READ				0X54

#define LED_ON		1
#define LED_OFF		0

//Arduino analog pins
#define ARD_ANALOG_PIN0		0
#define ARD_ANALOG_PIN1		1
#define ARD_ANALOG_PIN2		2
#define ARD_ANALOG_PIN3		3
#define ARD_ANALOG_PIN4		4

//Arduino led
#define ARD_LED_PIN			9


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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
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

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	printf("ACK response: 0x%02X\n", ackbyte);
	if(ackbyte == 0xF5) return 1; //ack
	else return 0;	//nack
}



int main(void){
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	initialise_monitor_handles();
	printf("Application running\n");

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
		printf("CMD_LED_CTRL\n");

		//Enable the SP2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL	<pin no(1)>		<value(1)>
		uint8_t cmdCode = CMD_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &cmdCode, sizeof(cmdCode));
		//read dummy data to clear off RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send 1byte of dummy bits to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, sizeof(dummy_write));

		//Read ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte)){
			printf("Command accepted\n");
			//send arguments
			args[0] = ARD_LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2, args, 2);
			// dummy read
			SPI_ReceiveData(SPI2,args,2);
			//printf("COMMAND_LED_CTRL Executed\n");
		}


		// 2. CMD_SENSOR_READ	<analog pin number(1)>
		//Wait for the button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		//For button de-bouncing
		delay();
		printf("CMD_SENSOR_READ\n");

		//send command
		cmdCode = CMD_SENSOR_READ;
		SPI_SendData(SPI2, &cmdCode, sizeof(cmdCode));
		//read dummy data to clear off RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send 1byte of dummy bits to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, sizeof(dummy_write));
		//Read ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte)){
			printf("Command accepted\n");
			//send arguments
			args[0] = ARD_ANALOG_PIN0;
			SPI_SendData(SPI2, args, 2);
			// dummy read
			SPI_ReceiveData(SPI2,args,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) to fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("Sensor value %d\n",analog_read);

		}



		//Confirm SPI not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) == FLAG_SET);
		//Disable the SP2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI disabled\n");
	}

	return 0;
}

