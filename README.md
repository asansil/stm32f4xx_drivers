# stm32f4xx_drivers

This repository contains a from-scratch implementation of drivers for peripherals such as GPIO, I2C, SPI, and USART, developed in a bare-metal environment. The code was written as part of a personal practice and learning exercise focused on low-level programming for microcontrollers.

Development and testing have been carried out using the **STM32F407G-DISC1** development board.

---

## 📂 Example Projects

A set of example files is included in the `Src` directory to verify and demonstrate the functionality of the implemented drivers:

- **001led_toggle.c** – Toggles the onboard LED to confirm basic GPIO output control.
- **002led_button.c** – Lights up the onboard LED when the onboard button is pressed, testing both GPIO input and output.
- **005button_interrupt.c** – Similar to the previous example, but using interrupts to handle the button press.
- **006spi_tx_testing.c** – Verifies that the board, acting as SPI master, can send data over SPI.
- **007spi_txonly_arduino.c** – Sends data over SPI to an Arduino acting as slave, testing cross-device communication.
- **008spi_cmd_handling.c** – Implements a command-based communication protocol over SPI, where the STM32 (master) sends instructions to the Arduino (slave).
- **009spi_message_rcv_it.c** – Adds SPI interrupt handling for non-blocking message transmission and reception. It tests receiving messages typed on a serial monitor connected to the Arduino and relaying them back to the STM32 via SPI.
- **010i2c_master_tx_testing.c** – Sends a character string over I2C to an Arduino UNO, verifying the master transmit process.
- **011i2c_master_rx_testing.c** – Receives a character string over I2C from an Arduino UNO, testing the master receive process.
- **012i2c_master_rx_testingIT.c** – Same as the previous example, but using interrupts for non-blocking reception.
- **013i2c_slave_tx_string.c** – The STM32 acts as an I2C slave and responds by sending a character string using interrupts.
- **014i2c_slave_tx_long_string.c** – Same as the previous example but allows sending more than 32 bytes of data.
- **015uart_tx.c** – Sends a character string over UART to an Arduino, testing UART transmission.
- **016uart_case.c** – Receives a string over UART, toggles the case of each character, and sends the result back using interrupts.

> ⚠️ **Note:** To test a specific example, **all other example files must be excluded from the build**. Only one example source file should be compiled at a time.
