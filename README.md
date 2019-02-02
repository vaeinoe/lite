# Overview

Hello. I am a device that measures amount of ambient light in very rough and unscientific manner. I was implemented as a quick practise work for getting an idea of STM32F303 development, and quite likely of not much serious use without further improvements.


# Howto

Aside the sources, project files for compiling and programming the device are included for System Workbench for STM32 (SW4STM32).

After programming, the device should start to communicate via USART2 / USB serial port with a baud rate of 115200 (8-N-1), returning 12 bit measurement values and sequential measurement cycle numbers in format

`{ "ambientLightLevel": 1234, "measurementCycle": 10 }`


# Techncal

 - A LDR (NSL-5152) and a 10k resistor form a voltage divider between Aref and 
 Gnd from STM NUCLEO-F303K8
 - ADC (A0) is triggered by Timer 6, and does interrupt-driven sampling of the 
 resulting voltage to a buffer at the rate of 1kHz
 - The main loop polls the end-of-buffer flag and sends an unweighted average 
 of every second of measurement data as a JSON string over serial connection

The MCU system clock & HCLK & peripherals run at 16 MHz.

STM32Cube was used to generate the initial clock tree and system & peripheral initialization code. STM32 HAL libraries are used for initializing and interfacing with the timer, ADC and U(S)ART peripherals.

Refer to the included simple schematic drawing and photo in subdirectory `schem` on how to connect the hardware.
