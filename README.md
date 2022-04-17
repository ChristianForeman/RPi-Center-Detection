# Configuring the Hardware

## Setting up the Sensors and Actuators STM32

### Setting up PWM for Servos
Configure TIM4 on the nucleo. Set channel 3 and 4 to be PWM generation.
Initialize the ARR value to 3999, Prescalar to 19, and initial pulse width to 320 for channel 3, and 250 for channel 4. This means PD14 is for channel 3 and PD15 is for channel 4. We use channel 3 for the top servo and channel 4 for the rotational servo.

### Setting up UART for Raspberry Pi 4 Communication
To configure UART from RPI to Nucleo. Configure USART2 to be asynchronous, baudrate to 9600, word length 8, 1 stop bit, no parity. PD6 should be default configured to receive UART commands. Also, under the NVIC Settings Tab, make sure to enable USART2 as a global interrupt so whenever UART messages are sent, it triggers an interrupt on the STM.

### Setting up I2C for HiLetgo MPU6050 Communication
To set up I2C communication, click on I2C1 under Connectivity, and enable I2C for the I2C option.

### Setting up ADC for Sharp 6Y 2Y0A700 F Infrared Radar (IR) Sensor
To set up the ADC, click on ADC1 under Analog. Set IN2 to IN2-Single-Sided, Clock Prescaler to Asynchronous Clock divided by 32, and under the dropdown menu for Rank, set Sampling Time to 640.5 Cycles. Head to the Clock Configuration tab and run the diagnostic test with which the IDE prompts the user.

### Configure printf and float Capabilities
Enable LPUART1 to asynchronous. Then set the baudrate to 115200 bits/s, word length to 8 bits w/parity, parity to "None", and stop bits to 1.
Between `/* USER CODE BEGIN 4 */` and `/* USER CODE END 4 */`, add the following code:

```
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
```
Then, between `/* USER CODE BEGIN Includes */` and `/* USER CODE END Includes */`, add the following code:

```
#include "stdio.h"
```

Right-click on the project name and click on "Properties." Expand "C/C++ Build" and click on "MCU Settings." Check the option allowing floats to be used with printf and click "Apply and Close."

See [this printf tutorial](https://docs.google.com/document/d/1wHqY2mj5vSRLN-8riEKG_4z_IsbV6kCAaLI3gJkkE8w/edit) for more details.

### Connecting the Raspberry Pi 4
Make sure to connect ground between STM and RPi. Connect pin 8 (UART0_TXD) to PD6 on the nucleo.

### Connecting the HiLetgo MPU6050
Connect GND from the sensor to any pin labelled GND on the STM32. For VCC on the sensor, connect it to the port labelled 3V3. For SCL and SDA on the sensor, connect them to the respective pins labelled on the STM32 (PB8 and PB9, respectively).

### Connecting the IR Sensor
Set up an external power supply to power the sensor with 5V. Connect both supply voltage (red) inputs and ground (black) inputs to the respectively colored inputs on the power supply. Connect both of the IR sensor ground wires to the STM32. Connect the analog data (white) wire to a breadboard. Then, add a white wire and connect it to PC1 on the STM32.

## Setting up the UI and Display STM32

The user interface section of our project operates using a separate STM32. All below steps are using a NEW STM32 than the steps above. Copy the UserInterface.c file as your main file in STM32CudeIDE after configuring your project as described below. Copy the projector.v and projector.qsf file into your Quartus Prime Lite v.18.1 project.

### Setting up I2C Communication for WayinTop 20x4 2004 LCD Display Module via I2C Serial Interface Adapter
To set up I2C communication, click on I2C1 under Connectivity, and enable I2C for the I2C option. Solder the adapter to the back of the LCD display module. Connect SCL, SDA, ground and 5V to the module. SCL connects to the PB8 pin and SDA connects to the PB9 pin on the STM32L4R5ZI-P

### Setting up the 5-directional button
The five directional button has 9 pins: UP, DOWN, LEFT, RIGHT, CENTER, SET, RESET, POWER & GROUND. Connect UP to PA1. Connect DOWN to PA0. Connect RIGHT to PA4. Connect LEFT to PA2. Connect CENTER to PA3. Connect 5V to power and ground to ground. Enable PA0-4 as GPIO inputs. Connect nothing to SET and RESET.
				
### Connect FPGA and STM32
Enable as PB6, PF1 and PF2 as GPIO outputs for the STM32. Connect PB6 to FPGA pin AB22 and PF1 to FPGA pin AB21 and PF2 to FPGA pin AC21. For the DE2 FPGA, GPIO[0] is AB22, GPIO[2] is AB21, GPIO[4] is AC21.

### Connect FPGA to Projector
Plug in a VGA to VGA cable from FPGA VGA port to projector VGA port. Use our projector.v and project.qsf files and a USB blaster to program your FPGA.

### Configure printf and float Capabilities
Follow steps from above section.

### Setting up I2C for HiLetgo MPU6050 Communication
To set up I2C communication, click on I2C4 under Connectivity, and enable I2C for the I2C option. Connect SDA pin to PF15 and SCL pin to PF14. 

### Setting up ADC for Sharp 6Y 2Y0A700 F Infrared Radar (IR) Sensor
To set up the ADC, click on ADC1 under Analog. Set IN1 to IN1-Single-Sided, Clock Prescaler to Asynchronous Clock divided by 32, and under the dropdown menu for Rank, set Sampling Time to 640.5 Cycles. Head to the Clock Configuration tab and run the diagnostic test with which the IDE prompts the user. Connect analog input voltage of IR sensor to PC0.

### Connecting the IR Sensor
Add a wire that connects PC0 on this STM to the white analog input wires on the existing breadboard.

## Sources & References

In addition to the STM32 data sheets and our EECS 373 lab documents, we referenced these resources as well. 

### Materials used:
https://docs.google.com/spreadsheets/d/1BM6z1we4ULig2K3vdXIlRVfSoMTgGnRhy5ityknNGfo/edit#gid=0

### LCD module:
https://www.youtube.com/watch?v=7mQppaEJjT4&t=220s
https://controllerstech.com/i2c-lcd-in-stm32/
https://us.beta-layout.com/download/rk/RK-10290_410.pdf
https://cdn-shop.adafruit.com/datasheets/TC2004A-01.pdf

### VGA Projection:
https://www.youtube.com/watch?v=mR-eo7a4n5Q
https://github.com/dominic-meads/Quartus-Projects/tree/main/VGA_face
https://www.fpga4fun.com/PongGame.html
https://www.intel.com/content/www/us/en/404.html?ref=https://www.intel.com/content/dam/www/programmable/us/en/portal/dsn/42/doc-us-dsnbk-42-1404062209-de2-115-user-manual.pdf
