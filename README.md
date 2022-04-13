# Configuring the Hardware

## Setting up the Sensors and Actuators STM32

### Setting up PWM for Servos
Configure TIM4 on the nucleo. Set channel 3 and 4 to be PWM generation.
Initialize the ARR value to 999, Prescalar to 79, and initial pulse width to 60. This means PD14 is for channel 3 and PD15 is for channel 4. We use channel 3 for the top servo and channel 4 for the rotational servo.

### Setting up UART for Raspberry Pi 4 Communication
To configure UART from RPI to Nucleo. Configure USART2 to be asynchronous, baudrate to 9600, word length 8, 1 stop bit, no parity. PD6 should be default configured to receive UART commands. Also, make sure to enable the UART as a global interrupt so whenever UART messages are sent, it triggers an interrupt on the STM.

### Setting up I2C for HiLetgo MPU6050 Communication
To set up I2C communication, click on I2C1 under Connectivity, and enable I2C for the I2C option.

### Setting up ADC for Ultrasonic Sensor
To set up the ADC, click on ADC1 under Analog. Set IN1 to IN1-Single-Sided, Clock Prescaler to Asynchronous Clock divided by 32, and under the dropdown menu for Rank, set Sampling Time to 640.5 Cycles. Head to the Clock Configuration tab and run the diagnostic test with which the IDE prompts the user.

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

### Connecting the Raspberri Pi 4
Make sure to connect ground between STM and RPi. Connect pin 8 (UART0_TXD) to PD6 on the nucleo.

### Connecting the HiLetgo MPU6050
Connect GND from the sensor to any pin labelled GND on the STM32. For VCC on the sensor, connect it to the port labelled 3V3. For SCL and SDA on the sensor, connect them to the respective pins labelled on the STM32 (PB8 and PB9, respectively).

### Connecting the Ultrasonic Sensor
Set up an external power supply to power the sensor with 5V. Connect both supply voltage (red) inputs and ground (black) inputs to the respectively colored inputs on the power supply. Connect both of the ultrasonic sensor ground wires to the STM32. This may require an external board such as a breadboard to facilitate the connections. Connect the analog data (white) wire to the PC0 pin on the STM32.

## Setting up the UI and Display STM32
