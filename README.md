Setting up the STM

Configure TIM4 on the nucleo. Set channel 3 and 4 to be PWM generation.
Initialize the ARR value to 999, Prescalar to 79, and initial pulse width to 60. This means PD14 is for channel 3 and PD15 is for channel 4. We use channel 3 for the top servo and channel 4 for the rotational servo.

To configure UART from RPI to Nucleo. Configure USART2 to be asynchronous, baudrate to 9600, word length 8, 1 stop bit, no parity. PD6 should be default configured to receive UART commands. Also, make sure to enable the UART as a global interrupt so whenever UART messages are sent, it triggers an interrupt on the STM.

To configure printf: Enable LPUART1 to asynchronous, baudrate 115200, 8 bits, no parity, 1 stop bit.

RPi connections: Make sure to connect ground between STM and RPi. Connect pin 8 (UART0_TXD) to PD6 on the nucleo.

