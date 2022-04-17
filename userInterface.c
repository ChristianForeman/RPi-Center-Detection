/*USER CODE BEGIN Header */
/**
 ******************************************************************************
 *@file           : main.c
 *@brief          : Main program body
 ******************************************************************************
 *@attention
 *
 *Copyright (c) 2022 STMicroelectronics.
 *All rights reserved.
 *
 *This software is licensed under terms that can be found in the LICENSE file
 *in the root directory of this software component.
 *If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/*USER CODE END Header */
/*Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/*Private includes ----------------------------------------------------------*/
/*USER CODE BEGIN Includes */
#include "stdio.h"

/*USER CODE END Includes */

/*Private typedef -----------------------------------------------------------*/
/*USER CODE BEGIN PTD */

/*USER CODE END PTD */

/*Private define ------------------------------------------------------------*/
/*USER CODE BEGIN PD */
/*USER CODE END PD */

/*Private macro -------------------------------------------------------------*/
/*USER CODE BEGIN PM */
//int interruptPin = 0;
/*USER CODE END PM */

/*Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef hlpuart1;

/*USER CODE BEGIN PV */
#define ADDR 0x4E	//from PCF8574T data sheet / I2C scanning
int currentPosition = 0;

void lcd_send_cmd(char sent)
{
	uint8_t arr[4];
	char sent_upper = (sent & 0xf0);	//upper 2 bits
	char sent_lower = ((sent << 4) &0xf0);	//lower 2 bits
	arr[0] = sent_upper | 0x0C;	//en=1, rs=0 WHY??
	arr[1] = sent_upper | 0x08;	//en=0, rs=0 WHY??
	arr[2] = sent_lower | 0x0C;	//en=1, rs=0 WHY??
	arr[3] = sent_lower | 0x08;	//en=0, rs=0 WHY??
	HAL_I2C_Master_Transmit(&hi2c1, ADDR, (uint8_t*) arr, 4, 100);
}

void lcd_send_char(char sent)
{
	uint8_t arr[4];
	char sent_upper = (sent & 0xf0);	//upper 2 bits
	char sent_lower = ((sent << 4) &0xf0);	//lower 2 bits
	arr[0] = sent_upper | 0x0D;	//en=1, rs=0 WHY??
	arr[1] = sent_upper | 0x09;	//en=0, rs=0 WHY??
	arr[2] = sent_lower | 0x0D;	//en=1, rs=0 WHY??
	arr[3] = sent_lower | 0x09;	//en=0, rs=0 WHY??
	HAL_I2C_Master_Transmit(&hi2c1, ADDR, (uint8_t*) arr, 4, 100);
}

void lcd_clear(void)
{
	lcd_send_cmd(0x01);	//should turn off display
	lcd_send_cmd(0x02);	//should return to home
}

void lcd_init(void)
{
	// 4 bit initialisation
	HAL_Delay(70);	// wait for >40ms
	lcd_send_cmd(0x30);
	HAL_Delay(10);	// wait for >4.1ms
	lcd_send_cmd(0x30);
	HAL_Delay(5);	// wait for >100us
	lcd_send_cmd(0x30);
	HAL_Delay(100);
	lcd_send_cmd(0x20);	// 4bit mode
	HAL_Delay(100);

	// dislay initialisation
	lcd_send_cmd(0x28);	// Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(100);
	lcd_send_cmd(0x08);	//Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(100);
	lcd_send_cmd(0x01);	// clear display
	HAL_Delay(100);
	HAL_Delay(100);
	lcd_send_cmd(0x06);	//Entry mode set --> I/D = 1 (increment cursor) &S = 0 (no shift)
	HAL_Delay(100);
	lcd_send_cmd(0x0F);	//Display on/off control --> D = 1, C and B = 1. (Cursor and blink ON)
}

void lcd_print(char *msg)
{
	while (*msg != '\0')
	{
		lcd_send_char(*msg);
		++msg;
	}
}

void lcd_move_cursor(char direction)
{
	if (direction == 'L')
	{
		if (currentPosition > 0 && currentPosition < 0x13)
		{
			currentPosition = currentPosition - 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition > 40 && currentPosition < 0x53)
		{
			currentPosition = currentPosition - 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition > 14 && currentPosition < 0x27)
		{
			currentPosition = currentPosition - 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition > 54 && currentPosition < 0x67)
		{
			currentPosition = currentPosition - 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
	}
	else if (direction == 'R')
	{
		if (currentPosition >= 0 && currentPosition < 0x13)
		{
			currentPosition = currentPosition + 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition >= 40 && currentPosition < 0x53)
		{
			currentPosition = currentPosition + 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition >= 14 && currentPosition < 0x27)
		{
			currentPosition = currentPosition + 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition >= 54 && currentPosition < 0x67)
		{
			currentPosition = currentPosition + 1;
			lcd_send_cmd(0x80 | currentPosition);
		}
	}
	else if (direction == 'D')
	{
		if (currentPosition <= 0x13)
		{
			currentPosition += 0x40;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition <= 0x27)
		{
			currentPosition += 0x40;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition <= 0x53)
		{
			currentPosition -= 44;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition <= 0x67)
		{
			//do nothing
		}
	}
	else if (direction == 'U')
	{
		if (currentPosition <= 0x13)
		{
			//do nothing
		}
		else if (currentPosition <= 0x27)
		{
			currentPosition += 44;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition <= 0x53)
		{
			currentPosition -= 0x40;
			lcd_send_cmd(0x80 | currentPosition);
		}
		else if (currentPosition <= 0x67)
		{
			currentPosition -= 0x40;
			lcd_send_cmd(0x80 | currentPosition);

		}
	}
}

/*USER CODE END PV */

/*Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C4_Init(void);
/*USER CODE BEGIN PFP */

/*USER CODE END PFP */

/*Private user code ---------------------------------------------------------*/
/*USER CODE BEGIN 0 */

int16_t convert16Bit(uint8_t readData[])
{
	return (int16_t)(((uint16_t)((uint16_t) readData[0]) << 8) + (uint16_t) readData[1]);
}

/*USER CODE END 0 */

/**
 *@brief  The application entry point.
 *@retval int
 */
int main(void)
{
	/*USER CODE BEGIN 1 */

	/*USER CODE END 1 */

	/*MCU Configuration--------------------------------------------------------*/

	/*Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/*USER CODE BEGIN Init */

	/*USER CODE END Init */

	/*Configure the system clock */
	SystemClock_Config();

	/*USER CODE BEGIN SysInit */

	/*USER CODE END SysInit */

	/*Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_LPUART1_UART_Init();
	MX_I2C1_Init();
	MX_ADC1_Init();
	MX_I2C4_Init();
	/*USER CODE BEGIN 2 */

	//======================= SCREEN INIT =======================//
	lcd_init();	//initialize first
	HAL_Delay(4000);	//wait
	lcd_print("Option 1: HorizontalOption 3: Vertical  Option 2: Smiley");
	lcd_send_cmd(0x02);

	//======================= POLLING VARS =======================//
	volatile int prevPressedCenter = 0;
	volatile int prevPressedRight = 0;
	volatile int prevPressedLeft = 0;
	volatile int prevPressedUp = 0;
	volatile int prevPressedDown = 0;
	volatile int nowPressedCenter = 0;
	volatile int nowPressedRight = 0;
	volatile int nowPressedLeft = 0;
	volatile int nowPressedUp = 0;
	volatile int nowPressedDown = 0;

	/*USER CODE END 2 */

	/*Infinite loop */
	/*USER CODE BEGIN WHILE */

	//======================= SENSOR INIT =======================//
	uint8_t reader[14];
	for (uint8_t setter = 0; setter < 14; ++setter){
		reader[setter] = setter * 2;
	}

	uint8_t whoAmIChecker = 0;
	uint8_t wakeUpVal = 0;	// Same for gyro and accel config regs
	uint8_t divVal = 7;
	uint8_t ret;
	uint8_t DEVICE_ADDR = 0xD0;	// Address is 0x68, bit-shifted left for r/w bit
	uint8_t WHO_AM_I = 0x75;
	uint8_t PWR_M_1 = 0x6B;
	uint8_t SAMPLE_DIV = 0x19;
	uint8_t G_CONF = 0x1B;
	uint8_t A_CONF = 0x1C;
	float runningAverage = 0;

	ret = HAL_I2C_Mem_Read(&hi2c4, DEVICE_ADDR, WHO_AM_I, 1, &whoAmIChecker, 1, 1000);
	if (ret != HAL_OK) printf("Reading WHO_AM_I failed!\n");
	else printf("Received address in WHO_AM_I: 0x%x\n", whoAmIChecker);
	ret = HAL_I2C_Mem_Write(&hi2c4, DEVICE_ADDR, PWR_M_1, 1, &wakeUpVal, 1, 1000);
	if (ret != HAL_OK) printf("Waking up IMU failed!\n");
	else printf("Woke up the IMU!\n");
	ret = HAL_I2C_Mem_Write(&hi2c4, DEVICE_ADDR, SAMPLE_DIV, 1, &divVal, 1, 1000);
	if (ret != HAL_OK) printf("Setting the sample rate failed!\n");
	else printf("Sample rate set to 1kHz!\n");
	ret = HAL_I2C_Mem_Write(&hi2c4, DEVICE_ADDR, G_CONF, 1, &wakeUpVal, 1, 1000);
	if (ret != HAL_OK) printf("Gyroscope configuration failed!\n");
	else printf("Gyroscope configuration succeeded!\n");
	ret = HAL_I2C_Mem_Write(&hi2c4, DEVICE_ADDR, A_CONF, 1, &wakeUpVal, 1, 1000);
	if (ret != HAL_OK) printf("Accelerometer configuration failed!\n");
	else printf("Accelerometer configuration succeeded!\n");

	//======================= SENSOR VARS =======================//
	uint32_t ADC_VAL = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	ADC_VAL = HAL_ADC_GetValue(&hadc1);
	int counter = 0;

	while (1)
	{
		//======================= IR SENSOR BEGIN =======================//
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
		ADC_VAL = HAL_ADC_GetValue(&hadc1);
		float f1 = (ADC_VAL / 4096.0);
		float voltage = f1 * 3.3;
		float dist = 0.0;
		if (voltage > 1.388)
		{
			// Less than 2 meters
			dist = (voltage - 4.4712) / -1.5608;
		}
		else
		{
			dist = (voltage - 2.1445) / -0.3829;
		}

		//======================= IR SENSOR END =========================//

		//======================= IMU SENSOR BEGIN ======================//
		float angle = 0.0;
		ret = HAL_I2C_Mem_Read(&hi2c4, DEVICE_ADDR, 0x3B, 1, &reader[0], 14, 1000);
		if (ret != HAL_OK) printf("Error receiving data!\n");
		else
		{
			float accelY = convert16Bit(&reader[2]) / 16384.0;
			float accelZ = convert16Bit(&reader[4]) / 16384.0;
			// For accuracy divide accelZ value by 0.965
			angle = accelZ / 0.965 > 1.0 ? 0.0 : acosf(accelZ / 0.965) *180.0 / M_PI;
			if (accelY > 0) angle *= -1;
			//printf("Current Angle: %f degrees\n\n", angle);
			float gyroX = convert16Bit(&reader[8])/131.0;
			if (gyroX > 5 || gyroX < -5) {
				runningAverage = angle;
			}
			else {
				runningAverage = runningAverage*0.875 + 0.125*angle;
				angle = runningAverage;
			}
			printf("Current Angle: %f degrees\n\n", angle);
			//float gyroY = convert16Bit(&reader[10])/131.0;
			//float gyroZ = convert16Bit(&reader[12])/131.0;
			//printf("Received Gyro Values: (%3.3f, %3.3f, %3.3f)\n\n", gyroX, gyroY, gyroZ);
			//printf("Current ZValue: %f degrees\n\n", accelZ);
		}

		//======================= IMU SENSOR END ========================//

		//======================= POLLING BEGIN =======================//
		//1) UPDATE PREV PRESSED
		prevPressedCenter = nowPressedCenter;
		prevPressedRight = nowPressedRight;
		prevPressedLeft = nowPressedLeft;
		prevPressedUp = nowPressedUp;
		prevPressedDown = nowPressedDown;

		nowPressedDown = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		nowPressedUp = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		nowPressedLeft = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		nowPressedCenter = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		nowPressedRight = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

		if (nowPressedCenter == 1 && prevPressedCenter == 0)
		{
			printf("%i\n", currentPosition);
			printf("CENTER PRESSED\n");
			if (currentPosition >= 0 && currentPosition <= 19)
			{
				//horizontal is all 1s
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
			}
			else if (currentPosition >= 64 && currentPosition <= 79)
			{
				//SMILEY 001
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
			}
			else if (currentPosition >= 20 && currentPosition <= 37)
			{
				//VERTICAL 000
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
			}
		}
		else
		{
			if (nowPressedUp == 1 && prevPressedUp == 0)
			{
				lcd_move_cursor('U');
			}
			else if (nowPressedDown == 1 && prevPressedDown == 0)
			{
				lcd_move_cursor('D');
			}

			if (nowPressedLeft == 1 && prevPressedLeft == 0)
			{
				lcd_move_cursor('L');
			}
			else if (nowPressedRight == 1 && prevPressedRight == 0)
			{
				lcd_move_cursor('R');
			}
		}

		//======================= PRINT STATS AND UPDATE COUNTER =======================//
		if (counter == 100)
		{
			counter = 0;
		}
		else
		{
			counter++;
		}

		if (counter == 99)
		{
			lcd_send_cmd(0x80 | 0x54);
			char *str1 = "Dis:";
			char *str3 = "Deg:";
			char str2[20];
			snprintf(str2, 20, "%s %1.2f %s %2.1f", str1, dist, str3, angle);
			lcd_print(str2);
			lcd_send_cmd(0x80 | currentPosition);	//back to old pos
		}


		/*USER CODE END WHILE */

		/*USER CODE BEGIN 3 */

	}

	/*USER CODE END 3 */
}

/**
 *@brief System Clock Configuration
 *@retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the RCC Oscillators according to the specified parameters
	 *in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 *@brief ADC1 Initialization Function
 *@param None
 *@retval None
 */
static void MX_ADC1_Init(void)
{
	/*USER CODE BEGIN ADC1_Init 0 */

	/*USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/*USER CODE BEGIN ADC1_Init 1 */

	/*USER CODE END ADC1_Init 1 */
	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/*USER CODE BEGIN ADC1_Init 2 */

	/*USER CODE END ADC1_Init 2 */

}

/**
 *@brief I2C1 Initialization Function
 *@param None
 *@retval None
 */
static void MX_I2C1_Init(void)
{
	/*USER CODE BEGIN I2C1_Init 0 */

	/*USER CODE END I2C1_Init 0 */

	/*USER CODE BEGIN I2C1_Init 1 */

	/*USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00000E14;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}

	/*USER CODE BEGIN I2C1_Init 2 */

	/*USER CODE END I2C1_Init 2 */

}

/**
 *@brief I2C4 Initialization Function
 *@param None
 *@retval None
 */
static void MX_I2C4_Init(void)
{
	/*USER CODE BEGIN I2C4_Init 0 */

	/*USER CODE END I2C4_Init 0 */

	/*USER CODE BEGIN I2C4_Init 1 */

	/*USER CODE END I2C4_Init 1 */
	hi2c4.Instance = I2C4;
	hi2c4.Init.Timing = 0x00000E14;
	hi2c4.Init.OwnAddress1 = 0;
	hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c4.Init.OwnAddress2 = 0;
	hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c4) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
	{
		Error_Handler();
	}

	/*USER CODE BEGIN I2C4_Init 2 */

	/*USER CODE END I2C4_Init 2 */

}

/**
 *@brief LPUART1 Initialization Function
 *@param None
 *@retval None
 */
static void MX_LPUART1_UART_Init(void)
{
	/*USER CODE BEGIN LPUART1_Init 0 */

	/*USER CODE END LPUART1_Init 0 */

	/*USER CODE BEGIN LPUART1_Init 1 */

	/*USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}

	/*USER CODE BEGIN LPUART1_Init 2 */

	/*USER CODE END LPUART1_Init 2 */

	}

	/**
	  * @brief GPIO Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_GPIO_Init(void)
	{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  HAL_PWREx_EnableVddIO2();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	  /*Configure GPIO pins : PE2 PE3 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pins : PF1 PF2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /*Configure GPIO pin : PF7 */
	  GPIO_InitStruct.Pin = GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA0 PA1 PA2 PA3
	                           PA4 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
	                          |GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA5 PA6 PA7 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : PE7 PE8 PE9 PE10
	                           PE11 PE12 PE13 */
	  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
	                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pins : PE14 PE15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB10 */
	  GPIO_InitStruct.Pin = GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : PB12 PB13 PB15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB14 */
	  GPIO_InitStruct.Pin = GPIO_PIN_14;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : PD8 PD9 */
	  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : PD14 PD15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pin : PC6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PC7 */
	  GPIO_InitStruct.Pin = GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA8 PA10 */
	  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA9 */
	  GPIO_InitStruct.Pin = GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PC11 PC12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PD0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pin : PD2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : PB3 PB4 PB5 */
	  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PE0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	}

	/* USER CODE BEGIN 4 */

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


	/* USER CODE END 4 */

	/**
	  * @brief  This function is executed in case of error occurrence.
	  * @retval None
	  */
	void Error_Handler(void)
	{
	  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1)
	  {
	  }
	  /* USER CODE END Error_Handler_Debug */
	}

	#ifdef  USE_FULL_ASSERT
	/**
	  * @brief  Reports the name of the source file and the source line number
	  *         where the assert_param error has occurred.
	  * @param  file: pointer to the source file name
	  * @param  line: assert_param error line source number
	  * @retval None
	  */
	void assert_failed(uint8_t *file, uint32_t line)
	{
	  /* USER CODE BEGIN 6 */
	  /* User can add his own implementation to report the file name and line number,
	     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	  /* USER CODE END 6 */
	}
	#endif /* USE_FULL_ASSERT */
