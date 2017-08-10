/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32l1xx_hal.h"
#include <stm32l1xx_hal_uart.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
//typedef uint64_t XBeeAddress64;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
void printTime(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);
void ZBTxMakeHeader(ZBTxHeader h, char buffer[16]);
void ZBTxMakePayload(ZBTxPayload p, char* buffer);
uint8_t GenerateChecksum(char* buff,int size );
/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	initialise_monitor_handles();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_RTC_Init();
	char mymsg[13] = "To mwraki !!!";
	uint8_t headBbuffer[TX_FRAME_LENGTH + sizeof(mymsg)+1];

	ZBTxHeader h;
	h.startByte = START_BYTE;
	h.length = ((TX_FRAME_LENGTH - 4)+sizeof(mymsg)+1);
	h.frameType = 0x10;
	h.frameID = 0x01;
	h.address = 0x0000000000000000;;
	h.destinationNetworkAddr = 0xFFFE;
	h.broadcastRadius = 0x00;
	h.otpions = 0x00;




	ZBTxPayload p;
	p.payloadSize = sizeof(mymsg);
	p.payload = malloc(strlen(mymsg)+1);
	strcpy (p.payload, mymsg);

	//char payBuff[8];
	ZBTxMakeHeader(h, headBbuffer + 0);
	ZBTxMakePayload(p, headBbuffer + TX_FRAME_LENGTH);

	volatile uint8_t checksum =  GenerateChecksum(headBbuffer, sizeof(headBbuffer));

	//memcpy(headBbuffer + TX_FRAME_LENGTH + sizeof(mymsg), checksum, 1);
	headBbuffer[TX_FRAME_LENGTH + sizeof(mymsg)] = checksum;

	//printf ("headBbuffer contains: %i\n", headBbuffer);
	printf("%i", checksum);

	//char aMESSAGE[8] = "WHATEVER";

	while (1) {

		HAL_UART_Transmit(&huart1, headBbuffer, sizeof(headBbuffer), 10000);
		//printTime();
		HAL_Delay(5000);
	}
}

void printTime() {
	RTC_TimeTypeDef currentTime;
	RTC_DateTypeDef currentDate;
	HAL_RTC_WaitForSynchro(&hrtc);
	HAL_RTC_GetTime(&hrtc, &currentTime, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, FORMAT_BIN);

	char uartMsg[32] = "Time: ";
	char time[16] = { 0 };
	sprintf(time, "%d:%d:%d %d\n", currentTime.Hours, currentTime.Minutes,
			currentTime.Seconds, currentTime.SubSeconds);
	strcat(uartMsg, time);
	printf(uartMsg);
}
/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void) {

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	/**Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initialize RTC and set the Time and Date
	 */
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2) {
		sTime.Hours = 0;
		sTime.Minutes = 0;
		sTime.Seconds = 0;
		sTime.TimeFormat = RTC_HOURFORMAT_24;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = RTC_MONTH_JANUARY;
		sDate.Date = 1;
		sDate.Year = 0;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LD4_Pin | LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ZBTxMakeHeader(ZBTxHeader h, char buffer[17]) {
	uint8_t u8 = 0;
	uint16_t u16 = 0;
	uint64_t u64 = 0;

	//start byte
	u8 = h.startByte;
	memcpy(buffer + 0, &u8, 1);
	//Length bytes
	u16 = __REV16(h.length);
	memcpy(buffer + 1, &u16, 2);
	//Frame type
	u8 = h.frameType;
	memcpy(buffer + 3, &u8, 1);
	//Frame ID
	u8 = h.frameID;
	memcpy(buffer + 4, &u8, 1);
	//64-bit Destination Address
	u64 = htonll(h.address);
	memcpy(buffer + 5, &u64, 8);
	//16-bit Destination Network Address
	u16 = __REV16(h.destinationNetworkAddr);
	memcpy(buffer + 13, &u16, 2);
	//Broadcast radius
	u8 = h.broadcastRadius;
	memcpy(buffer + 15, &u8, 1);
	//Options
	u8 = h.otpions;
	memcpy(buffer + 16, &u8, 1);
}

void ZBTxMakePayload(ZBTxPayload p, char* buffer) {
	memcpy(buffer+0,p.payload,p.payloadSize);
}

uint8_t ZBTxRequest(XBeeAddress64 addr64, uint8_t *payload,
		uint8_t payloadLength) {

	char buffer[TX_FRAME_LENGTH + payloadLength + 1];

	ZBTxHeader h;
	h.startByte = START_BYTE;
	h.length = (TX_FRAME_LENGTH - 3) + payloadLength;
	h.frameType = 0x10;
	h.frameID = 0x01;
	h.address = addr64;
	h.destinationNetworkAddr = 0xFFFE;
	h.broadcastRadius = 0x00;
	h.otpions = 0x00;

	ZBTxPayload p;
	p.payloadSize = payloadLength;
	p.payload = (uint8_t*) malloc(sizeof(uint8_t) * p.payloadSize);
	p.payload = payload;

	ZBTxMakeHeader(h, buffer + 0);
	//ZBTxMakePayload(p, buffer + TX_FRAME_LENGTH, payloadLength);



	return buffer;
}

uint8_t GenerateChecksum(char* buff,int size ) {
	uint8_t value = 0;
	for(int i = 3; i < size; i++){
		volatile uint8_t temp = buff[i];
		value += buff[i];
	}
	value = 0xFF - value;
	return value + 1;
}

uint64_t htonll(uint64_t value) {
	// The answer is 42
	static const int num = 42;

	// Check the endianness
	if (*(char *) &num == 42) {
		const uint32_t high_part = __REV((uint32_t) (value >> 32));
		const uint32_t low_part = __REV((uint32_t) (value & 0xFFFFFFFFLL));

		return ((uint64_t) (low_part) << 32) | high_part;
	} else {
		return value;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
