/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct {
	uint8_t command;
	uint16_t P;
	uint16_t I;
	uint16_t D;
} pid_transmit_struct;

struct {
	uint8_t command;
	int16_t offsetR;
	int16_t offsetP;
} offset_transmit_struct;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t last_time = 0;

uint8_t buffer[64];

uint8_t uart_buff[32] = { 0, };
uint8_t buff_counter = 0;
uint8_t lora_rec_string[32] = { 0, };
uint8_t lora_tx_buff[32] = { 0, };

char trans_str[60] = { 0, };

uint8_t length_of_packet = 0;
_Bool COMdataAvailable = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		/*if(uart_buff[buff_counter] != "$" && buff_counter + 1 <= sizeof(uart_buff)){
		 buff_counter++;
		 HAL_UART_Receive_IT(&huart1, uart_buff[buff_counter], 1);
		 }
		 else{
		 memcpy(lora_rec_string, uart_buff, buff_counter + 1);
		 CDC_Transmit_FS(lora_rec_string, buff_counter + 1);
		 buff_counter = 0;
		 }*/

		CDC_Transmit_FS(uart_buff, 32);

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		HAL_UART_Receive_IT(&huart1, uart_buff, 32);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (COMdataAvailable) {
			COMdataAvailable = 0;

			if (!strncmp(buffer, "PID_R", 5)) {
				pid_transmit_struct.command = 2;
				int P, I, D;
				sscanf(buffer, "PID_R %d %d %d", &P, &I, &D);

				pid_transmit_struct.P = (uint16_t) P;
				pid_transmit_struct.I = (uint16_t) I;
				pid_transmit_struct.D = (uint16_t) D;

				memcpy(lora_tx_buff, &pid_transmit_struct,
						sizeof(pid_transmit_struct));

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
						sizeof(lora_tx_buff));

			} else if (!strncmp(buffer, "PID_P", 5)) {
				pid_transmit_struct.command = 3;
				int P, I, D;
				sscanf(buffer, "PID_P %d %d %d", &P, &I, &D);

				pid_transmit_struct.P = (uint16_t) P;
				pid_transmit_struct.I = (uint16_t) I;
				pid_transmit_struct.D = (uint16_t) D;

				memcpy(lora_tx_buff, &pid_transmit_struct,
						sizeof(pid_transmit_struct));

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
						sizeof(lora_tx_buff));

			} else if (!strncmp(buffer, "OFFSET", 6)) {
				offset_transmit_struct.command = 5;

				int Pitch, Roll;
				sscanf(buffer, "OFFSET %d %d", &Pitch, &Roll);

				offset_transmit_struct.offsetP = Pitch;
				offset_transmit_struct.offsetR = Roll;

				memcpy(lora_tx_buff, &offset_transmit_struct,
						sizeof(offset_transmit_struct));
				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
						sizeof(lora_tx_buff));

			} else if (!strncmp(buffer, "PID_ALT", 7)) {

				pid_transmit_struct.command = 6;
				int P, I, D;
				sscanf(buffer, "PID_ALT %d %d %d", &P, &I, &D);

				pid_transmit_struct.P = (uint16_t) P;
				pid_transmit_struct.I = (uint16_t) I;
				pid_transmit_struct.D = (uint16_t) D;

				memcpy(lora_tx_buff, &pid_transmit_struct,
						sizeof(pid_transmit_struct));

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
						sizeof(lora_tx_buff));

			} else if(!strncmp(buffer, "GPS_POS", 7)){

				lora_tx_buff[0] = 7;

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
										sizeof(lora_tx_buff));
			}
			else if(!strncmp(buffer, "GPS_SPEED", 9)){
				lora_tx_buff[0] = 8;

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
										sizeof(lora_tx_buff));
			}
			else if(!strncmp(buffer, "GET_COMPASS", 11)){
				lora_tx_buff[0] = 9;

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
										sizeof(lora_tx_buff));
			}
			else if(!strncmp(buffer, "GET_IMU", 7)){
				lora_tx_buff[0] = 10;

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
										sizeof(lora_tx_buff));
			}
			else if(!strncmp(buffer, "GYRO_RAW", 8)){
				lora_tx_buff[0] = 11;

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
										sizeof(lora_tx_buff));
			}
			else if(!strncmp(buffer, "ACCEL_RAW", 9)){
				lora_tx_buff[0] = 12;

				HAL_UART_Transmit_IT(&huart1, lora_tx_buff,
										sizeof(lora_tx_buff));
						}

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB11 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

