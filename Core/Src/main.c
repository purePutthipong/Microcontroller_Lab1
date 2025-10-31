/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct _ButMtx_Struct {
	GPIO_TypeDef *Port;
	uint16_t Pin;
};

struct _ButMtx_Struct BMX_L[4] = { { GPIOA, GPIO_PIN_9 }, { GPIOC, GPIO_PIN_7 },
		{ GPIOB, GPIO_PIN_6 }, { GPIOA, GPIO_PIN_7 } };

struct _ButMtx_Struct BMX_R[3] = { { GPIOB, GPIO_PIN_5 }, { GPIOB, GPIO_PIN_4 },
		{ GPIOB, GPIO_PIN_10 } };

uint16_t ButtonState = 0;
uint16_t input = 0;
int inputCount = 0;
int MAX_INPUT = 11;
int state_input = 0;
int state_check = 0;
int correct_count = 0;
int wrong_count = 0;
uint16_t States_input[4] = { 0, 0, 0, 0 };
uint16_t pressedValues[11];
uint16_t password[11] = { 6, 7, 3, 4, 0, 5, 0, 0, 0, 3, 2 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void ButtonMatrixRead();
int getButtonInde();
int getButtonIndex(uint16_t ButtonState);
void getOutput(int value);
void getLed(uint16_t States_input[4]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  /* USER CODE BEGIN 2 */
	getOutput(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		static uint32_t BTMX_TimeStamp = 0;
		if (HAL_GetTick() > BTMX_TimeStamp) {
			BTMX_TimeStamp = HAL_GetTick() + 25;
			ButtonMatrixRead();
			input = getButtonIndex(ButtonState);
		}
		if (input == 12) {
			state_input = 0;
		}
		if (input != 12 && input != 10 && state_input == 0) {
//			getOutput(input);
			if (inputCount < MAX_INPUT && input != 11) {
				pressedValues[inputCount] = input;
				inputCount++;
				state_check = 0;
			} else {
				if (input == 11 && !state_check && inputCount >= MAX_INPUT) {
					state_check = 1;
					for (int i = 0; i < MAX_INPUT; i++) {
						if (pressedValues[i] == password[i]) {
							correct_count += 1;
							if (correct_count == 11) {
								wrong_count = 4;
								getOutput(wrong_count);
							}
						} else {
							wrong_count += 1;
							correct_count = 0;
							inputCount = 0;
							for (int i = 0; i < MAX_INPUT; i++) {
								pressedValues[i] = 0;
							}
							getOutput(wrong_count);
							break;
						}
					}
				}
			}
			state_input = 1;
		}

	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13 || GPIO_Pin == GPIO_PIN_8) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		for (int i = 0; i < 13; i++) {
			pressedValues[i] = 0;
		}
		inputCount = 0;
		state_input = 0;
		state_check = 0;
		correct_count = 0;
		wrong_count = 0;
//		getOutput(wrong_count);
//		getLed(States_input);

	}
}

void ButtonMatrixRead() {
	static uint8_t X = 0;
	for (int i = 0; i < 4; i++) {
		if (HAL_GPIO_ReadPin(BMX_L[i].Port, BMX_L[i].Pin) == GPIO_PIN_RESET) {
			ButtonState |= 1 << (i + (X * 4));
		} else {
			ButtonState &= ~(1 << (i + (X * 4)));
		}
	}

	HAL_GPIO_WritePin(BMX_R[X].Port, BMX_R[X].Pin, GPIO_PIN_SET);

	uint8_t nextX = (X + 1) % 3;
	HAL_GPIO_WritePin(BMX_R[nextX].Port, BMX_R[nextX].Pin, GPIO_PIN_RESET);
	X = nextX;
}

int getButtonIndex(uint16_t value) {
	switch (value) {
	case 8:
		return 0;
	case 4:
		return 1;
	case 64:
		return 2;
	case 1024:
		return 3;
	case 2:
		return 4;
	case 32:
		return 5;
	case 512:
		return 6;
	case 1:
		return 7;
	case 16:
		return 8;
	case 256:
		return 9;
	case 128:
		return 10;
	case 2048:
		return 11;
	default:
		return 12;
	}
}
void getOutput(int value) {
	switch (value) {
	case 0:
		States_input[0] = 1;
		States_input[1] = 1;
		States_input[2] = 1;
		States_input[3] = 1;
		break;
	case 1:
		States_input[0] = 1;
		States_input[1] = 0;
		States_input[2] = 1;
		States_input[3] = 1;
		break;
	case 2:
		States_input[0] = 1;
		States_input[1] = 0;
		States_input[2] = 0;
		States_input[3] = 1;
		break;
	case 3:
		States_input[0] = 1;
		States_input[1] = 0;
		States_input[2] = 0;
		States_input[3] = 0;
		getLed(States_input);
		while (1) {
		}
		break;
	case 4:
		States_input[0] = 0;
		States_input[1] = 1;
		States_input[2] = 1;
		States_input[3] = 1;
		break;
	default:
		break;
	}
	getLed(States_input);
}

void getLed(uint16_t States_input[4]) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	HAL_Delay(100);
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, States_input[i]);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		HAL_Delay(100);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
	HAL_Delay(100);
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
