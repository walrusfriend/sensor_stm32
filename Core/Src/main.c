/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sht3x.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define USE_CLI 0
#define BLE_TIMEOUT_QUANTUM_MS 100
#define BLE_MAX_CONNECTION_TRIES (1000 / BLE_TIMEOUT_QUANTUM_MS * 3) // 3 seconds

#define BLE_5s_delay_counter 3125
#define BLE_30s_delay_counter 4095

// TODO Create an error code table
const char SENSOR_UNAVAILABLE_ERROR_STR[] = "e:1\n";
const char BLE_NO_RESPONSE_FROM_HOST[] = "e:2\n";

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
inline void prepare_and_send_data();
inline void wait_reply_from_host();
inline void ble_on();
inline void ble_off();
inline void sleep();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Create the handle for the sensor.
sht3x_handle_t handle = {
	.i2c_handle = &hi2c1,
	.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW
//	.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH
};


bool is_sensor_work = true;

// TODO Test
#define BUFF_SIZE 128

bool on_BLE_data_ready = false;
uint8_t BLE_buff[BUFF_SIZE];
uint16_t BLE_received_data_size = 0;
uint8_t uart_sym;

extern uint32_t IWDG_prescaler;
extern uint32_t IWDG_reload;

const char request[] = "REQ\n";

const uint8_t BLE_MAX_REQUESTS = 5;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* Clear the WU FLAG */
   __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);	// Clear a flag to prevent endless MCU reseting
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   /*
    * Какую инфу нужно отправлять?
    * 	- температура
    * 	- влажность
    * 	- заряд батереи!
    * 	- ошибку, если датчик влажности не работает
    */

  ble_on();

  bool is_connected = false;

  // Wait for connect
  for (uint8_t i = 0; i < BLE_MAX_CONNECTION_TRIES; ++i) {
	  is_connected = HAL_GPIO_ReadPin(BLE_STATE_GPIO_Port, BLE_STATE_Pin);

	  if (is_connected)
		  break;

	  HAL_Delay(BLE_TIMEOUT_QUANTUM_MS);
  }

  HAL_UART_Receive_IT(&huart1, &uart_sym, 1);

  // Initialise sensor (tests connection by reading the status register).
  is_sensor_work = sht3x_init(&handle);

  if (is_connected) {
	  if (is_sensor_work) {

		  HAL_Delay(1000);
		  prepare_and_send_data();

//		  wait_reply_from_host();
		  bool exit_flag = false;
		uint8_t ble_current_request_count = 0;
		while (exit_flag == false) {
			if (on_BLE_data_ready) {
				if (BLE_buff[0] == 'S') {
					exit_flag = true;

					if (BLE_buff[2] == '1') {
						// Set freq to 5 sec
						IWDG_prescaler = IWDG_PRESCALER_64;
						IWDG_reload = BLE_5s_delay_counter;
					}
					else if (BLE_buff[2] == '2') {
						// Set freq to 26 sec
						IWDG_prescaler = IWDG_PRESCALER_256;
						IWDG_reload = BLE_30s_delay_counter;
					}
					else {
						// Handle error
					}
				}
			}

			// Timeout
			if (ble_current_request_count > BLE_MAX_REQUESTS) {
//				HAL_UART_Transmit(&huart1, (uint8_t*)BLE_NO_RESPONSE_FROM_HOST,
//								strlen(BLE_NO_RESPONSE_FROM_HOST), 100);
				break;
			}

			++ble_current_request_count;

			HAL_Delay(300);
			HAL_UART_Transmit(&huart1, request, sizeof(request), 100);
		}
	  }
	  else {
		  // Change timer period to max period
		  // Set freq to 26 sec
		  IWDG_prescaler = IWDG_PRESCALER_256;
		  IWDG_reload = BLE_30s_delay_counter;

		  // Send an error
		  HAL_UART_Transmit(&huart1, (uint8_t*)SENSOR_UNAVAILABLE_ERROR_STR,
				  	  	    strlen(SENSOR_UNAVAILABLE_ERROR_STR), 100);
	  }
  }
  else {
	  // If couldn't connect to BLE go to low power checking mode - just try to connect every 26 seconds
	  // Set freq to 26 sec
	  IWDG_prescaler = IWDG_PRESCALER_256;
	  IWDG_reload = BLE_30s_delay_counter;
  }

  // Apply new IWDG timer settings before sleep
  MX_IWDG_Init();

  sleep();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// TODO Add check to string overflow

	if (uart_sym == 'S' ||
		uart_sym == ':' ||
		uart_sym == '1' ||
		uart_sym == '2')
	{
		BLE_buff[BLE_received_data_size++] = uart_sym;
		on_BLE_data_ready = true;
	}

	HAL_UART_Receive_IT(&huart1, &uart_sym, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		uint32_t er = HAL_UART_GetError(huart);

		if (er == HAL_UART_ERROR_NONE) {
			return;
		}

		if (er & HAL_UART_ERROR_PE) {
			__HAL_UART_CLEAR_PEFLAG(huart);
		}
		if (er & HAL_UART_ERROR_NE) {
			__HAL_UART_CLEAR_NEFLAG(huart);
		}
		if (er & HAL_UART_ERROR_FE) {
			__HAL_UART_CLEAR_FEFLAG(huart);
		}
		if (er & HAL_UART_ERROR_ORE) {
			__HAL_UART_CLEAR_OREFLAG(huart);
		}
		if (er & HAL_UART_ERROR_DMA) {
			__HAL_UART_CLEAR_NEFLAG(huart);
		}
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		HAL_UART_Receive_IT(huart, (uint8_t*) &uart_sym, 1);
 	}
 }

// Read temperature and humidity.
void prepare_and_send_data() {
	uint16_t temperature;
	uint16_t humidity;
	uint8_t battery_charge;

	sht3x_read_temperature_and_humidity(&handle, &temperature, &humidity);

	// Read battery charge
	battery_charge = 88;

	const uint8_t str_size = 19;
	char str[str_size];
	snprintf(str, str_size, "d:t%.2dh%.2db%.2d\n", temperature, humidity, battery_charge);

//	if (is_heater_used) {
//		sht3x_set_header_enable(&handle, true);
//		HAL_Delay(500);
//		sht3x_set_header_enable(&handle, false);
//	}

	HAL_UART_Transmit(&huart1, str, strlen(str), 100);
}

void wait_reply_from_host() {
//	bool exit_flag = false;
//	uint8_t ble_current_request_count = 0;
//	while (exit_flag == false) {
//		if (on_BLE_data_ready) {
//			if (BLE_buff[0] == 'S') {
//				exit_flag = true;
//
//				if (BLE_buff[2] == '1') {
//					// Set freq to 5 sec
//					IWDG_prescaler = IWDG_PRESCALER_64;
//					IWDG_reload = BLE_5s_delay_counter;
//				}
//				else if (BLE_buff[2] == '2') {
//					// Set freq to 26 sec
//					IWDG_prescaler = IWDG_PRESCALER_256;
//					IWDG_reload = BLE_30s_delay_counter;
//				}
//				else {
//				    // Handle error
//				}
//			}
//		}
//
//		// Timeout
//		if (ble_current_request_count > BLE_MAX_REQUESTS) {
//			HAL_UART_Transmit(&huart1, (uint8_t*)BLE_NO_RESPONSE_FROM_HOST,
//							strlen(BLE_NO_RESPONSE_FROM_HOST), 100);
//			break;
//		}
//
//		++ble_current_request_count;
//
//		HAL_Delay(300);
//		HAL_UART_Transmit(&huart1, request, sizeof(request), 100);
//	}
}

void ble_on() {
	// Setup GPIO pin in output push-pull mode
	BLE_POWER_GPIO_Port->CRH |= GPIO_CRH_MODE11_0;

//	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void ble_off() {
	// Setup GPIO pin in analog
	BLE_POWER_GPIO_Port->CRH &= ~GPIO_CRH_MODE11;

//	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void sleep() {
	ble_off();
//	HAL_PWR_EnterSTANDBYMode();
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
