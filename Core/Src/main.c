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

// TODO Create an error code table
const char SENSOR_UNAVAILABLE_ERROR_STR[] = "e:1";

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
void read_data_from_sensor();
inline void ble_on();
inline void ble_off();
inline void sleep();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Create the handle for the sensor.
sht3x_handle_t handle = {
	.i2c_handle = &hi2c1,
	.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH
};


bool is_sensor_work = true;

// TODO Test
#define BUFF_SIZE 128

bool on_BLE_data_ready = false;
uint8_t BLE_buff[BUFF_SIZE];
uint16_t BLE_received_data_size = 0;
uint8_t uart_sym;

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

  // Initialise sensor (tests connection by reading the status register).
  is_sensor_work = sht3x_init(&handle);

  ble_on();

  // TODO Test IWDG period
//  const char connect_command[] = "AT+CO0FC45C3913196";
//  HAL_UART_Transmit(&huart1, connect_command, strlen(connect_command), 100);

//  HAL_IWDG_Refresh(&hiwdg); // Reset the WatchDog

  bool is_connected = false;

  // Wait for connect
  for (uint8_t i = 0; i < BLE_MAX_CONNECTION_TRIES; ++i) {
	  is_connected = HAL_GPIO_ReadPin(BLE_STATE_GPIO_Port, BLE_STATE_Pin);

	  HAL_Delay(BLE_TIMEOUT_QUANTUM_MS);

	  if (is_connected)
		  break;
  }

  HAL_UART_Receive_IT(&huart1, &uart_sym, 1);

  static const char gag_array[40] = {'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z',
									  'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z',
									  'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z',
									  'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'};

  HAL_Delay(1000);

  if (is_connected) {
	  if (is_sensor_work) {
		  // Read the data from the sensor and send it to the BLE
//		  read_data_from_sensor();

		  // TODO Find out how to fix this bug
		  // Send data to skip 40 bytes
		  HAL_UART_Transmit(&huart1, gag_array, sizeof(gag_array), 100);

		  // Send payload
//		  const char test_arr[] = "Test message that arrived after gag array\n";
//		  HAL_UART_Transmit(&huart1, test_arr, sizeof(test_arr), 100);
		  read_data_from_sensor();

		  // Then wait for answer from Host

		  // Change sleep param

		  // Go to sleep

//			  ble.is_data_from_BLE_received = false;

		  // Change timer period
	  }
	  else {
		  // Change timer period to max period

		  // Send an error
		  HAL_UART_Transmit(&huart1, (uint8_t*)SENSOR_UNAVAILABLE_ERROR_STR,
				  	  	    strlen(SENSOR_UNAVAILABLE_ERROR_STR), 100);
	  }
  }
  else {
	  // If couldn't connect go to low power checking mode - just try to connect every 26 seconds
	  // Set new prescaler and counter values for the IWDG
	  HAL_UART_Transmit(&huart1, (uint8_t*)"test",
	  				  	  	    strlen("test"), 100);
  }

  // Wait until data has been arrived from BLE
  HAL_Delay(1000);

  ble_off();
//  sleep();

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

	BLE_buff[BLE_received_data_size++] = uart_sym;
	on_BLE_data_ready = true;
	HAL_UART_Receive_IT(&huart1, &uart_sym, 1);
}

// Read temperature and humidity.
void read_data_from_sensor() {
	uint16_t temperature;
	uint16_t humidity;

	sht3x_read_temperature_and_humidity(&handle, &temperature, &humidity);

	char str[16];
	snprintf(str, 16, "d:t%.2dh%.2d", temperature, humidity);

//	if (is_heater_used) {
//		sht3x_set_header_enable(&handle, true);
//		HAL_Delay(500);
//		sht3x_set_header_enable(&handle, false);
//	}

	HAL_UART_Transmit(&huart1, str, strlen(str), 100);
}

void ble_on() {
	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_SET);
}

void ble_off() {
	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_RESET);
}

void sleep() {
//	ble_off();
	HAL_PWR_EnterSTANDBYMode();
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
