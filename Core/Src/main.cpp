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
#include "rtc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "usbd_cdc_if.h"

extern "C" {
#include "sht3x.h"
}

#include "CLI.h"
#include "RingBuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * TODO: Set BLE_BRK to low before go to the standby mode and set BLE_BRK to high
 * when returns from standby mode, that sequence forces BLE module to wake up
 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct Sensor {
	 uint8_t hum;	// Humidity in %
	 int8_t temp;	// Temperature in Celsius
 } sensor;

#define BUFF_SIZE 128

bool on_BLE_data_ready = false;
uint8_t BLE_buff[BUFF_SIZE];
uint16_t BLE_received_data_size = 0;

uint8_t uart_sym;
bool is_heater_used = false;

CommandLineInterpreter cli = 128;
char cli_sym;
struct ringbuf_t* input_data_buffer;
static constexpr uint16_t MAX_PRINT_SIZE = 128;
char send_buf[MAX_PRINT_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void read_data_from_sensor();

void print(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vsnprintf(send_buf, sizeof(send_buf), fmt, args);
	CDC_Transmit_FS((uint8_t*) send_buf, strlen(send_buf));
	va_end(args);
}

void cli_parser_init();
void cli_process_command();

// CLI command handlers
void help_handler();
void ble_on();
void ble_off();
void ble_send();
void ble_wakeup();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Create the handle for the sensor.
sht3x_handle_t handle = {
	.i2c_handle = &hi2c1,
	.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH
};
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
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Initialise sensor (tests connection by reading the status register).
  if (!sht3x_init(&handle)) {
	  print("SHT30 access failed\n");
  }

  input_data_buffer = ringbuf_new(64);
  cli_parser_init();

  HAL_UART_Receive_IT(&huart1, &uart_sym, 1);

  // TODO Connect to host
  const char connect_command[] = "AT+CONFC45C3913196";
  HAL_UART_Transmit(&huart1, connect_command, strlen(connect_command), 100);

  while (1)
  {
	  // Parse message
	  if (on_BLE_data_ready) {
		  /**
		   * Commands:
		   * d - read data from sensor and send it to BLE
		   * r - restart MCU
		   * h%d - enable/disable heater
		   */
//		  if (BLE_buff[0] == 'd') {
//			  read_data_from_sensor();
//			  BLE_received_data_size = 0;
//		  }
//		  else if (BLE_buff[0] == 'r') {
//			  NVIC_SystemReset();
//		  }
//		  else if (BLE_buff[0] == 'h') {
//			  if (BLE_received_data_size > 2) {
//				  if (BLE_buff[1] == '0') {
//					  is_heater_used = false;
//				  }
//				  else if (BLE_buff[1] == '1') {
//					  is_heater_used = true;
//				  }
//				  else {
//					  // TODO: Delete debug print
////					  print("Unknown heater argument\n");
//				  }
//			  }
//			  else {
//				  // TODO: Delete debug print
////				  print("Too few size\n");
//			  }
//
//			  BLE_received_data_size = 0;
//		  }
//		  else {
//			  // TODO: Delete debug print
////			  print("Unknown command: ");
//			  print(BLE_buff);
//
//			  BLE_received_data_size = 0;
//		  }
//
		  on_BLE_data_ready = false;
		  BLE_received_data_size = 0;

		  print("%s", BLE_buff);
	  }

	  cli_process_command();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	sprintf(str, "d:t%.2d,h%.2d", temperature, humidity);

	if (is_heater_used) {
		sht3x_set_header_enable(&handle, true);
		HAL_Delay(500);
		sht3x_set_header_enable(&handle, false);
	}

	HAL_UART_Transmit(&huart1, str, strlen(str), HAL_MAX_DELAY);
}

void cli_parser_init()
{
	std::vector<CLI::Command> commands;
	commands.emplace_back(
			"NOT_A_COMMAND",
			CLI::CommandRoles::SERVICE_COMMAND,
			[]()
			{
				print("Unknown command!\n");
			});

	commands.emplace_back(
			"ble_on",
			CLI::CommandRoles::USER_COMMAND,
			ble_on);

	commands.emplace_back(
			"ble_off",
			CLI::CommandRoles::USER_COMMAND,
			ble_off);

	commands.emplace_back(
			"ble_send",
			CLI::CommandRoles::USER_COMMAND,
			ble_send);

	commands.emplace_back(
			"ble_wakeup",
			CLI::CommandRoles::USER_COMMAND,
			ble_wakeup);

	cli.add_commands(commands);
}

void cli_process_command()
{
	// Check queue for new syms
	if (ringbuf_is_empty (input_data_buffer))
		return;

	// Add the sym and try to process a command
	ringbuf_memcpy_from(&cli_sym, input_data_buffer, 1);

	CLI::StatusCode status = cli.process(cli_sym);

	if (status == CLI::StatusCode::BUSY) {
		return;
	}
	else if (status == CLI::StatusCode::MAX_SIZE_REACHED) {
		print("Max size reached!\n");
	}
	else if (status == CLI::StatusCode::UNKNOWN_COMMAND) {
		print("Unknown command!\n");
	}

	cli.args.clear();
}

void ble_on() {
	print("BLE power ON!\n");
//	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_SET);
}

void ble_off() {
	print("BLE power OFF!\n");
//	HAL_GPIO_WritePin(BLE_POWER_GPIO_Port, BLE_POWER_Pin, GPIO_PIN_RESET);
}

void ble_send() {
	print("Send data to BLE through UART: %s\n", cli.args[0].c_str());
	HAL_UART_Transmit(&huart1, cli.args[0].c_str(), cli.args[0].size(), 100);
}

void ble_wakeup() {
	print("Wakeup handler\n");
	HAL_GPIO_WritePin(BLE_BRK_GPIO_Port, BLE_BRK_Pin, GPIO_PIN_RESET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(BLE_BRK_GPIO_Port, BLE_BRK_Pin, GPIO_PIN_SET);
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
