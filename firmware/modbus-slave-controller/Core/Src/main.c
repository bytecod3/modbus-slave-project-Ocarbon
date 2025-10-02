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
  *
  * THIS IS THE SLAVE DEVICE
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <modbus_rtu.h>
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "custom_types.h"
#include "string.h"
#include "stdio.h"
#include "defines.h"
#include "relays.h" // todo: create a task to control the relays
#include "modbus_rtu.h"
#include "modbus_tcp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** modbus variables */
int modbus_rx_len = 0; // actual received modbus message length
uint8_t modbus_rx_message[MODBUS_RTU_MAX_SIZE] = {0}; // to hold the received modbus message
char uart_buff[10] = {0}; // for debug

#define COIL_COUNT 100
uint8_t COIL[COIL_COUNT] = {0}; // this is for debugging - does not follow MODBUS protocol of 1 bit per coil

uint8_t coils[(COIL_COUNT + 7) / 8] = {0x4D, 0x0D}; // MODBUS spec coils -> rounds off to the nearest byte (ceiling method)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId x_task_receive_modbus_RTU_handle;
osThreadId x_task_receive_modbus_TCP_handle;
osThreadId x_task_relay_control_handle;
osThreadId x_task_ethernet_control_handle;
osThreadId x_task_get_device_diagnostics_handle;
osThreadId x_task_print_to_terminal_handle;
osThreadId x_task_clean_modbus_RTU_queue_handle;


//============ DATA QUEUE HANDLES  ============
QueueHandle_t modbus_RTU_queue_handle;

// =========== SEMPAHORE HANDLES ===============
SemaphoreHandle_t x_relay_control_semaphore;

// ============ EVENT GROUPS =================
#define RECEIVE_MODBUS_BIT		(1 << 0UL)		// bit set if MODBUS RTU received from MODBUS_RTU queue
#define PRINT_TO_TERMINAL_BIT	(1 << 1UL)		// bit set if Print to terminal task received from MODBUS_RTU queque

EventGroupHandle_t modbus_event_group_handle;

// diagnostics variable
diagnostics_type_t diagnostics;
char uart_tx_buffer[255]; // todo: remove magic buffer size

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/**
 * ====================== Function prototypes ===========================
 */

#ifdef __GNUC__
  int __io_putchar(int ch) {
      HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
      return ch;
  }
#else
  int fputc(int ch, FILE *f) {
      HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
      return ch;
  }
#endif

/**
 * @brief Function to print to console
 */
void UART_print(const char* message);

/**
 * @brief Function to send data to MODBUS master
 *
 */
void send_modbus_data_to_UART1(char* msg);

/**
 * @brief send response to master
 */
void MODBUS_send_response(uint8_t* response, uint16_t len);

/**
 * @brief Reply to MODBUS
 */
void modbus_reply(char* msg, uint16_t length);

/**
 * Task prototypes
 *
 */
void x_task_receive_modbus_RTU(void const* argument);
void x_task_receive_modbus_TCP(void const* argument);
void x_task_relay_control(void const* argument);
void x_task_ethernet_control(void const* argument);
void x_task_get_device_diagnostics(void const* argument);
void x_task_print_to_terminal(void const* argument);
void x_task_clean_modbus_RTU_queue(void const* argument);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @fn void UART_print(const char*)
 * @brief This function prints messages to serial terminal regadless of where it is called
 * It solvs the problem of not being able to print to terminal once t
 * the RTOS starts creating tasks, queues etc...
 *
 * @param message char buffer to print
 */
//void UART_print(const char* msg) {
//	const char* p = msg;
//
//	while(*p) {
//		while(!(huart1.Instance->SR & USART_SR_TXE)); 	// wait until TX buffer is empty
//		huart1.Instance->DR = (*p++ & 0xFF);			// write next character
//	}
//
//	//HAL_UART_Transmit(&huart1,(uint8_t*)message , strlen(message), HAL_MAX_DELAY);
//
//	while(!(huart1.Instance->SR & USART_SR_TC)); 		// wait for last byte to fully transmit
//}

void UART_print(const char* msg) {
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // enable IDLE LINE INTERRUPT for UART2 that is connected to MAX485 module
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, modbus_rx_message, MODBUS_RTU_MAX_SIZE);

  UART_print("=======MODBUS SLAVE DEVICE======= \r\n");
//  HAL_UART_Transmit(&huart1, (uint8_t*)"=======MODBUS SLAVE DEVICE======= \r\n\n", strlen("=======MODBUS SLAVE DEVICE======= \r\n\n"), HAL_MAX_DELAY);
//  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  //x_relay_control_semaphore = xSemapahoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  modbus_RTU_queue_handle = xQueueCreate(5, sizeof(ModBus_type_t)); // todo: check for successful creation

  if(modbus_RTU_queue_handle != NULL) {
	  //UART_print("MODBUS queue created OK");
	  HAL_UART_Transmit(&huart1,(uint8_t*)"MODBUS queue created OK\r\n", strlen("MODBUS queue created OK\r\n"), HAL_MAX_DELAY);
  } else {
	  //UART_print("MODBUS queue failed to create");
	  HAL_UART_Transmit(&huart1,(uint8_t*)"MODBUS queue failed to create\r\n", strlen("MODBUS queue failed to create\r\n"), HAL_MAX_DELAY);
  }


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadDef(get_device_diagnostics, x_task_get_device_diagnostics, osPriorityIdle + 3, 0, 128); // task to get the device parameters
  x_task_get_device_diagnostics_handle = osThreadCreate(osThread(get_device_diagnostics), NULL);

  osThreadDef(receive_modbus_RTU, x_task_receive_modbus_RTU, osPriorityNormal , 0, 2048); // task to receive MODBUS data
  x_task_receive_modbus_RTU_handle = osThreadCreate(osThread(receive_modbus_RTU), NULL);

  osThreadDef(receive_modbus_TCP, x_task_receive_modbus_TCP, osPriorityIdle + 3, 0, 128); // task to receive data via MODBUS TCP
  x_task_receive_modbus_TCP_handle = osThreadCreate(osThread(receive_modbus_TCP), NULL);

  osThreadDef(print_to_terminal, x_task_print_to_terminal, osPriorityNormal, 0, 128); // task to print to UART if using UART debug
  x_task_print_to_terminal_handle = osThreadCreate(osThread(print_to_terminal), NULL);

  osThreadDef(control_relay, x_task_relay_control, osPriorityNormal , 0, 1024); // task to control relays
  x_task_relay_control_handle = osThreadCreate(osThread(control_relay), NULL);

  osThreadDef(ethernet_control, x_task_ethernet_control, osPriorityNormal , 0, 1024); // task to control ethernet communicattion
  x_task_ethernet_control_handle = osThreadCreate(osThread(ethernet_control), NULL);


  // todo -> check successful task creation

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(user_led_GPIO_Port, user_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_RE_PIN_GPIO_Port, DE_RE_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISR_DBG_LED_GPIO_Port, ISR_DBG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : user_led_Pin */
  GPIO_InitStruct.Pin = user_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(user_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DE_RE_PIN_Pin */
  GPIO_InitStruct.Pin = DE_RE_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DE_RE_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ISR_DBG_LED_Pin */
  GPIO_InitStruct.Pin = ISR_DBG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ISR_DBG_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @fn void x_task_get_device_diagnostics(const void*)
 * @brief Get the device diagnostics values (see firmware docs for more info)
 *
 * @param args parameter to the task
 */
void x_task_get_device_diagnostics(void const* args) {


	for(;;) {
		diagnostics.chip_parameters.uid[0] = HAL_GetUIDw0();		// read the chip's UID
		diagnostics.chip_parameters.uid[0] = HAL_GetUIDw0();
		diagnostics.chip_parameters.uid[2] = HAL_GetUIDw0();        // todo: create a 96 bit struct for this (64 bit + 32 bit)

		uint32_t core_freq = HAL_RCC_GetHCLKFreq();					// read the HCLK frequency. todo: read PCLK low freq speed
		diagnostics.chip_parameters.core_frequency = core_freq;

		diagnostics.free_heap_size = xPortGetFreeHeapSize();			// free heap size at the time this function is called
		diagnostics.minimum_ever_free_heap_size = xPortGetMinimumEverFreeHeapSize();

		sprintf(uart_tx_buffer,   // package uart message
				"UID: %lu%lu%lu, HCLK: %lu, FREE_HEAP: %u, MIN_EVER_HEAP: %u \r\n",
				diagnostics.chip_parameters.uid[0],
				diagnostics.chip_parameters.uid[1],
				diagnostics.chip_parameters.uid[2],
				diagnostics.chip_parameters.core_frequency,
				diagnostics.free_heap_size,
				diagnostics.minimum_ever_free_heap_size
		);


		vTaskDelay(pdMS_TO_TICKS(10));

	}
}

/**
 * @fn void x_task_print_to_terminal(const void*)
 * @brief This task prints modbus RTU received packet to terminal
 *
 * @param arguments
 */
void x_task_print_to_terminal(void const* arguments ) {

	for(;;) {
		// todo: use event group to queue peek here
		// use mutex to prevent writing to the uart tx buffer while the buffer is printing
		//UART_print(uart_tx_buffer);
		vTaskDelay(pdMS_TO_TICKS(5)); 	// prevent task starvation
	}
}

/**
 * @brief send MODBUS message to UART1 for debugging
 */
void send_modbus_data_to_UART1(char* msg) {
	// enable transmit
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(  (char*) msg), HAL_MAX_DELAY); // check delay time in prod code

}

void MODBUS_send_response(uint8_t* response, uint16_t len) {
	HAL_UART_Transmit(&huart2, response, len, HAL_MAX_DELAY);

	// todo: use interrupts here
}

/**
 * @brief THis task receives MODBUS data from the master and parses it
 */
void x_task_receive_modbus_RTU(void const* argument) {
	ModBus_type_t modbus_message;
	uint8_t response[MODBUS_RTU_MAX_SIZE];  // this will hold the slave response back to master
	//uint16_t response_length;

	//HAL_UARTEx_ReceiveToIdle_IT(&huart2, modbus_rx_message, MODBUS_MSG_MAX_SIZE);

	for(;;) {
		//send_modbus_data_to_UART1("In receive modbus data task\n");

		if(xQueuePeek(modbus_RTU_queue_handle, &modbus_message, 1000) == pdTRUE) {
			send_modbus_data_to_UART1("RECEIVED MASTER REQUEST OK\n");
			// debug via USART1

			if(modbus_message.len < 4) continue; // skip this frame its too short

			// check CRC
			uint16_t crc_hi = modbus_message.data[modbus_message.len - 1];
			uint16_t crc_lo = modbus_message.data[modbus_message.len - 2];
			uint16_t received_crc = (crc_hi << 8) | crc_lo;

			/* calculate crc and compare */
			uint16_t calculated_crc = MAX485_calculate_CRC(modbus_message.data, modbus_message.len - 2);

			if(received_crc != calculated_crc) {
				continue; // ignore bad CRC
			}

			/* check if this message is addressed for me */
			uint8_t received_slave_id = modbus_message.data[0];
			if((received_slave_id != SLAVE_ID ) && received_slave_id != 0 ) {
				continue;
			}

			/* check what function code was received */
			uint8_t function_code = modbus_message.data[1];

			// todo: check for supported functions


			// Here I handle supported functions
			if(function_code == 0x01) { // READ COILS  todo: put this into its own functions

				// sample packet structure for read coils request is as follows
				// [addr][func=0x01][addr_hi][addr_lo][qty_hi][qty_lo][crc_lo][crc_hi]
				//MAX_485_read_coils_handler();
				HAL_UART_Transmit(&huart1, (uint8_t*) "READ COILS\r\n", strlen("READ COILS \r\n"), HAL_MAX_DELAY);

				uint16_t start = (modbus_message.data[2] << 8) | (modbus_message.data[3]); // what coil to start from
				uint16_t qty = (modbus_message.data[4] << 8) | (modbus_message.data[5]);

				// todo: check number of coils if valid greater than 2000 of less than 1
				// build an exception here

				// get slave address
				response[0] = received_slave_id;
				response[1] = 0x01; // function code
				response[2] = (qty + 7)/8; // 7 ensures a proper roundup to get number of bytes needed to hold the requested number of bits

				uint16_t index = 3;
				uint8_t coil_byte = 0;
				uint8_t bit_pos = 0;

				for(uint16_t i = 0; i < qty; i++) {

					/* get the actual coil value - bit extraction*/
					uint8_t coil_val = (coils[(start + i) / 8] >>  ( (start + i) %8)) & 0x01;

					if(coil_val) {
						coil_byte |= (1 << bit_pos); // if coil is 1, set the bit at that position
					}

					bit_pos++; // next bit position

					if(bit_pos == 8 || i == qty - 1) {
						response[index++] = coil_byte;
						coil_byte = 0;
						bit_pos = 0;
					}
				}

				// append the CRC byte
				uint16_t crc = MAX485_calculate_CRC(response, index);
				response[index++] = crc & 0xFF; // get CRC LOW
				response[index++] = (crc >> 8) & 0xFF; // get CRC HIGH

				uint8_t response_length = index;

				send_modbus_data_to_UART1(response);

				printf("Received REQUEST\r\n");
				for(uint16_t i=0; i < modbus_message.len; i++) {
					printf("%02X ", modbus_message.data[i]);
				}

				printf("\r\n");

				printf("Computed response\r\n");
				for(uint16_t i=0; i < response_length; i++) {
					printf("%02X ", response[i]);
				}

				printf("\r\n");

				// todo: transmit to master
				MODBUS_send_response(response, response_length);
				MODBUS_send_response(modbus_message.data, modbus_message.len); // loopback test

			} else if (function_code == 0x05) {   /* WRITE SINGLE COIL */

				HAL_UART_Transmit(&huart1, (uint8_t*) "WRITE SINGLE COIL\r\n", strlen("WRITE SINGLE COIL\r\n"), HAL_MAX_DELAY);

				// get the starting coil address
				uint16_t start_addr = (modbus_message.data[2] << 8) | modbus_message.data[3];
				uint16_t coil_value = (modbus_message.data[4] << 8) | modbus_message.data[5];

				// check if value ON or OFF
				if(coil_value == 0xFF00) { // coil ON
					COIL[start_addr] = 1;
				} else if(coil_value == 0x00) { // COIL OFF
					COIL[start_addr] = 0;
				} else {
					// todo: handle noise/illegal data
				}

				// build the response
				response[0] = received_slave_id;
				response[1] = 0x05; // function code
				response[2] = modbus_message.data[2];
				response[3] = modbus_message.data[3];
				response[4] = modbus_message.data[4];
				response[5] = modbus_message.data[5];

				// calculate CRC
				uint16_t crc = MAX485_calculate_CRC(response, 6);
				response[6] = crc & 0xFF;
				response[7] = (crc >> 8) & 0xFF;

				// echo back the same request as response

				uint8_t response_length = 8;
				// todo: send response (response_buffer, response_length)

			} else if(function_code == 0x0F) {  //  WRITE MULTIPLE COILS

				HAL_UART_Transmit(&huart1, (uint8_t*) "WRITE MULTIPLE COILS\r\n", strlen("WRITE MULTIPLE COILS\r\n"), HAL_MAX_DELAY);

				/* extract data from MODBUS packet */
				uint16_t start = (modbus_message.data[2] << 8) | (modbus_message.data[3]);
				uint16_t qty = (modbus_message.data[4] << 8) | modbus_message.data[5];
				uint8_t byte_count = modbus_message.data[6];

				/* update my COIL array from packed request bytes */
				for(uint16_t i = 0; i < qty; i++) {
					uint16_t byte_index = i / 8;  // depending on number of bytes received
					uint8_t bit_index = i % 8;   // get bit position
					uint8_t coil_val = (COIL[byte_index] >> bit_index) & 0x01;  // this sets or clears a bit at that position
					COIL[start + i] = coil_val; // here, I update the coil byte with the hnew written value
				}

				/* release semaphore here to notify the relay control task that we are done updating coils */


			}

			vTaskDelay(pdMS_TO_TICKS(5));
		} else {
			//UART_print("Failed to receive from queue\n");
			// todo: log error count
		}

	}

}

/**
 * @brief This task receives MODBUS data via MODBUS TCP
 */

void x_task_receive_modbus_TCP(void const* arguments) {

	for(;;) {
		// get the MODBUS HEADER
	}
}



/**
 * @brief Reply to MODBUS Master
 */
void modbus_reply(char* msg, uint16_t length) {
	HAL_GPIO_WritePin(DE_RE_PIN_GPIO_Port, DE_RE_PIN_Pin, GPIO_PIN_SET);

	// send data
	HAL_UART_Transmit(&huart2, modbus_rx_message, strlen((char*)modbus_rx_message), 500);

	// enable receive
	HAL_GPIO_WritePin(DE_RE_PIN_GPIO_Port, DE_RE_PIN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief This task controls the relays
 */
void x_task_relay_control(void const* arguments) {
	for(;;) {
		// if can take sempahore,
		// HAL write relay banks
		//give semaphore
	}
}

/**
 * @brief This task controls ethernet communication
 */
void x_task_ethernet_control(void const* argument) {
	for(;;) {

	}
}

/**
 * @brief This task removes data from MODBUS RTU queue after all teh consuming tasks are done peeking
 * the queue
 */
void x_task_clean_modbus_RTU_queue(void const* argument) {
	EventBits_t x_event_group_value;			 ///< event group value
	const EventBits_t x_bits_to_wait_for = (PRINT_TO_TERMINAL_BIT | RECEIVE_MODBUS_BIT); ///< wait for these 2 bits to be set
	ModBus_type_t modbus_RTU_message; ///< variable to save the removed value to

	for(;;) {

		x_event_group_value = xEventGroupWaitBits(
				modbus_event_group_handle, 		///< event group handle to wait for
				x_bits_to_wait_for, 			///< bit sequence to wait for
				pdTRUE, 						///< clear all bits on exit if unblock condition is met
				pdTRUE, 						///< wait for all bits to be set
				HAL_MAX_DELAY); 				///< TODO: remove this from MAX in prod
	};

	// at this point all the items have been received by consumer
	xQueueReceive(modbus_RTU_queue_handle, &modbus_RTU_message, HAL_MAX_DELAY); // todo: use a defined time in prod

}


/**
 * @brief Callback called when IDLE is detected or we a re done receiving MODBUS Message
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	if(huart->Instance == USART2) { 			  // MAX485 is connected to USART2

		//HAL_UART_Transmit(&huart1, (uint8_t*)"Received on USART 2\r\n", strlen( "Received on USART 2\r\n"), HAL_MAX_DELAY);

		//send_modbus_data_to_UART1("Data arrived on MODBUS\n");
		//HAL_GPIO_TogglePin(ISR_DBG_LED_GPIO_Port, ISR_DBG_LED_Pin);

		ModBus_type_t msg;
		msg.len = Size; 						   // whatever length that has been received
		memcpy(msg.data, modbus_rx_message, Size); // copy to MODBUS data field

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(modbus_RTU_queue_handle, &msg, &xHigherPriorityTaskWoken);
		// todo: check for failed queue send and log error

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	// if task waiting for modbus has a higher priority, it will ranm(pre-empt a lower priroty task)

		// restart the RECEIVE TO idle interrupt
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*) modbus_rx_message, MODBUS_RTU_MAX_SIZE);

	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
