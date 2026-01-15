/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHT41_ADDR 0x44  // 7-bit address
#define SGP40_ADDR 0x59  // 7-bit address

// SHT41 Commands
#define SHT41_MEAS_HIGH_PRECISION 0xFD
#define SHT41_SOFT_RESET 0x94

// SGP40 Commands (from Sensirion driver)
#define SGP40_CMD_MEASURE_RAW_SIGNAL 0x260F
#define SGP40_CMD_MEASURE_TEST 0x280E
#define SGP40_CMD_TURN_HEATER_OFF 0x3615
#define SGP40_CMD_SOFT_RESET 0x0006
#define SGP40_CMD_GET_SERIAL 0x3682

// SGP40 default parameters (from datasheet)
#define SGP40_DEFAULT_HUMIDITY 50.0f  // 50% RH
#define SGP40_DEFAULT_TEMPERATURE 25.0f  // 25°C
#define SGP40_WARMUP_TIME_MS 10000  // 10 seconds warmup
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Print functions
void print_string(const char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void print_int(int value) {
    char buffer[12];
    sprintf(buffer, "%d", value);
    print_string(buffer);
}

void print_float(float value) {
    int integer = (int)value;
    int decimal = (int)(fabs(value - integer) * 100);
    char buffer[20];
    sprintf(buffer, "%d.%02d", integer, decimal);
    print_string(buffer);
}

void print_hex(uint32_t value, int digits) {
    char buffer[10];
    int i;
    for (i = digits - 1; i >= 0; i--) {
        uint8_t nibble = value & 0xF;
        buffer[i] = nibble < 10 ? '0' + nibble : 'A' + (nibble - 10);
        value >>= 4;
    }
    buffer[digits] = '\0';
    print_string(buffer);
}

// Sensirion CRC8 function (from official driver)
static uint8_t sensirion_crc8(const uint8_t *data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = 0xFF;
    uint8_t crc_bit;

    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= data[current_byte];
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

// SGP40 state
typedef struct {
    uint8_t initialized;
    uint32_t last_measurement_time;
    uint16_t serial[3];
} sgp40_state_t;

sgp40_state_t sgp40_state = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// Sensor functions
HAL_StatusTypeDef SHT41_Read(float *temperature, float *humidity);
HAL_StatusTypeDef SGP40_Init(void);
HAL_StatusTypeDef SGP40_MeasureRaw(float temperature, float humidity, uint16_t *raw_signal);
void I2C_Scan(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// I2C Scan function
void I2C_Scan(void) {
    uint8_t devices = 0;
    print_string("Scanning I2C bus...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10);
        if (status == HAL_OK) {
            print_string("  Found device at: 0x");
            print_hex(addr, 2);
            print_string("\r\n");
            devices++;
        }
    }
    if (devices == 0) {
        print_string("  No I2C devices found!\r\n");
    }
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

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  float temperature = 0, humidity = 0;
  uint16_t sgp40_raw = 0;

  // Initialize LED
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  print_string("\r\n\r\n");
  print_string("==========================================\r\n");
  print_string("SHT41 + SGP40 Sensor System\r\n");
  print_string("==========================================\r\n\r\n");

  // I2C Scan
  I2C_Scan();

  // Warm-up period for SGP40
  print_string("\r\nWarming up SGP40 (10 seconds)...\r\n");
  for (int i = 0; i < 10; i++) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      print_string(".");
      HAL_Delay(1000);
  }
  print_string("\r\n\r\n");

  // Initialize SGP40
  if (SGP40_Init() != HAL_OK) {
      print_string("WARNING: SGP40 initialization had issues!\r\n");
      print_string("Will try to use it anyway...\r\n");
      // Force initialization flag to try using it
      sgp40_state.initialized = 1;
  }

  print_string("\r\n");
  print_string("Starting Continuous Measurements:\r\n");
  print_string("=================================\r\n\r\n");

  uint32_t measurement_count = 0;
  uint32_t sgp40_success_count = 0;
  uint32_t sgp40_error_count = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      measurement_count++;

      // Toggle LED
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

      // Read SHT41
      if (SHT41_Read(&temperature, &humidity) == HAL_OK) {
          print_string("[");
          print_int(measurement_count);
          print_string("] SHT41: ");
          print_float(temperature);
          print_string("°C, ");
          print_float(humidity);
          print_string("% RH\r\n");
      } else {
          print_string("[");
          print_int(measurement_count);
          print_string("] SHT41: ERROR\r\n");
          temperature = SGP40_DEFAULT_TEMPERATURE;
          humidity = SGP40_DEFAULT_HUMIDITY;
      }

      // Read SGP40 with SHT41 compensation
      if (sgp40_state.initialized) {
          if (SGP40_MeasureRaw(temperature, humidity, &sgp40_raw) == HAL_OK) {
              sgp40_success_count++;

              print_string("[");
              print_int(measurement_count);
              print_string("] SGP40: Raw=");
              print_int(sgp40_raw);

              // Convert raw signal to VOC index (simplified)
              if (sgp40_raw > 10000) {
                  float voc_index = (sgp40_raw - 20000) / 100.0f;
                  if (voc_index < 0) voc_index = 0;

                  print_string(" (VOC Index≈");
                  print_float(voc_index);
                  print_string(")");
              }

              print_string(" [OK:");
              print_int(sgp40_success_count);
              print_string("]\r\n");

              sgp40_error_count = 0;
          } else {
              sgp40_error_count++;
              print_string("[");
              print_int(measurement_count);
              print_string("] SGP40: ERROR [");
              print_int(sgp40_error_count);
              print_string("]\r\n");

              // Re-initialize after too many errors
              if (sgp40_error_count > 10) {
                  print_string("Re-initializing SGP40...\r\n");
                  SGP40_Init();
                  sgp40_error_count = 0;
              }
          }
      } else {
          // Try to initialize if not already
          if (measurement_count % 20 == 0) {
              SGP40_Init();
          }
          print_string("[");
          print_int(measurement_count);
          print_string("] SGP40: Not initialized\r\n");
      }

      print_string("-----\r\n\r\n");

      HAL_Delay(2000);  // 2 second measurement interval
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

// Read SHT41 Temperature and Humidity
HAL_StatusTypeDef SHT41_Read(float *temperature, float *humidity) {
    uint8_t cmd = SHT41_MEAS_HIGH_PRECISION;
    uint8_t data[6];

    if(HAL_I2C_Master_Transmit(&hi2c1, SHT41_ADDR << 1, &cmd, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(10);

    if(HAL_I2C_Master_Receive(&hi2c1, SHT41_ADDR << 1, data, 6, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    uint16_t temp_raw = (data[0] << 8) | data[1];
    *temperature = -45.0f + (175.0f * temp_raw) / 65535.0f;

    uint16_t hum_raw = (data[3] << 8) | data[4];
    *humidity = -6.0f + (125.0f * hum_raw) / 65535.0f;

    if(*humidity < 0.0f) *humidity = 0.0f;
    if(*humidity > 100.0f) *humidity = 100.0f;

    return HAL_OK;
}

// Simple SGP40 I2C write
static HAL_StatusTypeDef sgp40_write_cmd(uint16_t command) {
    uint8_t buffer[2];
    buffer[0] = command >> 8;
    buffer[1] = command & 0xFF;

    return HAL_I2C_Master_Transmit(&hi2c1, SGP40_ADDR << 1, buffer, 2, 100);
}

// Simple SGP40 I2C read
static HAL_StatusTypeDef sgp40_read_word(uint16_t *data) {
    uint8_t buffer[3];

    if (HAL_I2C_Master_Receive(&hi2c1, SGP40_ADDR << 1, buffer, 3, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    *data = (buffer[0] << 8) | buffer[1];
    // Ignore CRC for now
    return HAL_OK;
}

// Get SGP40 Serial Number
static HAL_StatusTypeDef SGP40_GetSerial(uint16_t *serial) {
    // Send get serial command
    if (sgp40_write_cmd(SGP40_CMD_GET_SERIAL) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(1);

    // Read 3 words (serial number)
    uint8_t buffer[9];
    if (HAL_I2C_Master_Receive(&hi2c1, SGP40_ADDR << 1, buffer, 9, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Extract serial numbers
    serial[0] = (buffer[0] << 8) | buffer[1];
    serial[1] = (buffer[3] << 8) | buffer[4];
    serial[2] = (buffer[6] << 8) | buffer[7];

    return HAL_OK;
}

// Simplified Self Test (optional)
static HAL_StatusTypeDef SGP40_SelfTest(uint16_t *result) {
    // Send self-test command
    if (sgp40_write_cmd(SGP40_CMD_MEASURE_TEST) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(250);

    // Read result
    if (sgp40_read_word(result) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// SGP40 Measure Raw with Compensation
HAL_StatusTypeDef SGP40_MeasureRaw(float temperature, float humidity, uint16_t *raw_signal) {
    uint8_t buffer[8];
    uint16_t index = 0;

    // Command for raw measurement with compensation
    buffer[index++] = 0x26;  // MSB of command
    buffer[index++] = 0x0F;  // LSB of command

    // Humidity compensation (convert to ticks)
    uint16_t hum_ticks = (uint16_t)((humidity * 65535.0f) / 100.0f);
    buffer[index++] = hum_ticks >> 8;
    buffer[index++] = hum_ticks & 0xFF;
    buffer[index++] = sensirion_crc8(&buffer[2], 2);  // CRC for humidity

    // Temperature compensation (convert to ticks)
    uint16_t temp_ticks = (uint16_t)(((temperature + 45.0f) * 65535.0f) / 175.0f);
    buffer[index++] = temp_ticks >> 8;
    buffer[index++] = temp_ticks & 0xFF;
    buffer[index++] = sensirion_crc8(&buffer[5], 2);  // CRC for temperature

    // Send command with compensation data
    if (HAL_I2C_Master_Transmit(&hi2c1, SGP40_ADDR << 1, buffer, index, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Wait for measurement
    HAL_Delay(30);

    // Read raw signal
    return sgp40_read_word(raw_signal);
}

// Initialize SGP40
HAL_StatusTypeDef SGP40_Init(void) {
    uint16_t test_result;
    uint16_t serial[3];

    print_string("Initializing SGP40...\r\n");

    // Wait for power stabilization
    HAL_Delay(100);

    // Try to get serial number first (basic communication test)
    print_string("  Testing basic communication... ");
    if (SGP40_GetSerial(serial) == HAL_OK) {
        print_string("OK\r\n");
        print_string("  Serial: ");
        print_hex(serial[0], 4);
        print_hex(serial[1], 4);
        print_hex(serial[2], 4);
        print_string("\r\n");
    } else {
        print_string("FAILED\r\n");
        print_string("  Check I2C connection and power\r\n");
        return HAL_ERROR;
    }

    // Try self-test (optional)
    print_string("  Running self-test... ");
    uint8_t attempts = 3;
    HAL_StatusTypeDef test_status = HAL_ERROR;

    for (uint8_t i = 0; i < attempts; i++) {
        if (SGP40_SelfTest(&test_result) == HAL_OK) {
            test_status = HAL_OK;
            break;
        }
        HAL_Delay(100);
    }

    if (test_status != HAL_OK) {
        print_string("SKIPPED (I2C error)\r\n");
        print_string("  Continuing without self-test...\r\n");
    } else {
        print_string("Result: 0x");
        print_hex(test_result, 4);

        if (test_result == 0xD400) {
            print_string(" (PASS)\r\n");
        } else {
            print_string(" (Unexpected result)\r\n");
            print_string("  Continuing anyway...\r\n");
        }
    }

    // Mark as initialized
    sgp40_state.initialized = 1;
    sgp40_state.last_measurement_time = HAL_GetTick();

    // Store serial number
    sgp40_state.serial[0] = serial[0];
    sgp40_state.serial[1] = serial[1];
    sgp40_state.serial[2] = serial[2];

    print_string("SGP40 Initialized!\r\n");
    return HAL_OK;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
