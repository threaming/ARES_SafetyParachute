/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLE_ACCEL_LED 1
#define ENABLE_TEMP 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
void scanI2C();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/// @brief Overwrites the __io_putchar function in syscalls.c to use SWO for
/// printf  __io_putchar is called by printf to print characters to the console
int __io_putchar(int ch) {
  // Write character to ITM ch.0
  ITM_SendChar(ch);
  return (ch);
}

/// @brief Scan the I2C bus for devices. Prints out the addresses of devices
/// that respond.
void scanI2C(void) {
  for (uint8_t i = 0; i < 128; i++) {
    uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 100);
    if (ret == HAL_OK) {
      printf("I2C device ready: I2C1: 0x%02X\n", i);
    }
    ret = HAL_I2C_IsDeviceReady(&hi2c2, i, 1, 100);
    if (ret == HAL_OK) {
      printf("I2C device ready: I2C2: 0x%02X\n", i);
    }
  }
}

/// @brief Read the temperature from LIS2DW12. See datasheet p35
/// @param
void I2C1_read_temperature(void) {
  // From documentation:
  // The temperature is available in OUT_T_L (0Dh), OUT_T_H (0Eh) stored as
  // two's complement data, left-justified in 12-bit mode
  uint8_t data[2];
  HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x0D, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
  int16_t temp_raw = (int16_t)((data[1] << 8) | data[0]);
  temp_raw >>= 4; // 12 bit value, so shifted right by 4
  float temperature = 25 + temp_raw / 16.0;
  // NO SUPPORT FOR FLOATING POINT IN PRINTF ENABLED
  uint16_t integerPart = (uint16_t)temperature;
  uint16_t fractionalPart =
      (uint16_t)((temperature - integerPart) * 100); // For 2 decimal places
  printf("Temperature: %d.%02d C\n", integerPart, fractionalPart);
}

/// @brief Read the WHOAMI register of the LIS2DW12. Should always return 0x44.
/// See datasheet p35
/// @param
void I2C1_read_whoami(void) {
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x0F, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  printf("WHOAMI: 0x%02X\n", data);
}

/// @brief Setup the CTRL1 register of the LIS2DW12 to enable the accelerometer.
/// Set up: 0111 400Hz, 01 high performance mode, 00 LP-Mode 12 Bit resolution
/// (not needed). See datasheet p36
/// @param
void I2C1_setup_ctrl1(void) {
  // to register 20h, write 0b0111 01 00
  HAL_I2C_Mem_Write(&hi2c1, 0x30, 0x20, I2C_MEMADD_SIZE_8BIT, (uint8_t *)"\x74",
                    1, 100);
}

/// @brief Read the CTRL1 register of the LIS2DW12, in order for the user to
/// verify it is set up correctly
void I2C1_read_ctrl1(void) {
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x20, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  printf("CTRL1: 0x%02X\n", data);
}
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} xyz_t;

/// @brief Read the X, Y, Z values from the LIS2DW12. See datasheet p43/44
/// @return xyz_t struct containing the raw X, Y, Z values
xyz_t I2C1_read_xyz(void) {
  uint8_t data[6];
  // Read from 0x28 to 0x2D
  HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x28, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
  int16_t x = (int16_t)((data[1] << 8) | data[0]);
  int16_t y = (int16_t)((data[3] << 8) | data[2]);
  int16_t z = (int16_t)((data[5] << 8) | data[4]);
  printf("X: %d, Y: %d, Z: %d\n", x, y, z);
  return (xyz_t){x, y, z};
}

/// @brief Set the state of the red LED
/// @param state GPIO_PIN_SET or GPIO_PIN_RESET
void red_led_state(uint8_t state) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state);
}

/// @brief Set the state of the yellow LED
/// @param state GPIO_PIN_SET or GPIO_PIN_RESET
void yellow_led_state(uint8_t state) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, state);
}

/// @brief Set the state of the green LED
/// @param state GPIO_PIN_SET or GPIO_PIN_RESET
void green_led_state(uint8_t state) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, state);
}

/// @brief Initialize the SPI flash memory
void SPI_flash_init(
    void) { // TODO: UNTESTED, DOESN'T WORK YET - SPI FLASH NEEDS CS PIN!!!
  uint8_t initCommands[] = {0x06}; // Example command to wake up the flash
  HAL_SPI_Transmit(&hspi1, initCommands, sizeof(initCommands), 100);
}
/// @brief Write "Hello World" to the external SPI flash and then read it back
/// and print what it read
void SPI_flash_write_read(
    void) { // TODO: UNTESTED, DOESN'T WORK YET - SPI FLASH NEEDS CS PIN!!!
  uint8_t writeData[] = "Hello World";
  uint8_t readData[12] = {0};

  // Assuming the flash memory is already initialized and ready to use
  // Write "Hello World" to address 0x0000
  HAL_SPI_Transmit(&hspi1, writeData, sizeof(writeData), 100);

  // Read back the data from address 0x0000
  HAL_SPI_Receive(&hspi1, readData, sizeof(readData), 100);

  // Print the read data
  printf("Read from SPI flash: %s\n", readData);
}
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  // SWO_Init(0x1, SystemCoreClock);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  uint32_t count = 0;
  I2C1_setup_ctrl1();
  SPI_flash_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ENABLE_TEMP
    if (count % 100 == 0) {
      I2C1_read_temperature();
    }
#endif
#if ENABLE_ACCEL_LED
    // if (count % 10 == 0) {
    xyz_t results = I2C1_read_xyz();
    if (results.x > 10000) {
      red_led_state(GPIO_PIN_SET);
    } else {
      red_led_state(GPIO_PIN_RESET);
    }
    if (results.y > 10000) {
      yellow_led_state(GPIO_PIN_SET);
    } else {
      yellow_led_state(GPIO_PIN_RESET);
    }
    if (results.z > 10000) {
      green_led_state(GPIO_PIN_SET);
    } else {
      green_led_state(GPIO_PIN_RESET);
    }
    // }
#endif
    HAL_Delay(10);
    count++;
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
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
  huart1.Init.BaudRate = 115200;
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
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 |
                        GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_5 |
                        GPIO_PIN_8 | GPIO_PIN_9,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 PB14 PB15 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 |
                        GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_5 |
                        GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
