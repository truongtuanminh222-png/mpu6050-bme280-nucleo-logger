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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// --- I2C DEVICE HARDWARE ADDRESSES ---
#define MPU6050_ADDR         (0x68 << 1) // Left-shifted for HAL 8-bit addressing
#define BME280_ADDR          (0x76 << 1) // Change to (0x77 << 1) if your module requires it

// --- SENSOR REGISTER MAP ---
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define BME280_REG_DIG_T1    0x88        // Starting address of BME280 calibration data
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_DATA      0xF7        // Press MSB, Press LSB, Press XLSB, Temp MSB...

// --- GLOBAL STORAGE VARIABLES ---
uint8_t i2c_buf[24];
int16_t raw_accel_x, raw_accel_y, raw_accel_z;
float accel_x, accel_y, accel_z;
float bme_temp, bme_press;

// --- BME280 FACTORY CALIBRATION CONSTANTS ---
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t  t_fine; // Shares temperature tracking cross-calculations dynamically

// --- LAST-READ SUCCESS FLAGS (diagnostic: distinguishes real sensor data
//     from stale cached values caused by a failed I2C transaction) ---
uint8_t accel_read_ok = 0;
uint8_t bme_read_ok = 0;

// --- I2C BUS RECOVERY TRACKING ---
uint16_t i2c_consec_fail_count = 0;   // resets to 0 on any successful read
uint32_t last_recovery_tick = 0;      // prevents recovery attempts back-to-back
#define I2C_FAIL_THRESHOLD 5          // consecutive failures before attempting recovery
#define I2C_RECOVERY_COOLDOWN_MS 500  // minimum gap between recovery attempts

// --- DEFERRED BUTTON EVENT ---
// The ISR only sets this flag; all real work (toggling state, printf) happens
// in the main loop. Interrupt handlers must stay short and non-blocking --
// calling printf() from inside an ISR can corrupt UART/I2C timing state.
volatile uint8_t button_event_pending = 0;

// --- START/STOP STREAMING CONTROL ---
volatile uint8_t streaming_active = 0;   // 0 = idle, 1 = printing data
volatile uint32_t last_button_tick = 0;  // for button debounce

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
  return ch;
}

// --- DECODER MATH: CONVERT RAW BME280 BYTES TO ACTUAL PHYSICAL VALUES ---
int32_t BME280_Compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t BME280_Compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 31);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0; // Prevent fatal division-by-zero crashes
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

// --- SAFE, NON-INTERRUPT WAY TO CHECK FOR AN "ENTER" KEYPRESS ---
// Polls the UART with a very short timeout so it never blocks the main loop
// for long, and never touches the NVIC/interrupt vector table.
static uint8_t Check_Enter_Pressed(void)
{
    uint8_t byte;
    if (HAL_UART_Receive(&huart2, &byte, 1, 5) == HAL_OK) // 5 ms timeout poll
    {
        if (byte == '\r' || byte == '\n') {
            return 1;
        }
    }
    return 0;
}

// --- SAFE I2C1 RE-INITIALIZATION (does NOT call Error_Handler on failure) ---
// Mirrors the register setup in MX_I2C1_Init(), but that CubeMX-generated
// function calls Error_Handler() -> __disable_irq(); while(1){} on failure,
// which would permanently freeze the whole MCU (including the button and
// UART) if the bus was still unhappy. This version just reports the result
// so the rest of the program (button, UART, main loop) keeps running.
static HAL_StatusTypeDef I2C1_SafeReinit(void)
{
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(10);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_StatusTypeDef status = HAL_I2C_Init(&hi2c1);
    HAL_Delay(10);
    return status; // caller decides what to do -- NEVER calls Error_Handler() here
}

// --- I2C BUS RECOVERY ---
// Called after several consecutive read failures in a row. Re-initializes the
// I2C peripheral (clears stuck HAL state) and re-wakes the MPU6050. This fixes
// software-level lockups; if the bus is physically stuck (SDA held low by a
// slave mid-transaction), a manual GPIO clock-out sequence would be needed
// instead, which requires knowing the exact I2C1 SCL/SDA pin assignment.
static void I2C_Bus_Recover(void)
{
    printf("!!! I2C BUS RECOVERY TRIGGERED after %u consecutive failures !!!\r\n", i2c_consec_fail_count);
    fflush(stdout);

    HAL_StatusTypeDef status = I2C1_SafeReinit();

    if (status == HAL_OK) {
        // Re-wake the MPU6050 in case its internal state needs to be reasserted
        uint8_t wake_buf[2] = { MPU6050_PWR_MGMT_1, 0x00 };
        HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, wake_buf, 2, 100);
        printf("I2C recovery: peripheral re-initialized OK.\r\n");
    } else {
        printf("I2C recovery FAILED (HAL status %d) -- check wiring/connections. Will keep retrying.\r\n", (int)status);
    }
    fflush(stdout);

    i2c_consec_fail_count = 0;
    last_recovery_tick = HAL_GetTick();
}

// --- SHARED START/STOP LOGIC ---
// Called ONLY from the main loop (never from an ISR) so printf/blocking UART
// calls never run inside interrupt context.
static void Handle_Streaming_Toggle(void)
{
    streaming_active = !streaming_active;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // visual confirmation, independent of terminal
    if (streaming_active) {
        printf("--- STREAMING STARTED ---\r\n");
        printf("AccelX_g,AccelY_g,AccelZ_g,Temp_C,Press_hPa,AccelOK,BmeOK\r\n");
    } else {
        printf("--- STREAMING STOPPED ---\r\n");
    }
    fflush(stdout);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Force UNBUFFERED stdout: without this, printf() output can sit in an
  // internal buffer indefinitely and never actually reach the UART, since
  // this stream is not recognized as a real terminal by the C library.
  setvbuf(stdout, NULL, _IONBF, 0);

  HAL_Delay(500); // Give lines time to physically settle completely

  // 1. WAKE UP MPU6050: Write 0x00 to register 0x6B (Power Management)
  i2c_buf[0] = MPU6050_PWR_MGMT_1;
  i2c_buf[1] = 0x00;
  if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, i2c_buf, 2, 100) != HAL_OK) {
      printf("BUS_ERROR: MPU6050 Init Failed\r\n");
  }

  // 2. READ BME280 FACTORY CALIBRATION PARAMS (24 consecutive memory bytes)
  i2c_buf[0] = BME280_REG_DIG_T1;
  HAL_I2C_Master_Transmit(&hi2c1, BME280_ADDR, i2c_buf, 1, 100);
  if (HAL_I2C_Master_Receive(&hi2c1, BME280_ADDR, i2c_buf, 24, 100) == HAL_OK) {
      dig_T1 = (i2c_buf[1] << 8) | i2c_buf[0];
      dig_T2 = (i2c_buf[3] << 8) | i2c_buf[2];
      dig_T3 = (i2c_buf[5] << 8) | i2c_buf[4];
      dig_P1 = (i2c_buf[7] << 8) | i2c_buf[6];
      dig_P2 = (i2c_buf[9] << 8) | i2c_buf[8];
      dig_P3 = (i2c_buf[11] << 8) | i2c_buf[10];
      dig_P4 = (i2c_buf[13] << 8) | i2c_buf[12];
      dig_P5 = (i2c_buf[15] << 8) | i2c_buf[14];
      dig_P6 = (i2c_buf[17] << 8) | i2c_buf[16];
      dig_P7 = (i2c_buf[19] << 8) | i2c_buf[18];
      dig_P8 = (i2c_buf[21] << 8) | i2c_buf[20];
      dig_P9 = (i2c_buf[23] << 8) | i2c_buf[22];
  } else {
      printf("BUS_ERROR: BME280 Calib Table Read Failed\r\n");
  }
  fflush(stdout);

  // Print column header once, then wait for the user to start streaming
  printf("AccelX_g,AccelY_g,AccelZ_g,Temp_C,Press_hPa,AccelOK,BmeOK\r\n");
  printf("AccelOK/BmeOK: 1 = fresh reading this cycle, 0 = I2C read failed (value is stale)\r\n");
  printf("Press ENTER in the terminal or the blue User button (B1) to START/STOP logging.\r\n");
  fflush(stdout);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Non-blocking-ish check for "Enter" pressed in the terminal (5 ms max wait)
	  if (Check_Enter_Pressed())
	  {
	        streaming_active = !streaming_active;
	        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // visual confirmation, independent of terminal
	        if (streaming_active) {
	            printf("--- STREAMING STARTED ---\r\n");
	            printf("AccelX_g,AccelY_g,AccelZ_g,Temp_C,Press_hPa,AccelOK,BmeOK\r\n");
	        } else {
	            printf("--- STREAMING STOPPED ---\r\n");
	        }
	        fflush(stdout);
	  }

	  if (streaming_active)
	  {
	        // ----------------------------------------------------------------------
	        // STEP A: POLL THE MOTION CHIP (MPU6050 Continuous Read)
	        // ----------------------------------------------------------------------
	        i2c_buf[0] = MPU6050_ACCEL_XOUT_H;
	        HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, i2c_buf, 1, 50);
	        if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, i2c_buf, 6, 50) == HAL_OK) {
	            accel_read_ok = 1;
	            raw_accel_x = (int16_t)((i2c_buf[0] << 8) | i2c_buf[1]);
	            raw_accel_y = (int16_t)((i2c_buf[2] << 8) | i2c_buf[3]);
	            raw_accel_z = (int16_t)((i2c_buf[4] << 8) | i2c_buf[5]);

	            accel_x = (float)raw_accel_x / 16384.0f;
	            accel_y = (float)raw_accel_y / 16384.0f;
	            accel_z = (float)raw_accel_z / 16384.0f;
	        } else {
	            accel_read_ok = 0; // keeps last-known values, but flags them as stale
	        }

	        // ----------------------------------------------------------------------
	        // STEP B: TRIGGER CLIMATE SNAPSHOT (BME280 Forced Mode Activation)
	        // ----------------------------------------------------------------------
	        i2c_buf[0] = BME280_REG_CTRL_MEAS;
	        i2c_buf[1] = 0x25; // Temp oversampling x1, Press oversampling x1, Forced Mode
	        HAL_I2C_Master_Transmit(&hi2c1, BME280_ADDR, i2c_buf, 2, 50);

	        HAL_Delay(10); // Give the sensor a brief window to complete conversion math

	        // ----------------------------------------------------------------------
	        // STEP C: READ OUT CLIMATE MEASUREMENTS
	        // ----------------------------------------------------------------------
	        i2c_buf[0] = BME280_REG_DATA;
	        HAL_I2C_Master_Transmit(&hi2c1, BME280_ADDR, i2c_buf, 1, 50);
	        if (HAL_I2C_Master_Receive(&hi2c1, BME280_ADDR, i2c_buf, 6, 50) == HAL_OK) {
	            bme_read_ok = 1;
	            int32_t raw_press = (int32_t)((i2c_buf[0] << 12) | (i2c_buf[1] << 4) | (i2c_buf[2] >> 4));
	            int32_t raw_temp  = (int32_t)((i2c_buf[3] << 12) | (i2c_buf[4] << 4) | (i2c_buf[5] >> 4));

	            bme_temp  = (float)BME280_Compensate_T(raw_temp) / 100.0f;
	            bme_press = (float)BME280_Compensate_P(raw_press) / 256.0f / 100.0f;
	        } else {
	            bme_read_ok = 0; // keeps last-known values, but flags them as stale
	        }

	        // ----------------------------------------------------------------------
	        // I2C HEALTH CHECK: recover automatically if reads keep failing
	        // ----------------------------------------------------------------------
	        if (accel_read_ok && bme_read_ok) {
	            i2c_consec_fail_count = 0;
	        } else {
	            i2c_consec_fail_count++;
	            if (i2c_consec_fail_count >= I2C_FAIL_THRESHOLD &&
	                (HAL_GetTick() - last_recovery_tick) > I2C_RECOVERY_COOLDOWN_MS) {
	                I2C_Bus_Recover();
	            }
	        }

	        // ----------------------------------------------------------------------
	        // STEP D: EMIT MACHINE-READABLE CSV FRAME
	        // ----------------------------------------------------------------------
	        printf("%.3f,%.3f,%.3f,%.2f,%.2f,%d,%d\r\n", accel_x, accel_y, accel_z, bme_temp, bme_press, accel_read_ok, bme_read_ok);
	        fflush(stdout);

	        HAL_Delay(90); // Controls overall output speed (~10 samples per second)
	  }
	  else
	  {
	        HAL_Delay(20); // Idle: avoid busy-spinning while waiting to start
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // Ensure the EXTI interrupt for B1 (blue User button) is enabled at the NVIC.
  // Harmless if already enabled elsewhere (e.g. by CubeMX-generated code).
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Called by the HAL when a configured EXTI line fires. B1 (blue User
  *         button) is wired to a falling-edge interrupt. A simple 200 ms
  *         software debounce prevents a single physical press from being
  *         read as several rapid toggles.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_button_tick) > 200)
    {
      last_button_tick = now;
      streaming_active = !streaming_active;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // visual confirmation, independent of terminal
      if (streaming_active) {
        printf("--- STREAMING STARTED ---\r\n");
        printf("AccelX_g,AccelY_g,AccelZ_g,Temp_C,Press_hPa,AccelOK,BmeOK\r\n");
      } else {
        printf("--- STREAMING STOPPED ---\r\n");
      }
      fflush(stdout);
    }
  }
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
