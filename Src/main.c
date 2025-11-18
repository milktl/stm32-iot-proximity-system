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
#include "gpio.h"
#include "usart.h"
#include "icache.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>

/* === Configuration === */
#define BUZZER_Pin      GPIO_PIN_0
#define BUZZER_Port     GPIOE
#define AVG_WINDOW      3
#define RX_BUFFER_SIZE  64

/* === Wi-Fi Credentials === */
#define WIFI_SSID  "Verizon_TG93GB-IoT"
#define WIFI_PSK   "loop-privy4-icy"

/* === Variables === */
float   last_readings[AVG_WINDOW] = {0};
int     idx = 0;
char    last_display[32] = "";
static uint8_t esp_rx[RX_BUFFER_SIZE];

/* === Function Prototypes === */
void SystemClock_Config(void);
void Error_Handler(void);
void Buzzer_Beep(uint32_t duration_ms);
/* USER CODE BEGIN PFP */
// Manual init for LPUART1 (PC terminal)
void MX_LPUART1_UART_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Handle for LPUART1
UART_HandleTypeDef hlpuart1;

/* Simple tagged logger to PC terminal (LPUART1) */
static void log_pc(const char *tag, const char *s)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)tag, strlen(tag), HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)s,   strlen(s),   HAL_MAX_DELAY);
}

/* keep last thing we TX to ESP so we can suppress pure echo */
static char last_tx_copy[128];

/* Send raw to ESP8266 (USART2) and mirror to PC log */
static void esp_tx(const char *s)
{
  /* save last TX (bounded) */
  strncpy(last_tx_copy, s, sizeof(last_tx_copy)-1);
  last_tx_copy[sizeof(last_tx_copy)-1] = 0;

  log_pc("[TX2ESP] ", s);
  HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* Command helper: send -> wait -> print all RX -> optionally check substring */
static int esp_cmd(const char *cmd, uint32_t wait_ms, const char *must_contain)
{
  esp_tx(cmd);

  static uint8_t buf[512];
  uint32_t last = HAL_GetTick();
  size_t pos = 0;

  while ((HAL_GetTick() - last) < wait_ms && pos < sizeof(buf) - 1)
  {
    uint8_t b;
    if (HAL_UART_Receive(&huart2, &b, 1, 50) == HAL_OK)
    {
      buf[pos++] = b;
      last = HAL_GetTick();
    }
  }
  buf[pos] = 0;

  if (pos)
  {
    /* suppress pure echo of our last TX */
    size_t txlen = strlen(last_tx_copy);
    if (!(txlen && pos == txlen && strncmp((char*)buf, last_tx_copy, txlen) == 0))
    {
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)"[ESP_RX] ", 9, HAL_MAX_DELAY);
      HAL_UART_Transmit(&hlpuart1, buf, pos, HAL_MAX_DELAY);
    }
  }

  if (must_contain == NULL) return 1;
  return (strstr((char*)buf, must_contain) != NULL);
}

/* Quietly drain any ESP bytes for up to idle_ms; only prints if data exists */
static void esp_drain(uint32_t idle_ms)
{
  static uint8_t buf[256];
  uint32_t last = HAL_GetTick();
  size_t pos = 0;

  while ((HAL_GetTick() - last) < idle_ms && pos < sizeof(buf)-1)
  {
    uint8_t b;
    if (HAL_UART_Receive(&huart2, &b, 1, 10) == HAL_OK) {
      buf[pos++] = b;
      last = HAL_GetTick();
    }
  }
  if (pos) {
    /* suppress pure echo of our last TX */
    size_t txlen = strlen(last_tx_copy);
    if (!(txlen && pos == txlen && strncmp((char*)buf, last_tx_copy, txlen) == 0))
    {
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)"[ESP_RX] ", 9, HAL_MAX_DELAY);
      HAL_UART_Transmit(&hlpuart1, buf, pos, HAL_MAX_DELAY);
    }
  }
}

/* Reset ESP then disable echo (ATE0 must be done AFTER every reset) */
static void esp_reset_and_noecho(void)
{
  /* soft reset, wait for "ready" */
  esp_cmd("AT+RST\r\n", 3000, "ready");

  /* try ATE0 a few times until OK shows up */
  for (int i = 0; i < 3; i++) {
    if (esp_cmd("ATE0\r\n", 600, "OK")) break;
    HAL_Delay(100);
  }
}

/* Manual init function, including clock & GPIO setup for LPUART1 */
void MX_LPUART1_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_LPUART1_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* PG7 = TX, PG8 = RX for LPUART1 */
  GPIO_InitStruct.Pin       = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  hlpuart1.Instance            = LPUART1;
  hlpuart1.Init.BaudRate       = 115200;
  hlpuart1.Init.WordLength     = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits       = UART_STOPBITS_1;
  hlpuart1.Init.Parity         = UART_PARITY_NONE;
  hlpuart1.Init.Mode           = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE END 0 */

/* === Buzzer Function === */
void Buzzer_Beep(uint32_t duration_ms) {
  for (uint32_t i = 0; i < duration_ms * 2; i++) {
    HAL_GPIO_WritePin(BUZZER_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(BUZZER_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USART2_UART_Init();    /* ESP8266 interface (PA2 TX, PA3 RX) */
  MX_LPUART1_UART_Init();   /* PC terminal (PG7 TX, PG8 RX) */
  LCD_Init();

  /* Startup banner on LCD */
  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_SendString("Booting IOT...");

  /* Sanity check banner to PC */
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Nucleo Start\r\n", 14, HAL_MAX_DELAY);

  /* === ESP8266 bring-up with visible responses === */
  esp_cmd("AT\r\n",                  600,  "OK");
  esp_cmd("AT+CWMODE=1\r\n",         800,  "OK");

  /* Reset then disable echo (so TX doesn't appear again as RX) */
  esp_reset_and_noecho();

  /* Mask creds in PC log, send real ones to ESP */
  log_pc("[TX2ESP] ", "AT+CWJAP=\"<ssid>\",\"<psk>\"\r\n");
  char join_cmd[96];
  snprintf(join_cmd, sizeof(join_cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PSK);
  esp_cmd(join_cmd,                 20000, "WIFI GOT IP");

  /* Confirm association and show IP */
  esp_cmd("AT+CWJAP?\r\n",           1500, "+CWJAP:");
  esp_cmd("AT+CIFSR\r\n",            1500, "STAIP");

  /* Enable DWT for ultrasonic timing */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  while (1)
  {
    /* ── Ultrasonic Trigger ── */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
    uint32_t start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
    uint32_t end = DWT->CYCCNT;

    float us = (float)(end - start) / (SystemCoreClock / 1000000);
    float dist_cm = us / 58.0f;

    last_readings[idx] = dist_cm;
    idx = (idx + 1) % AVG_WINDOW;

    float avg = 0;
    for (int i = 0; i < AVG_WINDOW; i++) avg += last_readings[i];
    avg /= AVG_WINDOW;

    char lcd_msg[32];
    if (avg < 2 || avg > 400) {
      snprintf(lcd_msg, sizeof(lcd_msg), "Out of range");
    } else {
      snprintf(lcd_msg, sizeof(lcd_msg), "Dist: %.2f cm", avg);
    }

    /* Update LCD if changed */
    if (strcmp(lcd_msg, last_display) != 0) {
      LCD_Clear();
      LCD_SetCursor(0, 0);
      LCD_SendString(lcd_msg);
      strcpy(last_display, lcd_msg);
    }

    /* Send distance to ESP and drain any response quietly (echo suppressed) */
    char dist_line[48];
    snprintf(dist_line, sizeof(dist_line), "DIST: %.2f cm\r\n", avg);
    esp_tx(dist_line);
    esp_drain(200);

    /* Buzzer feedback */
    if (avg >= 2 && avg <= 400) {
      if      (avg < 10.0f) Buzzer_Beep(100);
      else if (avg < 30.0f) Buzzer_Beep(50);
      else if (avg < 60.0f) Buzzer_Beep(25);
    }

    HAL_Delay(350);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
    Error_Handler();

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 1;
  RCC_OscInitStruct.PLL.PLLN            = 55;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ }
#endif /* USE_FULL_ASSERT */
