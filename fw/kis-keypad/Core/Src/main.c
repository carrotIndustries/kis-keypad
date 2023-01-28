/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_hid.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern USBD_HandleTypeDef hUsbDeviceFS;


typedef struct
{
	uint8_t mod;
	uint8_t reserved;
	uint8_t keys[6];
} keyboardHID;
static keyboardHID keyboardhid = {0,0,{0,0,0,0,0,0}};

static void spi_delay()
{
  for(uint32_t i = 0; i<1000; i++) {
    __NOP();
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
  uint16_t keys = 0;
  uint16_t keys_last = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_WritePin(SR_LOAD_GPIO_Port, SR_LOAD_Pin, GPIO_PIN_RESET);
    spi_delay();
    HAL_GPIO_WritePin(SR_LOAD_GPIO_Port, SR_LOAD_Pin, GPIO_PIN_SET);
    spi_delay();
    
    volatile uint32_t sr1 = 0;
    volatile uint32_t sr2 = 0;
    
    for(int i = 0;i < 16; i++) {
      sr1 <<= 1;
      sr2 <<= 1;
      if(HAL_GPIO_ReadPin(SR_Q_GPIO_Port, SR_Q_Pin) == GPIO_PIN_SET)
        sr1 |= 1;
      if(HAL_GPIO_ReadPin(SR_SER_GPIO_Port, SR_SER_Pin) == GPIO_PIN_SET)
        sr2 |= 1;
      spi_delay();    
      HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_SET);
      spi_delay();
      HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
    }
    
    keys = 0;
    
    if(!(sr1 & (1<<12)))
      keys |= (1<<0);

    if(!(sr1 & (1<<15)))
      keys |= (1<<1);

    if(!(sr1 & (1<<13)))
      keys |= (1<<2);
    
    if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
      keys |= (1<<3);
    if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
      keys |= (1<<4);
    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
      keys |= (1<<5);
    
    if(!(sr2 & (1<<12)))
      keys |= (1<<6);

    if(!(sr2 & (1<<15)))
      keys |= (1<<7);

    if(!(sr2 & (1<<13)))
      keys |= (1<<8);

    if(!(sr2 & (1<<4)))
      keys |= (1<<9);

    if(!(sr2 & (1<<7)))
      keys |= (1<<10);

    if(!(sr2 & (1<<5)))
      keys |= (1<<11);
    
    if(keys != keys_last)
    {
       keyboardhid.mod = 0x00;
       memset(keyboardhid.keys, 0, sizeof(keyboardhid.keys));
       unsigned int key_index = 0;
       uint16_t mask = 1;
       for(unsigned int i = 0; i < 12; i++) {
         if(key_index >= 6)
           break;
         if(keys & mask) {
            keyboardhid.keys[key_index] = 0x3a+i;
            key_index++;
         }
         
         mask <<= 1;
       }
             
        USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboardhid, sizeof (keyboardhid));
      
    }
    
    keys_last = keys;
    
   
    HAL_Delay (10);

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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SR_LOAD_Pin|SR_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SR_LOAD_Pin SR_CLK_Pin */
  GPIO_InitStruct.Pin = SR_LOAD_Pin|SR_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SR_Q_Pin */
  GPIO_InitStruct.Pin = SR_Q_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SR_Q_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY0_Pin */
  GPIO_InitStruct.Pin = KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY2_Pin KEY1_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin|KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SR_SER_Pin */
  GPIO_InitStruct.Pin = SR_SER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SR_SER_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
