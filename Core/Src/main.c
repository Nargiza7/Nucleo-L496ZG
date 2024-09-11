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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart_rx;

SD_HandleTypeDef hsd1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
/* USER CODE BEGIN PV */

//uint8_t buffer[32];
//const char *ledOnMessage = "LED ON\r\n";
//const char *ledOffMessage = "LED OFF\r\n";
//const char *errorMessage = "ERROR\r\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t RxData[4096];
int isSizeRxed = 0;
uint32_t size = 0;

//int counthalf = 0;
//int countfull = 0;

//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
//{
//	counthalf++;
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (isSizeRxed == 0)
//	{
//		size = ((RxData[0]-48)*1000) + ((RxData[1]-48)*100) + ((RxData[2]-48)*10) + ((RxData[3]-48));
//		isSizeRxed = 1;
//		HAL_UART_Receive_DMA(&hlpuart1, RxData, size);
//	}
//	else if (isSizeRxed == 1)
//	{
//		isSizeRxed = 0;
//		HAL_UART_Receive_DMA(&hlpuart1, RxData, 4); //TODO //FIXME
//	}

//	countfull++;
//	HAL_UART_Receive_DMA(&hlpuart1, RxData, 10);
//}

void WriteToSDCard(void) {
    FIL myFile;
    FRESULT res;
    UINT bytesWritten;

    if (f_mount(&SDFatFS, SDPath, 1) == FR_OK) {

        res = f_open(&myFile, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
        if (res == FR_OK) {

            res = f_write(&myFile, "Merhaba STM32!", 14, &bytesWritten);
            if (res == FR_OK) {

                f_close(&myFile);
                HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Dosya başarıyla yazıldı!\r\n", 26, HAL_MAX_DELAY);
            }
        }

        f_mount(0, "", 0);
    } else {
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)"SD Kart mount edilemedi.\r\n", 27, HAL_MAX_DELAY);
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
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_LPUART1_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&hlpuart1, RxData, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  ledTimeOut = HAL_GetTick();


//HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, buffer, 32);


//  HAL_StatusTypeDef status;

//  uint8_t rxBuffer[10];
//  const char *ledOnMessage = "LED ON\r\n";
//  const char *ledOffMessage = "LED OFF\r\n";
//  const char *errorMessage = "ERROR\r\n";


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 // HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	//  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
	//  HAL_Delay(250);
	//  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
	//  HAL_Delay(750);

	//  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, GPIO_PIN_SET);

//	  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
//	  {
//
//      if (buttonPressed == 0)
//      {
//
//      HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
//
//      HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
//      HAL_Delay(500);
//
//      buttonPressed = 1;
//      }
//	  }
//
//	  else
//	  {
//        if (buttonPressed == 1)
//        {
//          buttonPressed = 0;
//
//          HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, GPIO_PIN_RESET);
//        }
//	  }


//	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
//
//	  if(ledTimeOut + 500 < HAL_GetTick()){
//
//		  ledTimeOut = HAL_GetTick();
//
//		  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
//	  }


//	  status = HAL_UART_Receive(&hlpuart1, buffer, 1, 5000);
//
//	  if (HAL_OK == status || HAL_TIMEOUT == status) {
//			if (buffer[0] == 'K') {
//				HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
//			}
//			if (buffer[0] == 'M') {
//				HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
//			}
//	  }
//
//	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
//	  	  HAL_Delay(500);
//	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
//	  	  HAL_Delay(500);
//
//
//	  buffer[0] = 0;
//

//	  if (HAL_UART_Receive(&hlpuart1, rxBuffer, 1, 1000) == HAL_OK)
//	  {
//		 if (rxBuffer[0] == '1')
//		 {
//			 HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
//			 HAL_UART_Transmit(&hlpuart1, (uint8_t *)ledOnMessage, strlen(ledOnMessage), HAL_MAX_DELAY);
//		 }
//		 else if (rxBuffer[0] == '0')
//		 {
//			 HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
//			 HAL_UART_Transmit(&hlpuart1, (uint8_t *)ledOffMessage, strlen(ledOffMessage), HAL_MAX_DELAY);
//		 }
//		 else
//		 {
//			 HAL_UART_Transmit(&hlpuart1, (uint8_t *)errorMessage, strlen(errorMessage), HAL_MAX_DELAY);
//		 }
//      }

	  HAL_Delay(1000);
	  WriteToSDCard();  // SD karta yazma işlemi
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_0;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;


  hsd1.hdmarx->ChannelIndex = (((uint32_t)hsd1.hdmarx->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
  DMA2_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hsd1.hdmarx->ChannelIndex & 0x1cU));


  hsd1.hdmatx->ChannelIndex = (((uint32_t)hsd1.hdmatx->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
  DMA2_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hsd1.hdmatx->ChannelIndex & 0x1cU));
  DMA2_CSELR->CSELR |= (uint32_t)(hsd1.hdmatx->Init.Request << (hsd1.hdmatx->ChannelIndex & 0x1cU));


  /* Write block(s) in DMA transfer mode */
  if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}


uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;

  /* Read block(s) in DMA transfer mode */

  hsd1.hdmatx->ChannelIndex = (((uint32_t)hsd1.hdmatx->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
  DMA2_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hsd1.hdmatx->ChannelIndex & 0x1cU));


  hsd1.hdmarx->ChannelIndex = (((uint32_t)hsd1.hdmarx->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2U;
  DMA2_CSELR->CSELR &= ~(DMA_CSELR_C1S << (hsd1.hdmarx->ChannelIndex & 0x1cU));
  DMA2_CSELR->CSELR |= (uint32_t)(hsd1.hdmarx->Init.Request << (hsd1.hdmarx->ChannelIndex & 0x1cU));

  if (HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }



  return sd_state;
}









volatile uint32_t ledTimeOut = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

//	if(GPIO_Pin == B1_Pin) {
//		HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
//	}

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//	 if (huart->Instance == LPUART1)
//			    {
//
//		 buffer[Size] = '\0';
//
//			        if (strstr((char *)buffer, "RED") != NULL)
//			        {
//			            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
//			            HAL_UART_Transmit(&hlpuart1, (uint8_t *)ledOnMessage, strlen(ledOnMessage), HAL_MAX_DELAY);
//			        }
//			        if (strstr((char *)buffer, "BLUE") != NULL)
//			        {
//			            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//			            HAL_UART_Transmit(&hlpuart1, (uint8_t *)ledOnMessage, strlen(ledOnMessage), HAL_MAX_DELAY);
//			        }
//			        if (strstr((char *)buffer, "CLEAR") != NULL)
//			        {
//			            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//			            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//
//			            HAL_UART_Transmit(&hlpuart1, (uint8_t *)errorMessage, strlen(errorMessage), HAL_MAX_DELAY);
//			        }
//
//			        HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, buffer, 32);
//			    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

//	 if (huart->Instance == LPUART1)
//	    {
//	        if (buffer[0] == 'M')
//	        {
//	            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	        }
//	        if (buffer[0] == 'K')
//	        {
//	            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	        }
//
//	        HAL_UART_Receive_IT(&hlpuart1, buffer, 1);
//	    }

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
