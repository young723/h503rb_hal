
#include "bsp_hardware.h"
#include <stdlib.h>
#include <string.h>

//ADC_HandleTypeDef hadc1;
extern I3C_HandleTypeDef hi3c1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
//SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
#ifdef BSP_TIM6_SUPPORT
TIM_HandleTypeDef htim6;
#endif
#ifdef BSP_TIM7_SUPPORT
TIM_HandleTypeDef htim7;
#endif
#ifdef BSP_UART2_SUPPORT
UART_HandleTypeDef huart2;
#endif
UART_HandleTypeDef huart3;

#if 0
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
#endif
static bsp_tim_func_t bsp_tim;
static bsp_irq_func_t bsp_irq;
static bsp_usart_rx_t uart2_rx;
static bsp_usart_rx_t uart3_rx;
static bsp_key_func_t bsp_key;
static bsp_ibi_func_t bsp_ibi;

static evb_port_t bsp_port = {INTERFACE_USER_SEL, INTERFACE_USER_SEL, INTERFACE_USER_SEL, EVB_SPI_MODE0};

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 250;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}


void MX_I2C1_Init(uint32_t clk)
{
	hi2c1.Instance = I2C1;
	if(clk == 100*1000)
		hi2c1.Init.Timing = 0x60808CD3;	// 100K
	else if(clk == 400*1000)
		hi2c1.Init.Timing = 0x10C043E5;	// 400k
	else if(clk == 1000*1000)
		hi2c1.Init.Timing = 0x00C035A6; // 1000k
	else
		hi2c1.Init.Timing = 0x10C043E5; // 400k
		
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	
	HAL_I2C_DeInit(&hi2c1);
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

void MX_I2C2_Init(uint32_t clk)
{
	hi2c2.Instance = I2C2;
	if(clk == 100*1000)
		hi2c2.Init.Timing = 0x60808CD3; // 100K
	else if(clk == 400*1000)
		hi2c2.Init.Timing = 0x10C043E5; // 400k
	else if(clk == 1000*1000)
		hi2c2.Init.Timing = 0x00C035A6; // 1000k
	else
		hi2c2.Init.Timing = 0x10C043E5; // 400k

	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_DeInit(&hi2c2);
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */
}

#if 0
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
#endif


/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(uint32_t Period)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (SystemCoreClock/1000) - 1;	//(SystemCoreClock/10000) - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = Period - 1 ;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(uint32_t Period)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (SystemCoreClock/1000) - 1;	//(SystemCoreClock/1000) - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = Period - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

static void MX_TIM3_Init(uint32_t Period)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (SystemCoreClock/1000) - 1;	//(SystemCoreClock/1000) - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = Period - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

#ifdef BSP_TIM6_SUPPORT
static void MX_TIM6_Init(uint32_t Period)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = (SystemCoreClock/1000) - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = Period - 1;
  htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim6, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void TIM6_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}
#endif

#ifdef BSP_TIM7_SUPPORT
/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(uint32_t Period)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = (SystemCoreClock/1000) - 1;	// 10k
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = Period - 1;
  htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim7, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}


void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim7);
}
#endif

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
#ifdef BSP_UART2_SUPPORT
void MX_USART2_UART_Init(void)
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
	if(HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
#if 0
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_usart2_rx);
	__HAL_LINKDMA(huart2,hdmarx,hdma_usart2_rx);

	hdma_usart2_tx.Instance = DMA1_Stream6;
	hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_tx.Init.Mode = DMA_NORMAL;
	hdma_usart2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
	hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_usart2_tx);
	__HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);
#endif
	/* USER CODE BEGIN USART2_Init 2 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);//接收中断
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//空闲中断
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_DisableIRQ(USART2_IRQn);

}

void MX_USART2_UART_DeInit(void)
{
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_UART_DeInit(&huart2);
}
#endif
/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;	//921600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);//接收中断
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//空闲中断
	/* USER CODE BEGIN USART3_Init 2 */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_DisableIRQ(USART3_IRQn);
}

void MX_USART3_UART_DeInit(void)
{
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	HAL_UART_DeInit(&huart3);
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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_8;	// RST
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}



static void MX_IRQ_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	// KEY INT
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// INT1/INT2
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	// INT1/INT2
	HAL_NVIC_SetPriority(EXTI7_IRQn, 1, 2);
	HAL_NVIC_DisableIRQ(EXTI7_IRQn);	
	HAL_NVIC_SetPriority(EXTI8_IRQn, 1, 2);
	HAL_NVIC_DisableIRQ(EXTI8_IRQn);
	// KEY INT
	HAL_NVIC_SetPriority(EXTI13_IRQn, 1, 2);
	HAL_NVIC_DisableIRQ(EXTI13_IRQn);
}


void evb_tim_handle(void)
{
	if(bsp_tim.tim1_flag)
	{
		if(bsp_tim.ingore_count)
		{
			bsp_tim.ingore_count--;
			return;
		}
		bsp_tim.tim1_flag = 0;
		if(bsp_tim.tim1_func)
			bsp_tim.tim1_func();
	}
	if(bsp_tim.tim2_flag)
	{
		if(bsp_tim.ingore_count)
		{
			bsp_tim.ingore_count--;
			return;
		}
		bsp_tim.tim2_flag = 0;
		if(bsp_tim.tim2_func)
			bsp_tim.tim2_func();
	}	
	if(bsp_tim.tim3_flag)
	{
		if(bsp_tim.ingore_count)
		{
			bsp_tim.ingore_count--;
			return;
		}
		bsp_tim.tim3_flag = 0;
		if(bsp_tim.tim3_func)
			bsp_tim.tim3_func();
	}
#ifdef BSP_TIM6_SUPPORT
	if(bsp_tim.tim6_flag)
	{
		if(bsp_tim.ingore_count)
		{
			bsp_tim.ingore_count--;
			return;
		}
		bsp_tim.tim6_flag = 0;
		if(bsp_tim.tim6_func)
			bsp_tim.tim6_func();
	}
#endif
#ifdef BSP_TIM7_SUPPORT
	if(bsp_tim.tim7_flag)
	{
		if(bsp_tim.ingore_count)
		{
			bsp_tim.ingore_count--;
			return;
		}
		bsp_tim.tim7_flag = 0;
		if(bsp_tim.tim7_func)
			bsp_tim.tim7_func();
	}
#endif
}

void evb_setup_timer(TIM_TypeDef * tim_id, int_callback func, uint16_t ms, FunctionalState enable)
{
	TIM_HandleTypeDef *tim_ptr;

	ms = ms*480/100;
	if(tim_id == TIM1)
	{
		tim_ptr = &htim1;
		bsp_tim.tim1_flag = 0;
		bsp_tim.tim1_func = func;
		MX_TIM1_Init(ms);
	}
	else if(tim_id == TIM2)
	{
		tim_ptr = &htim2;
		bsp_tim.ingore_count = 1;
		bsp_tim.tim2_flag = 0;
		bsp_tim.tim2_func = func;
		MX_TIM2_Init(ms);
	}
	else if(tim_id == TIM3)
	{
		tim_ptr = &htim3;
		bsp_tim.tim3_flag = 0;
		bsp_tim.tim3_func = func;
		MX_TIM3_Init(ms);
	}
#ifdef BSP_TIM6_SUPPORT
	else if(tim_id == TIM6)
	{
		tim_ptr = &htim6;
		bsp_tim.tim6_flag = 0;
		bsp_tim.tim6_func = func;
		MX_TIM6_Init(ms);
	}
#endif
#ifdef BSP_TIM7_SUPPORT
	else if(tim_id == TIM7)
	{
		tim_ptr = &htim7;
		bsp_tim.tim7_flag = 0;
		bsp_tim.tim7_func = func;
		MX_TIM7_Init(ms);
	}
#endif

	if(enable != DISABLE)
	{
		HAL_TIM_Base_Start_IT(tim_ptr);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(tim_ptr);
	}
}


void evb_setup_irq(int int_type, int_callback func, FunctionalState enable)
{
	if(int_type & QST_NUCLEOPC7_SENSOR_INT1)
	{
		bsp_irq.irq1_func = func;
		
		if(enable)
			HAL_NVIC_EnableIRQ(EXTI7_IRQn);
		else
			HAL_NVIC_DisableIRQ(EXTI7_IRQn);
	}
	if(int_type & QST_NUCLEOPA8_SENSOR_INT2)
	{
		bsp_irq.irq2_func = func;
		
		if(enable)
			HAL_NVIC_EnableIRQ(EXTI8_IRQn);
		else
			HAL_NVIC_DisableIRQ(EXTI8_IRQn);
	}
}

extern void evb_irq_handle(void)
{
	if(bsp_irq.irq1_flag)
	{
		bsp_irq.irq1_flag = 0;
		if(bsp_irq.irq1_func)
			bsp_irq.irq1_func();
	}
	if(bsp_irq.irq2_flag)
	{
		bsp_irq.irq2_flag = 0;
		if(bsp_irq.irq2_func)
			bsp_irq.irq2_func();
	}
}


void qst_delay_ms(unsigned int delay)
{
//	HAL_ResumeTick();
	HAL_Delay(delay);
//	HAL_SuspendTick();
}

void qst_delay_us(unsigned int delay)
{
	volatile unsigned int count = delay;
	while(count > 0)
	{
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		count--;
	}
}

void SysTick_Enable(unsigned char enable)
{
	if(enable)
		HAL_ResumeTick();
	else
		HAL_SuspendTick();
}

void bsp_event_clear(void)
{
	evb_setup_irq(QST_EVB_INT1, NULL, DISABLE);
	evb_setup_irq(QST_EVB_INT2, NULL, DISABLE);
	//evb_setup_user_key1(NULL, DISABLE);

	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
#ifdef BSP_TIM6_SUPPORT
	HAL_TIM_Base_Stop_IT(&htim6);
#endif
#ifdef BSP_TIM7_SUPPORT
	HAL_TIM_Base_Stop_IT(&htim7);
#endif
}

void bsp_power_pin_set(int on)
{
	if(on)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	qst_delay_ms(200);
}

void bsp_port_i2c_init(evb_interface_e type)
{
	if((bsp_port.i2c_type < INTERFACE_I2C_SW) || (bsp_port.i2c_type > INTERFACE_I2C_HW_1M))
	{
		bsp_port.i2c_type = type;
		if(type == INTERFACE_I2C_SW)
		{
			qst_logi("\r\nuser select I2C-SW\r\n");
			i2c_sw_gpio_config(0);
		}
		else if(type == INTERFACE_I2C_HW)
		{
			qst_logi("\r\nuser select I2C-HW400K\r\n");
			MX_I2C1_Init(400*1000);
			MX_I2C2_Init(400*1000);
		}
		else if(type == INTERFACE_I2C_HW_1M)
		{
			qst_logi("\r\nuser select I2C-HW1000K\r\n");
			MX_I2C1_Init(1000*1000);
			MX_I2C2_Init(1000*1000);
		}
		else
		{
			bsp_port.i2c_type = INTERFACE_USER_SEL;
			qst_logi("\r\nI2C init error!\r\n");
			return;
		}
	}
}

void bsp_port_i2c_deinit(void)
{
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_DeInit(&hi2c2);
	bsp_port.i2c_type = INTERFACE_USER_SEL;
}

void bsp_port_i3c_init(evb_interface_e type)
{
#ifdef HAL_I3C_MODULE_ENABLED
	int ret = 0;

	if((bsp_port.i3c_type < INTERFACE_I3C_4M) || (bsp_port.i3c_type > INTERFACE_I3C_12_5M))
	{	
I3C_INIT:
		MX_I3C1_Init(0x1e, 0x1e);	// 0x13
		qst_delay_ms(100);
		HAL_I3C_DeInit(&hi3c1);
		qst_delay_ms(100);
		bsp_port.i3c_type = type;
		if(type == INTERFACE_I3C_4M)
		{
			qst_logi("\r\nuser select I3C-4.0M\r\n");
			MX_I3C1_Init(0x1e, 0x1e);
		}
		else if(type == INTERFACE_I3C_6_25M)
		{
			qst_logi("\r\nuser select I3C-6.25M\r\n");
			MX_I3C1_Init(0x13, 0x13);
		}
		else if(type == INTERFACE_I3C_10M)
		{
			qst_logi("\r\nuser select I3C-10.0M\r\n");
			MX_I3C1_Init(0x0c, 0x0c);
		}
		else if(type == INTERFACE_I3C_12_5M)
		{
			qst_logi("\r\nuser select I3C-12.5M\r\n");
	//		MX_I3C1_Init(0x09, 0x0b);	// FAIL
	//		MX_I3C1_Init(0x0a, 0x0b);	// OK
	//		MX_I3C1_Init(0x0b, 0x0b);	// OK
	//		MX_I3C1_Init(0x0c, 0x0c);	// OK
	//		MX_I3C1_Init(0x0d, 0x0d);	// OK
	//		MX_I3C1_Init(0x0e, 0x0e);	// OK
			MX_I3C1_Init(0x09, 0x09);	// 12.5M
		}
		else
		{
			bsp_port.i3c_type = INTERFACE_USER_SEL;
			qst_logi("\r\nI3C init error!\r\n");
			return;
		}
		qst_delay_ms(100);
		ret = qst_evb_enry_i3c();
		if(ret == 0)
			goto I3C_INIT;
	}
//	qst_delay_ms(300);
#endif
}

void bsp_port_i3c_deinit(void)
{
#ifdef HAL_I3C_MODULE_ENABLED
	HAL_I3C_DeInit(&hi3c1);
	bsp_port.i3c_type = INTERFACE_USER_SEL;
#endif
}

void bsp_port_spi_init(evb_interface_e type, int mode)
{
#ifdef HAL_SPI_MODULE_ENABLED
	if((bsp_port.spi_type < INTERFACE_SPI_HW4) || (bsp_port.spi_type > INTERFACE_SPI_SW3))
	{
		bsp_port.spi_type = type;
		bsp_port.spi_mode = (evb_spi_mode_e)mode;

		if(type == INTERFACE_SPI_HW4)
		{
			qst_logi("\r\nuser select HW-4WIRE SPI mode[%d]\r\n", mode);
			MX_SPI1_Init(mode);
		}
		else if(type == INTERFACE_SPI_HW3)
		{
			qst_loge("bsp_port_init EVB_INTERFACE_SPI_HW_3 not support!!!\r\n");
		}
		else if(type == INTERFACE_SPI_SW4)
		{
			qst_logi("\r\nuser select SW-4WIRE SPI mode[%d]\r\n", mode);
			spi_sw_init(4, mode);
		}
		else if(type == INTERFACE_SPI_SW3)
		{
			qst_logi("\r\nuser select SW-3WIRE SPI mode[%d]\r\n", mode);
			spi_sw_init(3, mode);
		}
		else
		{
			bsp_port.spi_type = INTERFACE_USER_SEL;
			qst_loge("bsp_port_spi_init no function!!!\r\n");
		}
	}
#endif
}

int bsp_i2c_write_reg(unsigned char slave, unsigned char reg, unsigned char value)
{
	int ret = 0;

	if(bsp_port.i2c_type == INTERFACE_I2C_SW)
	{
		ret = qst_sw_writereg(slave<<1, reg, value);
	}
	else if((bsp_port.i2c_type == INTERFACE_I2C_HW)||(bsp_port.i2c_type == INTERFACE_I2C_HW_1M))
	{
		ret = bsp_i2c1_write(slave<<1, reg, value);
	}
	else
	{
		qst_loge("bsp_i2c_write_reg no function!!!\r\n");
	}

	return ret;
}

int bsp_i2c_read_reg(unsigned char slave, unsigned char reg, uint8_t* buff, unsigned short len)
{
	int ret = 0;

	//qst_logi("interface : %d--%d	", bsp_port.interface, bsp_port.spi_mode);
	if(bsp_port.i2c_type == INTERFACE_I2C_SW)
	{
		ret = qst_sw_readreg(slave<<1, reg, buff, len);
	}
	else if((bsp_port.i2c_type == INTERFACE_I2C_HW)||(bsp_port.i2c_type == INTERFACE_I2C_HW_1M))
	{
		ret = bsp_i2c1_read(slave<<1, reg, buff, len);
	}
	else
	{
		qst_loge("bsp_i2c_read_reg no function!!!\r\n");
	}

	return ret;
}

int bsp_i3c_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
#ifdef HAL_I3C_MODULE_ENABLED
	ret = bsp_i3c_write(reg, value);
#endif
	return ret;
}

int bsp_i3c_read_reg(unsigned char reg, uint8_t* buff, unsigned short len)
{
	int ret = 0;
#ifdef HAL_I3C_MODULE_ENABLED
	ret = bsp_i3c_read(reg, buff, len);
#endif
	return ret;
}

int bsp_spi_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
#ifdef HAL_SPI_MODULE_ENABLED

	if(bsp_port.spi_type == INTERFACE_SPI_HW4)
	{
		ret = qst_hw_spi_write(reg, value);
	}
	else if(bsp_port.spi_type == INTERFACE_SPI_SW4)
	{
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi4_mode0_write(reg, value);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi4_mode3_write(reg, value);
		}
	}
	else if(bsp_port.spi_type == INTERFACE_SPI_SW3)
	{
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi3_mode0_write(reg, value);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi3_mode3_write(reg, value);
		}
	}
#endif
	return ret;
}

int bsp_spi_read_reg(unsigned char reg, uint8_t* buff, unsigned short len)
{
	int ret = 0;
#ifdef HAL_SPI_MODULE_ENABLED
	if(bsp_port.spi_type == INTERFACE_SPI_HW4)
	{
		ret = qst_hw_spi_read(reg, buff, len);
	}
	else if(bsp_port.spi_type == INTERFACE_SPI_SW4)
	{
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi4_mode0_read(reg, buff, len);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi4_mode3_read(reg, buff, len);
		}
	}
	else if(bsp_port.spi_type == INTERFACE_SPI_SW3)
	{
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi3_mode0_read(reg, buff, len);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi3_mode3_read(reg, buff, len);
		}
	}
#endif
	return ret;
}


int bsp_spi_write_ext(unsigned char dummy, unsigned char reg, unsigned char value)
{
	int ret = 0;
#ifdef HAL_SPI_MODULE_ENABLED
	switch(bsp_port.spi_type)
	{
	case INTERFACE_SPI_HW4:
		ret = qst_hw_spi_write(reg, value);
		break;
	case INTERFACE_SPI_SW4:
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi4_mode0_write(reg, value);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi4_mode3_write(reg, value);
		}
		break;
	case INTERFACE_SPI_SW3:
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi3_mode0_write(reg, value);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi3_mode3_write(reg, value);
		}
		break;
	default:
		break;
	}
#endif

	return ret;
}

int bsp_spi_read_ext(unsigned char dummy, unsigned char reg, uint8_t* buff, unsigned short len)
{
	int ret = 0;
#ifdef HAL_SPI_MODULE_ENABLED
	switch(bsp_port.spi_type)
	{
	case INTERFACE_SPI_HW4:
		ret = qst_hw_spi_read(reg, buff, len);
		break;
	case INTERFACE_SPI_SW4:
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi4_mode0_read(reg, buff, len);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi4_mode3_read(reg, buff, len);
		}

		break;
	case INTERFACE_SPI_SW3:
		if(bsp_port.spi_mode == EVB_SPI_MODE0)
		{
			ret = qst_sw_spi3_mode0_read(reg, buff, len);
		}
		else if(bsp_port.spi_mode == EVB_SPI_MODE3)
		{
			ret = qst_sw_spi3_mode3_read(reg, buff, len);
		}
		break;
	default:
		break;
	}
#endif
	return ret;
}

int bsp_i3c_write_ext(unsigned char dummy, unsigned char reg, unsigned char value)
{
	int ret = 0;
#ifdef HAL_I3C_MODULE_ENABLED
	ret = bsp_i3c_write(reg, value);
#endif
	return ret;
}

int bsp_i3c_read_ext(unsigned char dummy, unsigned char reg, uint8_t* buff, unsigned short len)
{
	int ret = 0;
#ifdef HAL_I3C_MODULE_ENABLED
	ret = bsp_i3c_read(reg, buff, len);
#endif
	return ret;
}

void bsp_entry_sel_interface(int *intf)
{
	typedef struct
	{
		evb_interface_e		index;
		char				support;
		char *				info;
	}bsp_interface_info;

	const bsp_interface_info interface_array[INTERFACE_TOTAL] = 
	{
		{INTERFACE_I2C_SW, 		1, 	"I2C-SW200K"},
		{INTERFACE_I2C_HW, 		1, 	"I2C-HW400K"},
		{INTERFACE_I2C_HW_1M, 	1, 	"I2C-HW1.0M"},
		{INTERFACE_I3C_4M, 		1, 	"I3C-4.0M"},
		{INTERFACE_I3C_6_25M, 	1, 	"I3C-6.25M"},
		{INTERFACE_I3C_10M,		1, 	"I3C-10.0M"},
		{INTERFACE_I3C_12_5M, 	1, 	"I3C-12.5M"},
		{INTERFACE_SPI_HW4, 	0, 	"SPI-HW-4WIRE"},
		{INTERFACE_SPI_HW3, 	0, 	"SPI-HW-3WIRE"},
		{INTERFACE_SPI_SW4, 	0, 	"SPI-SW-4WIRE"},
		{INTERFACE_SPI_SW3, 	0, 	"SPI-SW-3WIRE"},
	};

	if(*intf < 0)
	{
		while(1)
		{
			int items = 0;
			qst_logi("Select communication procotol:\r\n");
			
			for(items=0; items<(sizeof(interface_array)/sizeof(interface_array[0])); items++)
			{
				if(interface_array[items].support)
				{				
					qst_logi("[%d]: %s\r\n", interface_array[items].index, interface_array[items].info);
				}
			}

			scanf("%d", intf);
			if((*intf>=INTERFACE_I2C_SW)&&(*intf<INTERFACE_TOTAL))
			{
				if(interface_array[*intf].support)
				{
					break;
				}
			}
			else
			{
				qst_logi("Select communication procotol:%d error!!!\r\n", *intf);
			}
		}
	}

	if((*intf >= INTERFACE_I2C_SW)&&(*intf <= INTERFACE_I2C_HW_1M))
	{
		qst_logi("init i2c %d\r\n", *intf);
		bsp_port_i2c_init((evb_interface_e)(*intf));
	}
	else if((*intf >= INTERFACE_I3C_4M)&&(*intf <= INTERFACE_I3C_12_5M))
	{
		qst_logi("init i3c %d\r\n", *intf);
		bsp_port_i3c_init((*intf));
	}
	else if((*intf >= INTERFACE_SPI_HW4)&&(*intf <= INTERFACE_SPI_SW3))
	{
		qst_logi("init spi %d-%d\r\n", *intf, bsp_port.spi_mode);
		bsp_port_spi_init((evb_interface_e)(*intf), bsp_port.spi_mode);
	}
	else
	{
		qst_logi("Evb can not support this communication procotol!!!\r\n");
	}
}

void bsp_hardware_init(void)
{
	HAL_Init();

	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_ICACHE_Init();	
	MX_GPIO_Init();
#ifdef BSP_UART2_SUPPORT
	MX_USART2_UART_Init();
#endif
	MX_USART3_UART_Init();
	MX_TIM1_Init(100);
	MX_TIM2_Init(100);
	MX_TIM3_Init(100);
#ifdef BSP_TIM6_SUPPORT
	MX_TIM6_Init(100);
#endif
#ifdef BSP_TIM7_SUPPORT
	MX_TIM7_Init(100);
#endif
	/* USER CODE BEGIN 2 */
	MX_IRQ_Init();
	//MX_I3C1_Init();
	//MX_SPI1_Init(0);
	//BSP_LED_Init(LED2);
	//BSP_LED_On(LED2);
	//BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);

	//HAL_TIM_Base_Stop_IT(&htim1);
	//HAL_TIM_Base_Stop_IT(&htim2);
	//HAL_TIM_Base_Stop_IT(&htim3);
	//HAL_TIM_Base_Stop_IT(&htim6);
	//HAL_TIM_Base_Stop_IT(&htim7);
	qst_delay_ms(500);
}

//timer1 5ms interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		bsp_tim.tim1_flag = 1;
	}
	else if(htim->Instance == htim2.Instance)
	{
		bsp_tim.tim2_flag = 1;
	}
	else if(htim->Instance == htim3.Instance)
	{
		bsp_tim.tim3_flag = 1;
	}
#ifdef BSP_TIM6_SUPPORT
	else if(htim->Instance == htim6.Instance)
	{
		bsp_tim.tim6_flag = 1;
	}
#endif
#ifdef BSP_TIM7_SUPPORT
	else if(htim->Instance == htim7.Instance)
	{
		bsp_tim.tim7_flag = 1;
	}
#endif
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7)
		bsp_irq.irq1_flag = 1;
	else if(GPIO_Pin == GPIO_PIN_8)
		bsp_irq.irq2_flag = 1;
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
		bsp_key.key1_flag = 1;
//	else if(GPIO_Pin == GPIO_PIN_6)
//		bsp_irq.irq1_flag = 1;
//	else if(GPIO_Pin == GPIO_PIN_7)
//		bsp_irq.irq2_flag = 1;
}

void evb_setup_uart_rx(USART_TypeDef* USARTx, usart_callback func)
{
	if(USARTx == NULL)
	{
	}
#ifdef BSP_UART2_SUPPORT
	else if(USARTx == USART2)
	{
		MX_USART2_UART_DeInit();
		MX_USART2_UART_Init();
		if(func)
		{
			memset(&uart2_rx, 0, sizeof(uart2_rx));
			uart2_rx.rx_cbk = func;
			HAL_UART_Receive_IT(&huart2, uart2_rx.rx_it, 1);
			HAL_NVIC_EnableIRQ(USART2_IRQn);
		}		
		uart2_rx.rx_delay_count = uart2_rx.rx_cplt_count = 1000;
	}
#endif
	else if(USARTx == USART3)
	{
		MX_USART3_UART_DeInit();
		MX_USART3_UART_Init();
		if(func)
		{
			memset(&uart3_rx, 0, sizeof(uart3_rx));
			uart3_rx.rx_cbk = func;
			HAL_NVIC_EnableIRQ(USART3_IRQn);
		}
		uart3_rx.rx_delay_count = uart3_rx.rx_cplt_count = 1000;
	}
}

void evb_usart_rx_handle(void)
{
	if(uart2_rx.rx_cplt_count)
	{
		uart2_rx.rx_cplt_count--;
		if((uart2_rx.rx_cplt_count == 0)&&(uart2_rx.rx_len))
		{
			if(uart2_rx.rx_cbk)
			{
				uart2_rx.rx_cbk(uart2_rx.rx_buf, uart2_rx.rx_len);
			}
			memset(&uart2_rx.rx_buf, 0, sizeof(uart2_rx.rx_buf));
			uart2_rx.rx_len = 0;
		}
	}
	if( uart3_rx.rx_cplt_count)
	{
		 uart3_rx.rx_cplt_count--;
		if(( uart3_rx.rx_cplt_count == 0)&&( uart3_rx.rx_len))
		{
			if( uart3_rx.rx_cbk)
			{
				 uart3_rx.rx_cbk( uart3_rx.rx_buf,  uart3_rx.rx_len);
			}
			memset(& uart3_rx.rx_buf, 0, sizeof( uart3_rx.rx_buf));
			uart3_rx.rx_len = 0;
		}
	}
}

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Transmit(huart, uart2_rx.rx_it, 1, 100);
	if(huart == &huart2)
	{
		uart2_rx.rx_buf[uart2_rx.rx_len++] = uart2_rx.rx_it[0];
		uart2_rx.rx_len = uart2_rx.rx_len % 32;
		uart2_rx.rx_cplt_count = 1000;
		HAL_UART_Receive_IT(huart, uart2_rx.rx_it, 1);
		//UART_Start_Receive_DMA(huart, uart2_rx.rx_it, 1);
	}
	else if(huart == &huart3)
	{
		uart3_rx.rx_buf[uart3_rx.rx_len++] = uart3_rx.rx_it[0];
		uart3_rx.rx_len = uart3_rx.rx_len % 32;
		uart3_rx.rx_cplt_count = 1000;
		HAL_UART_Receive_IT(huart, uart3_rx.rx_it, 1);
		//UART_Start_Receive_DMA(huart, uart2_rx.rx_it, 1);
	}
}
#endif

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	huart->ErrorCode = HAL_UART_ERROR_NONE;
	if(huart == NULL)
	{
	}
#ifdef BSP_UART2_SUPPORT
	else if(huart == &huart2)
	{
		MX_USART2_UART_DeInit();
	}
#endif
	else if(huart == &huart3)
	{
		MX_USART3_UART_DeInit();
	}
}

#ifdef BSP_UART2_SUPPORT
void USART2_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
	{   
		HAL_UART_Receive(&huart2, uart2_rx.rx_it, 1, 1000);
		__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);

		uart2_rx.rx_buf[uart2_rx.rx_len++] = uart2_rx.rx_it[0];
		uart2_rx.rx_len = uart2_rx.rx_len % 32;
		uart2_rx.rx_cplt_count = 1000;
	}
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		uart2_rx.rx_cplt_count = 5;
	}
}
#endif

void USART3_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
	{   
		HAL_UART_Receive(&huart3, uart3_rx.rx_it, 1, 1000);
		__HAL_UART_CLEAR_FLAG(&huart3,UART_FLAG_RXNE);

		uart3_rx.rx_buf[uart3_rx.rx_len++] = uart3_rx.rx_it[0];
		uart3_rx.rx_len = uart3_rx.rx_len % 32;
		uart3_rx.rx_cplt_count = 1000;	//uart3_rx.rx_delay_count;
	}

	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		uart3_rx.rx_cplt_count = 5;	//uart3_rx.rx_delay_count;
	}
}

void evb_key_handle(void)
{
	if(bsp_key.key1_flag)
	{
		bsp_key.key1_flag = 0;
		if(bsp_key.key1_func)
		{
			bsp_key.key1_func();
		}
	}
	if(bsp_key.key2_flag)
	{
		bsp_key.key2_flag = 0;
		if(bsp_key.key2_func)
		{
			bsp_key.key2_func();
		}
	}
}

void evb_setup_user_key(int id, int_callback func)
{
	if(id == 1)
	{
		HAL_NVIC_EnableIRQ(EXTI13_IRQn);
		bsp_key.key1_func = func;
	}
}


// add by yangzhiqiang
int fputc(int ch, FILE *f)
{
//	int retry = 0;

	HAL_UART_Transmit(&huart3 , (uint8_t *)&ch, 1, 0xFFFF);
//	while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX)
//	{
//		if(retry++ > 1000)
//			return ch;
//	}
	return ch;
}

int fgetc(FILE *f)
{
	uint8_t ch = 0;
	//while(__HAL_USART_GET_FLAG(&huart2, USART_FLAG_RXNE) == RESET);
	HAL_UART_Receive(&huart3, &ch, 1, 0xffff);
	HAL_UART_Transmit(&huart3 , &ch, 1, 0xFFFF);
	return ch;
}

void usart_send_ch(uint8_t ch)
{
	HAL_UART_Transmit(&huart3 , (uint8_t *)&ch, 1, 0xFFFF);
//	while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX)
//	{
//		if(retry++ > 1000)
//			return ch;
//	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	static int error_count = 0;
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		if(error_count++ > 200000)
		{
			printf("Error_Handler!\r\n");
			error_count = 0;
		}
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


