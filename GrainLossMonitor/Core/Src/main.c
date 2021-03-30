/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SCALE (3.3 / 4095)
#define NUMBER_OF_CONVERSION 1
#define FILTER_LENGTH 20
#define SAMPLE_LENGTH (200 * NUMBER_OF_CONVERSION)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

float movingAvgFilter(int actualSample);

float stddev(float data[], int len);
float mean(float data[], int len);
void thresholding(float y[], int signals[], int lag, float threshold, float influence);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int contAmostras = 0;
int cont = 0;
int cont2 = 0;
//static long previousTime = 0;
uint8_t adcDataReady;
uint32_t adcDataDMA[NUMBER_OF_CONVERSION];
float adcAvgData = 0;
float adcData;
//float adcDataSamples[SAMPLE_LENGTH];
float adcAvgData2 = 0;
float adcAvgData3 = 0;
float contFiltered;
uint8_t Cmd_End[3] = {0xFF, 0xFF, 0xFF};

int signal[SAMPLE_LENGTH];

void SendToProgressBar(char *obj, uint16_t value)
{
  char buf[30];
  int len = sprintf(buf, "%s=%u", obj, value);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
  HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

void SendToGauge(char *obj, uint16_t value)
{
  char buf[30];

  value += 315;
  if (value >= 360)
    value = value - 360;

  int len = sprintf(buf, "%s=%u", obj, value);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
  HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}
void SendToWave(char *obj, uint16_t value)
{
  char buf[30];
  int len = sprintf(buf, "%s,%u", obj, value);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
  HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void thresholding(float y[], int signals[], int lag, float threshold, float influence)
{
  memset(signals, 0, sizeof(float) * SAMPLE_LENGTH);
  float filteredY[SAMPLE_LENGTH];
  memcpy(filteredY, y, sizeof(float) * SAMPLE_LENGTH);
  float avgFilter[SAMPLE_LENGTH];
  float stdFilter[SAMPLE_LENGTH];

  avgFilter[lag - 1] = mean(y, lag);
  stdFilter[lag - 1] = stddev(y, lag);

  for (int i = lag; i < SAMPLE_LENGTH; i++)
  {
    if (fabsf(y[i] - avgFilter[i - 1]) > threshold * stdFilter[i - 1])
    {
      if (y[i] > avgFilter[i - 1])
      {
        signals[i] = 1;
      }
      else
      {
        signals[i] = -1;
      }
      filteredY[i] = influence * y[i] + (1 - influence) * filteredY[i - 1];
    }
    else
    {
      signals[i] = 0;
    }
    avgFilter[i] = mean(filteredY + i - lag, lag);
    stdFilter[i] = stddev(filteredY + i - lag, lag);
  }
}

float mean(float data[], int len)
{
  float sum = 0.0, mean = 0.0;

  int i;
  for (i = 0; i < len; ++i)
  {
    sum += data[i];
  }

  mean = sum / len;
  return mean;
}

float stddev(float data[], int len)
{
  float the_mean = mean(data, len);
  float standardDeviation = 0.0;

  int i;
  for (i = 0; i < len; ++i)
  {
    standardDeviation += pow(data[i] - the_mean, 2);
  }

  return sqrt(standardDeviation / len);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  float ant = 0;
  int i = 0;
  int lag = 8;
  float threshold = 4;
  float influence = 0.6;
  float adcDataSamples[SAMPLE_LENGTH];
  //float y[SAMPLE_LENGTH] = {1, 1, 1.1, 1, 0.9, 1, 1, 1.1, 1, 0.9, 1, 1.1, 1, 1, 0.9, 1, 1, 1.1, 1, 1, 1, 1, 1.1, 0.9, 1, 1.1, 1, 1, 0.9, 1, 1.1, 1, 1, 1.1, 1, 0.8, 0.9, 1, 1.2, 0.9, 1, 1, 1.1, 1.2, 1, 1.5, 1, 3, 2, 5, 3, 2, 1, 1, 1, 0.9, 1, 1, 3, 2.6, 4, 3, 3.2, 2, 1, 1, 0.8, 4, 4, 2, 2.5, 1, 1, 1};

  HAL_ADCEx_Calibration_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



    do
    {
      //HAL_ADC_Start_DMA(&hadc1, adcDataDMA, NUMBER_OF_CONVERSION);
      //HAL_ADC_Start_IT(&hadc1);

    	HAL_ADC_Start(&hadc1);
    	  HAL_ADC_PollForConversion(&hadc1, 1);
    	  if(HAL_ADC_GetValue(&hadc1) > 80){
    		  adcDataSamples[contAmostras] = HAL_ADC_GetValue(&hadc1);
    	  }
    	  else{
    		  adcDataSamples[contAmostras] = 0;
    	  }

    	contAmostras++;
    	HAL_Delay(4);

    } while (contAmostras < SAMPLE_LENGTH);

    contAmostras = 0;

    thresholding(adcDataSamples, signal, lag, threshold, influence);

    for (i = 0; i < SAMPLE_LENGTH; i++)
    {
      if (signal[i] == 0 && signal[i + 1] == 1)
      {
        cont++;
      }
      /*if (signal[i] == 0 && signal[i + 1] == -1)
      {
        cont2++;
      }
      if(adcDataSamples[i] > 300){
          	int top = 0;
      }*/
    }



    /*if ((HAL_GetTick() - previousTime) >= 1000)
    {
      previousTime = HAL_GetTick();*/

      contFiltered = movingAvgFilter(cont*10);

      adcAvgData = map(contFiltered, 0, 60, 0, 100);
      adcAvgData2 = map(contFiltered, 0, 60, 0, 180);
      adcAvgData3 = map(contFiltered, 0, 60, 0, 270);

      SendToProgressBar("j0.val", (int)adcAvgData);
      SendToProgressBar("j1.val", (int)adcAvgData);
      SendToWave("add 2,0", (int)adcAvgData2);

      if (adcAvgData3 != ant)
      {
        ant = adcAvgData3;

        SendToGauge("z0.val", adcAvgData3);
      }
      cont = 0;
    //}

    // if (cont > 0)
    // {
    //   cont = 0;
    // }
    // cont = 0;

    //HAL_ADC_Start_DMA(&hadc1, adcData, NUMBER_OF_CONVERSION);
    //SendToProgressBar("j1.val", 20);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

float AvgWave = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  int i;
  for (i = 0; i < NUMBER_OF_CONVERSION; i++)
  {
    //adcDataSamples[contAmostras + i] = adcDataDMA[i];
    //adcDataSamples[contAmostras+i] = map(adcDataDMA[i], 0, 4095, 0, 10);
  }

  contAmostras = contAmostras + NUMBER_OF_CONVERSION;

  /*float aux = 0;
  int i = 0;
  for(i=0; i < NUMBER_OF_CONVERSION; i++){
	  aux += adcData[i];
  }
  adcAvgData = aux/(float)NUMBER_OF_CONVERSION;*/

  //adcAvgData = adcData[0];
  /*
  if (adcAvgData > 2000)
  {
    cont++;
  }

  if ((HAL_GetTick() - previousTime) >= 1000)
  {
    previousTime = HAL_GetTick();

    adcAvgData2 = map(cont, 0, 5000, 0, 150);
    adcAvgData = map(cont, 0, 5000, 0, 100);

    SendToProgressBar("j0.val", (int)adcAvgData);
    SendToWave("add 2,0", (int)adcAvgData2);

    cont = 0;
  }*/
}

float movingAvgFilter(int actualSample)
{
  static int i;
  static long accSum;
  static int filterVector[FILTER_LENGTH];

  accSum = accSum - filterVector[i] + actualSample;

  filterVector[i] = actualSample;

  if (++i >= FILTER_LENGTH)
    i = 0;

  return (float)accSum / (float)FILTER_LENGTH;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
