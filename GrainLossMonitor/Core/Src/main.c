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

#include "peakDetection.h"
#include "nextion.h"
#include "flash_page.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define FILTER_LENGTH 20
#define END_PAG63 0x08018C00 // Max (pag 63): 0x08018C00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

float movingAvgFilter(int actualSample);//Filtro media movel ADC1
float movingAvgFilter2(int actualSample);//Filtro media movel ADC2

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int contAmostras = 0;

uint16_t slider1 = 0;          //Valor do slider de calibracao 1
uint16_t slider1ant = 0;       //Ultimo valor do slider de calibracao 1
uint16_t slider2 = 0;          //Valor do slider de calibracao 2
uint16_t slider2ant = 0;       //Ultimo valor do slider de calibracao 2
uint16_t filter_length = 1;    //Valor do slider do filtro media movel
uint16_t filter_lengthAnt = 0; //Ultimo valor do slider do filtro media movel

static int filterVector[50];  //Media movel buffer
static long accSum;           //Media movel soma
static int filterVector2[50]; //Media movel buffer ADC2
static long accSum2;          //Media movel soma ADC2

char Rxbuf[10]; //Buffer recebimento serial

float adcDataSamples[SAMPLE_LENGTH];  //Global para usar DMA
float adcDataSamples2[SAMPLE_LENGTH]; //Global para usar DMA

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
  return ch;
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
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  int signal[SAMPLE_LENGTH]; //Vetor com a os sinais de pico
  int peakQtd = 0;           //Quantidade de picos detectados

  int signal2[SAMPLE_LENGTH]; //Vetor com a os sinais de pico ADC 2
  int peakQtd2 = 0;           //Quantidade de picos detectados ADC 2

  float progressBarValue1 = 0; //Valor convertido enviado para o display
  float waveValue1 = 0;        //Valor convertido enviado para o display
  float contFiltered;          //Contagem de pulsos com media movel

  float progressBarValue2 = 0; //Valor convertido enviado para o display ADC 2
  float waveValue2 = 0;        //Valor convertido enviado para o display ADC 2
  float contFiltered2;         //Contagem de pulsos com media movel ADC 2

  HAL_ADCEx_Calibration_Start(&hadc1);

  FLASH_le_16bits(END_PAG63, &slider1);           //Le da flash o valor da ultima calibracao
  FLASH_le_16bits(END_PAG63 + 2, &slider2);       //Le da flash o valor da ultima calibracao
  FLASH_le_16bits(END_PAG63 + 4, &filter_length); //Le da flash o valor da ultima calibracao
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    do
    {

      HAL_ADC_Start(&hadc1);
      HAL_ADC_Start(&hadc2);
      HAL_ADC_PollForConversion(&hadc1, 1);
      HAL_ADC_PollForConversion(&hadc2, 1);
      if (HAL_ADC_GetValue(&hadc1) > slider2 * 2) //Tamanho minimo do sinal
      {
        adcDataSamples[contAmostras] = HAL_ADC_GetValue(&hadc1);
      }
      else
      {
        adcDataSamples[contAmostras] = 0;
      }

      if (HAL_ADC_GetValue(&hadc2) > slider2 * 2) //Tamanho minimo do sinal
      {
        adcDataSamples2[contAmostras] = HAL_ADC_GetValue(&hadc2);
      }
      else
      {
        adcDataSamples2[contAmostras] = 0;
      }

      contAmostras++;
      HAL_Delay(4);

    } while (contAmostras < SAMPLE_LENGTH);
    contAmostras = 0;

    thresholding(adcDataSamples, signal);
    peakQtd = contPeak(signal);
    thresholding(adcDataSamples2, signal2);
    peakQtd2 = contPeak(signal2);

    contFiltered = movingAvgFilter(peakQtd * 10);
    contFiltered2 = movingAvgFilter2(peakQtd2 * 10);

    if (slider1 != 0) //Erro se fizer um map de 0
    {
      progressBarValue1 = map(contFiltered, 0, slider1, 0, 100);
      waveValue1 = map(contFiltered, 0, slider1, 0, 180);

      if (progressBarValue1 > 100)
        progressBarValue1 = 100;
      if (waveValue1 > 180)
        waveValue1 = 180;

      progressBarValue2 = map(contFiltered2, 0, slider1, 0, 100);
      waveValue2 = map(contFiltered2, 0, slider1, 0, 180);

      if (progressBarValue2 > 100)
        progressBarValue2 = 100;
      if (waveValue2 > 180)
        waveValue2 = 180;
    }

    SendToProgressBar("j0.val", (int)progressBarValue1);
    SendToProgressBar("j1.val", (int)progressBarValue2);
    SendToWave("add 2,0", (int)waveValue1);
    SendToWave("add 3,0", (int)waveValue2);

    peakQtd = 0;
    peakQtd2 = 0;

    GetPage(); //Receber o numero da pagina atual
    HAL_UART_Receive(&huart1, (uint8_t *)Rxbuf, 5, 100);
    if ((Rxbuf[1] == 0x66 && Rxbuf[2] == 0x03 && Rxbuf[4] == 0xFF)) //Pagina de calibracão
    {

      GetVal("h0"); //Recebe o valor do slider1
      HAL_UART_Receive(&huart1, (uint8_t *)Rxbuf, 8, 100);
      if (Rxbuf[1] == 0x71 && Rxbuf[7] == 0xFF)
      {
        slider1 = (uint16_t)Rxbuf[2];

        if (slider1 == 0) //Primeira vez que entrou na tela de calibragem
        {
          uint16_t slider1Flash;
          uint16_t slider2Flash;
          uint16_t filter_lengthFlash;
          FLASH_le_16bits(END_PAG63, &slider1Flash);
          FLASH_le_16bits(END_PAG63 + 2, &slider2Flash);
          FLASH_le_16bits(END_PAG63 + 4, &filter_lengthFlash);
          SendToSlider("h0", slider1Flash); //Seta os sliders para o valor que estava gravado na FLASH
          SendText("n0", slider1Flash);
          SendToSlider("h1", slider2Flash);
          SendText("n1", slider2Flash);
          SendToSlider("h2", filter_lengthFlash);
          SendText("n2", filter_lengthFlash);
        }

        if (slider1 != slider1ant && slider1 != 0)
        { //Somente escreve quando o slider foi atualizado e nao é a primeira vez que entra na tela
          FLASH_apaga(END_PAG63, 1);
          FLASH_escreve_16bits(END_PAG63, &slider1);
          FLASH_escreve_16bits(END_PAG63 + 2, &slider2);
          FLASH_escreve_16bits(END_PAG63 + 4, &filter_length);
        }

        slider1ant = slider1;
      }
      memset(Rxbuf, 0, sizeof(Rxbuf));

      GetVal("h1"); //Recebe o valor do slider2
      HAL_UART_Receive(&huart1, (uint8_t *)Rxbuf, 8, 100);
      if (Rxbuf[1] == 0x71 && Rxbuf[7] == 0xFF)
      {
        slider2 = (uint16_t)Rxbuf[2];

        if (slider2 != slider2ant && slider2 != 0)
        { //Somente escreve quando o slider foi atualizado e nao é a primeira vez que entra na tela
          FLASH_apaga(END_PAG63, 1);
          FLASH_escreve_16bits(END_PAG63, &slider1);
          FLASH_escreve_16bits(END_PAG63 + 2, &slider2);
          FLASH_escreve_16bits(END_PAG63 + 4, &filter_length);
        }

        slider2ant = slider2;
      }
    }
    GetVal("h2"); //Recebe o valor do slider2
    HAL_UART_Receive(&huart1, (uint8_t *)Rxbuf, 8, 100);
    if (Rxbuf[1] == 0x71 && Rxbuf[7] == 0xFF)
    {
      filter_length = (uint16_t)Rxbuf[2];

      if (filter_length != filter_lengthAnt && filter_length != 0)
      { //Somente escreve quando o slider foi atualizado e nao é a primeira vez que entra na tela
        FLASH_apaga(END_PAG63, 1);
        FLASH_escreve_16bits(END_PAG63, &slider1);
        FLASH_escreve_16bits(END_PAG63 + 2, &slider2);
        FLASH_escreve_16bits(END_PAG63 + 4, &filter_length);
        memset(filterVector, 0, sizeof(filterVector)); //Zera o buffer da media movel
        accSum = 0;//Zera soma da media movel
        memset(filterVector2, 0, sizeof(filterVector2)); //Zera o buffer da media movel ADC2
        accSum2 = 0;//Zera soma da media movel ADC2
      }

      filter_lengthAnt = filter_length;
    }
  }

  memset(Rxbuf, 0, sizeof(Rxbuf)); //Zera o buffer

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
/* USER CODE END 3 */

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
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

float movingAvgFilter(int actualSample)
{
  static int i;

  accSum = accSum - filterVector[i] + actualSample;

  filterVector[i] = actualSample;

  if (++i >= filter_length)
    i = 0;

  return (float)accSum / (float)filter_length;
}

float movingAvgFilter2(int actualSample2)
{
  static int j;

  accSum2 = accSum2 - filterVector2[j] + actualSample2;

  filterVector2[j] = actualSample2;

  if (++j >= filter_length)
    j = 0;

  return (float)accSum2 / (float)filter_length;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
