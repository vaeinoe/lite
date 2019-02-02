#include "main.h"
#include <string.h>

/* If defined, send additional debug JSON messages via UART */
#define DEBUG

/* Buffer size for storing samples. Adjust this according to Timer 6
 * prescaler + period, and the measurement interval requirement. */
#define ADC_BUFF_SIZE 1000

/* Master clock is 16MHz, prescale by 1600 and count to 10 -> 1KHz rate */
#define TIM6_PRESCALE 1600
#define TIM6_PERIOD 10

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);

uint16_t adcBuff[ADC_BUFF_SIZE];
uint16_t adcBuffIdx = -1;
uint32_t cycle = 0;
uint8_t readyToSend = 0;

char strBuff[256];

char *msgAdcError = "{ \"status\": \"ADC error\" }\r\n";
char *msgReady    = "{ \"status\": \"ready\", \"sysclkSource\": %lu, "
					"\"HCLK\": %lu, \"PCLK1\": %lu, \"PCLK2\": %lu }\r\n";
char *msgData     = "{ \"ambientLightLevel\": %u, \"measurementCycle\": %lu }\r\n";


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1) {
		adcBuffIdx = (adcBuffIdx + 1) % ADC_BUFF_SIZE;
		adcBuff[adcBuffIdx] = (uint16_t) HAL_ADC_GetValue(hadc);

		if (adcBuffIdx == ADC_BUFF_SIZE - 1) {
			readyToSend = 1;
		}
	}
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
	HAL_UART_Transmit(&huart2, (uint8_t*)msgAdcError, strlen(msgAdcError), 0xFFFF);
}

/* Sends some basic system / clock data for debugging purposes */
void transmitSystemData() {
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	uint32_t pFLatency;
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

	sprintf(strBuff, msgReady, RCC_ClkInitStruct.SYSCLKSource,
		  HAL_RCC_GetHCLKFreq(), HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq());
	HAL_UART_Transmit(&huart2, (uint8_t*)strBuff, strlen(strBuff), 0xFFFF);
}

/* Calculates the average of measurements in buffer, transmits the value via UART */
inline void transmitCurrentValue() {
  uint32_t avg = 0;

  for (int i = 0; i < ADC_BUFF_SIZE; i++) {
    avg += adcBuff[i];
  }

  avg /= ADC_BUFF_SIZE;
  uint16_t avg16 = (uint16_t) avg;

  sprintf(strBuff, msgData, avg16, cycle);
  HAL_UART_Transmit(&huart2, (uint8_t*)strBuff, strlen(strBuff), 0xFFFF);
}

int main(void)
{
  /* Reset peripherals, Initialize the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();

  for (int i = 0; i < ADC_BUFF_SIZE; i++) {
    adcBuff[i] = 0;
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
	Error_Handler();
  }

  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start_IT(&htim6);

#ifdef DEBUG
  transmitSystemData();
#endif

  while (1)
  {
    /* Polling: when a measurement buffer is marked as full, send it via UART */
	if (readyToSend == 1) {
		#ifdef DEBUG
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		#endif

		cycle++;
		transmitCurrentValue();
		readyToSend = 0;

		#ifdef DEBUG
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		#endif
	}
  }
}

/* Configure system clock and all relevant peripheral clocks at 16MHz */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* The board doesn't have HSE, disable it - use the PLL with HSI/2 input
   * and 4x multiplier instead. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize the CPU, AHB and APB bus clocks using the PLL source */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize the async ADC clock (in case we use it later...) */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Use synchronous clock with divider 4, 12 bits.
   * Trigger regular, non-continuous conversions via Tim6 */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* Subtract 1 from prescale and period to get correct values from constants.
   * Set up triggering for the ADC. */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = TIM6_PRESCALE - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = TIM6_PERIOD - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 (LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* Error handler TODO */
void Error_Handler(void)
{
}

#ifdef  USE_FULL_ASSERT
void assert_failed(char *file, uint32_t line)
{ 
}
#endif
