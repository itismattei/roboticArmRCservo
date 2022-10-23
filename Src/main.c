
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body per scheda NUCLEO-F722ZE
  ******************************************************************************
  **Il programma permette di controllare un braccio robotico a 6 gradi
  * di liberta' tramite joystick, di memorizzare e ripetere spostamenti, di colloquiare con
  * l'unita' centrale e muovere il braccio su proprio comando.
  *
  * Esso usa 6 PWM tipologia RC per controllare i servomotori di spostamento,
  * sui pin compatibili Arduino D9, D10, D11 (rispettivamente PD15, PD14, PA7, posti sul canale 4 e 3 PWM
  * del Timer 4 e  canale 2  del PWM sul timer 3) e D6, D5, D3 (rispettivamente PE9, PE11, PE13,
  * canali 1, 2 , 3 del PWM posto su Timer1)
  *
  * 2 sensori A/D di posizione, 2 accelerometri su bus I2C (canale 1, pin PB8 e PB9),
  * compatibili con piedinatura Arduino, due segnali da rispettivi  potenziometri per misura dei joystick.
  * I canali A/D sono compatibii Arduino con numerazione A0-A5 e sono mappati sui rispetti pin:
  * PA3 	-> ADC3_IN3
  * PC0 	-> ADC3_IN10
  * PC3 	-> ADC3_IN13
  * PF3 	-> ADC3_IN9
  * PF5 	-> ADC3_IN15
  * PF10 	-> ADC3_IN8
  *
  * Il TICK e' stabilito a 10ms e generato da TIMER5.
  *
  * Sono previste due UART, la n.3 comunicante con PC tramite interfaccia USB presente in ST.LINK
  * la seconda, n.4 tramite pin RX e TX (PA1 e PA0 rispettivamente) e la terza tramite
  * i pin RX e TX (rispettivamente PG9 e PG14) che corrispondono ai pin RX e TX della piedinatura di
  * Arduino.
  *
  *
  * 12/02/19: test dei 6 canali PWM con periodo a 20ms. Possono essere usati per comandare i serviRC
  * collegati al braccio. Possono essere usati anche per controllare n.6 ESC per motori drone.
  *
  * 20/02/19: impostata la uart3 ad interruzione in ricezione e verificato il funzionamento
  *
  * 19/03/19: verificato il funzionamento dei convertitori AD compatibili arduino
  *
  * 23/10/22: ripreso lo sviluppo
  *
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <servomotoreRC.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define  DIMBUFF  		64
char READ_BUFF[DIMBUFF];
volatile uint8_t WPTR = 0, RPTR = 0;
uint32_t adc[6], buffer[6], temperature;  // define variables
float vsense = 3.3/1023;
bool ADupdate = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
/*static*/ void MX_TIM3_Init(void);
/*static*/ void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);

void MX_TIM1_Init(void);
void MX_I2C1_Init(void);
static void MX_ADC3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART6_UART_Init(void);

/// usato per generare la base dei tempi dello RTOS
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void loop(void);
void setup(void);
                                
///vettore che raccoglie le informazioni sui delta e numero dei servi RC
/// la dimensione e' 6 ed e' definito in servomotoreRC.c
extern servoRC RC[];
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
void _Error_Handler(void);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/// variabile di conteggio del TICK del timer di sistema
volatile uint32_t TICK = 0, MS100 = 0, S1 = 0;
#define		TIC500		50
extern char versione[];
/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
  CPU_CACHE_Enable();
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /// stringa di benvenuto e versione sw
  printf("\n6DOF Robot Arm Control\n");
  printf("%s", versione);
  printf("\n********************\n");

  /// INIZIALIZZAZIONI STRUTTURE DATI SERVI RC
  /// I PWM SONO ANCORA SPENTI
  //  inizializza i 6 motori servoRC posizionando il braccio circai n posizione vertivcale, cioe'
  //  a meta' della cinematica dello snodo
 initRC(RC);
 printf("\n\nInizializzati i servi\n");
 printf("PWM spenti \n");
 //! Due lampeggi dei tre led
 TICK = 0;
 while (TICK < 50);
 HAL_GPIO_TogglePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin);
 while (TICK < 100);
 HAL_GPIO_TogglePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin);
 while (TICK < 150);
 HAL_GPIO_TogglePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin);
 printf("Accendo i PWM e posiziono i motori al centro!\n");
 for (int i = 0; i < 6; i++)
	 //! le strutture dati sono impostate e i PWM vengono avviati
	 goRC(&RC[i]);
 ////HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 printf("Sblocca il dispositivo premendo il tasto rosso!\n");
 //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET); //sblocco con pulsante rosso
 	 printf("Sbloccato.\nAttendo i messaggi da seriale!\n");

// while(1){
//	int a;
//	a = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10);
//	if (a == 1 )
//		break;
// }
 /// settaggio
 setup();

 while (1){
	 /// simile ad Arduino
 /* USER CODE END WHILE */
	  //HAL_ADC_Start_DMA(&hadc3, buffer, 5);
  /* USER CODE BEGIN 3 */
  //HAL_Delay (1000);
//	  GPIO_PinState STATO;
//	  while (1)
//		  STATO = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
	  loop();
  }
  /* USER CODE END 3 */

}


/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 6;
  //hadc3.Init.NbrOfConversion = 5;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;	/// PA3 -> A0 -> ADC_CH3
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_10;	/// PC0 -> A1 -> ADC_CH10
  sConfig.Rank = ADC_REGULAR_RANK_2;
  //sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_13;	/// PC3 -> A2 -> ADC_CH13
  sConfig.Rank = ADC_REGULAR_RANK_3;
  //sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler();
  }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
	sConfig.Channel = ADC_CHANNEL_9;  /// PF3 -> A3 -> ADC_CH9
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK){
	  _Error_Handler();
	}

/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
*/
	sConfig.Channel = ADC_CHANNEL_15;  /// PF5 -> A4 -> ADC_CH15
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK){
		_Error_Handler();
	}

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
	sConfig.Channel = ADC_CHANNEL_8;   /// PF10 -> A5 -> ADC_CH8
	sConfig.Rank = ADC_REGULAR_RANK_6;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK){
		_Error_Handler();
	}



}



/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler();
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler();
  }

}



/* TIM5 init function */
static void MX_TIM5_Init(void)
{

	//Calcola il prescaler per ottenere un tic ogni 100 us (10000 Hz)
	uint32_t uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;
	//! system core clock = 216MHz
	/// uwPrescalerValue = 216 / 20000 - 1 = 10799
	//! uwPrescalerValue = 216 / 80000 - 1 = 2699


  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = uwPrescalerValue;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  /// conta 100 eventi di periodo 100us, in modo da generare un reload ogni 10ms
  htim5.Init.Period = 100 - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler();
  }

  ////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////
  ////// DA AGGIUNGERE A "MANO"
  /// abilitazione ad emettere interruzioni
  if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
  {
    /* Starting Error */
	 _Error_Handler();
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler();
  }

}

/* USART3 init function
 * Questa seriale e' collegata allo ST-LINK e permette di ricevere o inviare messaggi
 * tramite connessione USB al programmatore della scheda.
 * La ricezione avviene da interruzione di programma */

static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler();
  }

  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();


  /// Configura il pin PA7 che e' D12 E SERVE ad abilitare la potenza per i servo RC
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */


  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6  */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5  */
  /// lo uso per un tasto configurato con pull_down
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12  */
  /// lo uso per un tasto configurato con pull_down
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);


  /*Configure GPIO pins : PF10  */
  /// lo uso per un tasto configurato con pull_down
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);



  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void CPU_CACHE_Enable(void){
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ADupdate = true;
	for (int i = 0; i < 6; i++)
	{
	   adc[i] = buffer[i];  // store the values in adc[]
	}

        temperature = (((adc[2]*vsense)-.76)/.0025)+25;
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @}
  */

/************************ *****END OF FILE****/
