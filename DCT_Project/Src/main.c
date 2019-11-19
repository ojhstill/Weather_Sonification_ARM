/* Includes ------------------------------------------------------------------*/
#include "../Inc/main.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#include "../Drivers/Audio/stm32f4_discovery_audio.h"


/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;
HCD_HandleTypeDef hhcd_USB_OTG_FS;

#define PBSIZE 4096 // Default: 4096
#define SINELOOKUPSIZE 1024 // Default: 1024
#define PI 3.141592653589793

// Defining 32-bit variables for ADC conversions.
uint32_t ADC1conv;
uint32_t ADC2conv;

int16_t PlayBuff[PBSIZE];
int16_t SineBuff[PBSIZE];
int32_t nextSample;
uint16_t buffer_offset = 0;
enum eNoteStatus { ready, going, finish }  noteStatus = ready;
enum eBufferStatus { empty, finished, firstHalfReq, firstHalfDone, secondHalfReq, secondHalfDone }  bufferStatus = empty;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/*
float hexToTemp(int hexValue)
{
 
	float Vout = hexValue / (4095/3.3);
	
	float R = (10000 * (Vout / 3.3))/(1 - (Vout/3.3));
	
	float Temp = (1160399.8/(3892 + 298.15*log(R/10000)));
	
	return Temp - 273.15;
}
*/

void setupADC1(void)
{
	RCC->APB2ENR|= RCC_APB2ENR_ADC1EN; // Enable clock for ADC1 and ADC2.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC.
	
	GPIOC->MODER |= 0x00000300; // Set PC4 to Analogue Mode.
	ADC1->SQR3 |= 0x0000000e; // Set ADC1 to look at input 14.
	ADC1->CR2  |= ADC_CR2_ADON; //0x00000001; // Enable ADC from the Control Register CR2.
	ADC1->CR1  |= ADC_CR1_DISCEN; // Set ADC1 to discontinuous conversion mode.
	ADC1->CR2  |= ADC_CR2_EOCS; // Enable EOC flag for ADC1.
	
	// ********************************************************************************************
}

void setupADC2(void)
{
	RCC->APB2ENR|= RCC_APB2ENR_ADC2EN; // Enable clock for ADC2.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC.
	
	GPIOC->MODER |= 0x00000c00; // Set PC5 to Analogue Mode.
	ADC2->SQR3 |= 0x0000000f; // Set ADC2 to look at input 15.
	ADC2->CR2  |= ADC_CR2_ADON; //0x00000001; // Enable ADC from the Control Register CR2.
	ADC2->CR1  |= ADC_CR1_DISCEN; // Set ADC2 to discontinuous conversion mode.
	ADC2->CR2  |= ADC_CR2_EOCS; // Enable EOC flag for ADC2.
	
	// ********************************************************************************************
}

void setupGPIO(void)
{
	GPIOE->MODER |= 0x00001540;
	GPIOB->MODER |= 0x40014400;
}

/*
float smoothValue(float currentValue, float targetValue) {
	
	if (currentValue > targetValue)
		currentValue -= 0.01;
	
	else if (currentValue < targetValue)
		currentValue += 0.01;
	
	return currentValue;
}
*/

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
  // Initialise the audio driver.
  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_44K);

  // Set up the sine look-up table.
  for (int i = 0; i <= SINELOOKUPSIZE; i++) {
    float q = 32760 * sin(i * 2.0 * PI / SINELOOKUPSIZE);
    SineBuff[i] = (int16_t)q;
  }

  // Silence the buffer.
  for(int i=0; i <= PBSIZE; i++)
    PlayBuff[i] = 0;
  
  // Start the audio driver play routine.
  BSP_AUDIO_OUT_Play((uint16_t*)&PlayBuff[0], PBSIZE);
	
  // Set up the fundamental.
  float noteFrequency = 440.0f; //(A4 - 440Hz).

  // Define all LUT increments base on musical intervals.
  float phaseInc1 = SINELOOKUPSIZE * noteFrequency / AUDIO_FREQUENCY_44K;						// Fundamental.
  float phaseInc2 = SINELOOKUPSIZE * (noteFrequency * 5/4) / AUDIO_FREQUENCY_44K;		// Major third.
  float phaseInc3 = SINELOOKUPSIZE * (noteFrequency * 3/2) / AUDIO_FREQUENCY_44K;		// Perfect fifth.
  float phaseInc4 = SINELOOKUPSIZE * (noteFrequency * 15/8) / AUDIO_FREQUENCY_44K;	// Major seventh.
  float phaseInc5 = SINELOOKUPSIZE * (noteFrequency * 9/4) / AUDIO_FREQUENCY_44K;		// Octave + major second.

  // Set all phases to 0.
  float currentPhase1 = 0.0;
  float currentPhase2 = 0.0;
  float currentPhase3 = 0.0;
  float currentPhase4 = 0.0;
  float currentPhase5 = 0.0;

  noteStatus = going;

  // Setup ADC1 and ADC2.
  setupADC1();
  setupADC2();
	
	// Setup GPIO pins.
	setupGPIO();
	
  while (1)
  {
		HAL_Delay(1);
		
		// Convert analogue signal from LDR.
		ADC1->CR2 |= ADC_CR2_SWSTART;		// Simultaneous Start Conversion
		while (!ADC_CSR_EOC1) {
		} // Wait until EOC
		ADC1conv = ADC1->DR;
		
		// Convert analogue signal from thermistor.
		ADC2->CR2 |= ADC_CR2_SWSTART;		// Simultaneous Start Conversion
		while (!ADC_CSR_EOC2) {
		} // Wait until EOC
		ADC2conv = ADC2->DR;
		
		// If there's an active request to fill half of the buffer, then do so:
		uint32_t startFill = 0, endFill = 0;
		if (bufferStatus == firstHalfReq) {
			startFill = 0;
			endFill = PBSIZE / 2;
			bufferStatus = firstHalfDone;
		}
		else if (bufferStatus == secondHalfReq) {
			startFill = PBSIZE / 2;
			endFill = PBSIZE;
			bufferStatus = secondHalfDone;
		}
		
 		if (startFill != endFill) {
			
			// Fill audio buffer.
			for (int i = startFill; i < endFill; i += 2) {

				// Increment phase positions in wavetable for all tones.
				currentPhase1 += phaseInc1;
				currentPhase2 += phaseInc2;
				currentPhase3 += phaseInc3;
				currentPhase4 += phaseInc4;
				currentPhase5 += phaseInc5;
				
				// Reset phase position if greater than wavetable.
				if (currentPhase1 > SINELOOKUPSIZE)
					currentPhase1 -= SINELOOKUPSIZE;
				if (currentPhase2 > SINELOOKUPSIZE)
					currentPhase2 -= SINELOOKUPSIZE;
				if (currentPhase3 > SINELOOKUPSIZE)
					currentPhase3 -= SINELOOKUPSIZE;
				if (currentPhase4 > SINELOOKUPSIZE)
					currentPhase4 -= SINELOOKUPSIZE;
				if (currentPhase5 > SINELOOKUPSIZE)
					currentPhase5 -= SINELOOKUPSIZE;
				
				// TERMISTOR HARMONIC SONIFICATION:

        // Retrieve ADC1 value for the LDR.
				float thermistorValue = ADC2conv;
		
				// Add more sine tones to fundamental dependent on the current temperature.
				if (thermistorValue > 2048) { // Fundamental at 25C or less.
					nextSample = (int32_t)(SineBuff[(uint16_t)(currentPhase1)]);
					
					
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
				}
				else if (thermistorValue > 1959) { // Add major third at 27C.
					nextSample = (int32_t)((SineBuff[(uint16_t)(currentPhase1)]) 
					+ (((2048 - thermistorValue)/89) * SineBuff[(uint16_t)(currentPhase2)]));
					
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
				}
				else if (thermistorValue > 1871) { // Add perfect fifth at 29C.
					nextSample = (int32_t)((SineBuff[(uint16_t)(currentPhase1)]) 
					+ (SineBuff[(uint16_t)(currentPhase2)]) 
					+ (((1959 - thermistorValue)/88) * SineBuff[(uint16_t)(currentPhase3)]));	
					
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
				}
				else if (thermistorValue > 1785) { // Add major seventh at 31C.
					nextSample = (int32_t)((SineBuff[(uint16_t)(currentPhase1)]) 
					+ (SineBuff[(uint16_t)(currentPhase2)]) 
					+ (SineBuff[(uint16_t)(currentPhase3)]) 
					+ (((1871 - thermistorValue)/86) * SineBuff[(uint16_t)(currentPhase4)]));
					
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);				
				}
				else if (thermistorValue > 1702) { // Add octave + major second at 33C.
					nextSample = (int32_t)((SineBuff[(uint16_t)(currentPhase1)]) 
					+ (SineBuff[(uint16_t)(currentPhase2)]) 
					+ (SineBuff[(uint16_t)(currentPhase3)]) 
					+ (SineBuff[(uint16_t)(currentPhase4)]) 
					+ (((1785 - thermistorValue)/83) * SineBuff[(uint16_t)(currentPhase5)]));
					
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);				
				}
				else { 
					nextSample = (int32_t)((SineBuff[(uint16_t)(currentPhase1)]) 
					+ (SineBuff[(uint16_t)(currentPhase2)]) 
					+ (SineBuff[(uint16_t)(currentPhase3)]) 
					+ (SineBuff[(uint16_t)(currentPhase4)]) 
					+ (SineBuff[(uint16_t)(currentPhase5)]));
				}

				// LDR AMPLITUDE SONIFICATION:
				
				// Retrieve ADC1 value for the LDR.
				float LDRValue = (float) ADC1conv / 4095;
				
				if (ADC1conv > 3723) 
				{ 
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				} 
				else if (ADC1conv > 2482) 
				{ 
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        }
				else if (ADC1conv > 1241) 
				{ 
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				}
			  else if (ADC1conv > 600)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				}
				else {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				}

				// Adjust the amplitude of the audio sample base on the LDR value.
				nextSample = ((1 - LDRValue) * (nextSample / 5));
				
				// Play sample.
  			PlayBuff[i] = (uint16_t) nextSample;
				PlayBuff[i + 1] = (uint16_t) nextSample;
			}
		}
  } // ... end of while loop.
	
}


// Audio callbacks:
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	if (noteStatus == going) bufferStatus = firstHalfReq;
}
void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)PlayBuff, PBSIZE);
	if (noteStatus == going) bufferStatus = secondHalfReq;
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PB10   ------> I2S2_CK
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
