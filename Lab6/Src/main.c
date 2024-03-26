/**
 *
 * Sanjay Gounder
 * u1144264
 *
******************************************************************************
* File Name          : main.c
* Description        : Main program body
******************************************************************************
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
*
* COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
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
#include "stm32f0xx_hal.h"
#include "stm32f072xb.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();               // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config();     // Configure the system clock

  __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable the GPIOB clock in the RCC
  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
  __HAL_RCC_ADC1_CLK_ENABLE(); // Enable the ADC1 clock in the RCC
  // Set up a configuration struct to pass to the initialization function (GPIOC w/out PC0) -- LEDs
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6,7,8 & PC9 (LEDs)
  // Set up a configuration struct to pass to the initialization function (PB0)
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high (orange)
  

  // ******** Initializing ADC ******** //
  // 1) Set the desired operating mode, data resolution, and trigger source
  // 2) Start the ADC calibration
  // 3) Wait for the hardware to signal that the calibration has completed.
  // 4) Set the peripheral enable.
  // 5) Wait until the ADC ready flag is set
  // 6) Start the ADC conversion. 

  // ******** Initializing DAC ******** //
  // 1) Set the trigger source for the channel/output update
  // 2) Enable the channel used for output.

  // Analog functions of GPIO pins
  // PC0 --> Additional Function: ADC_IN10
  // PA4 --> Additional Function: DAC_OUT1

  // Enable GPIOA/C in RCC peripheral
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // ******** 6.1 Measuring a Potentiometer With the ADC ******** //
  // Select a GPIO pin to use as the ADC input / no pull-up down resistors
  GPIOC->MODER |= (1 << 0); // PC0 set to analog mode (11)      
  GPIOC->MODER |= (1 << 1); // PC0 set to analog mode (11)
  GPIOC-> PUPDR &= ~(1 < 0); // no pull-up down (00)
  GPIOC-> PUPDR &= ~(1 < 1); // no pull-up down (00)
  // Enable the ADC/DAC in the RCC peripheral
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;

  // Configure the ADC to 8-bit resolution, continuous conversion mode, hardware triggers disabled (software trigger only).
  ADC1->CFGR1 |= (1 << 4); // Set it to 8-bit resolution (10)
  ADC1->CFGR1 |= (1 << 13); // Set it to continuous conversion mode (1)
  // ADC1->CFGR1 &= ~(1 << 11); // Hardware trigger disabled (00)
  // ADC1->CFGR1 &= ~(1 << 10); // Hardware trigger disabled (00)
  // Select/enable the input pin’s channel for ADC conversion (ADC Channel Selection Register (ADC_CHSELR))
  ADC1->CHSELR |= (1 << 10); //PC0 (ADC_IN10) selected for ADC conversion

  // Set to wait conversion
  ADC1->CFGR1 |= (1 << 14);
  // Perform a self-calibration and enable
  ADC1->CR &= ~(1); // Set ADEN to off
  ADC1->CFGR1 &= ~(1); // Set DMAEN to off

  // Set ADCAL = 1
  ADC1->CR |= (1 << 31);

  // Wait until ADCAL = 0
  while ((ADC1->CR & (1 <<31)) == (1 << 31)) {}
  ADC1->CR |= 1; // Enable ADC

  ADC1->CR |= (1 << 2); // Start ADC

  // ******** 6.2 Generating Waveforms with the DAC. ******** //
  // Set PA4 to analog (10)
  GPIOA->MODER = (1 << 8);
  GPIOA->MODER = (1 << 9);
  GPIOC-> PUPDR &= ~(1 < 16); // no pull-up down (00)
  GPIOC-> PUPDR &= ~(1 < 17); // no pull-up down (00)
	
  // Set the used DAC channel to software trigger mode.
  DAC1->CR = 0; //Clear
	DAC1->CR |= (7 << 3); //SW Trigger Mode
	DAC1->SWTRIGR = 0;
	DAC1->SWTRIGR |= (1 << 0); //SW Channel 1 Trigger

  // Enable the used DAC channel.
  DAC1->CR |= (1 << 0);

  // Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
	232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	// Triangle Wave: 8-bit, 32 samples/cycle
	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
	190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
	// Sawtooth Wave: 8-bit, 32 samples/cycle
	const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
	111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
	// Square Wave: 8-bit, 32 samples/cycle
	const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
	254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  // In the main application loop, read the ADC data register and turn on/off LEDs depending on the value
  int16_t threshold = 0;
  while (1) {		

    // ******** 6.2 Generating Waveforms with the DAC. ******** //
    for(int i = 0; i < 32; i++){
				DAC1->DHR8R1 = sine_table[i]; //Change for triange/saw
        if (i == 31){
          i = 0;
        }
				HAL_Delay(1);
    }
    
    // ******** 6.1 Measuring a Potentiometer With the ADC ******** //
    // while((ADC1->ISR & ADC_ISR_EOS) == 0){} // wait for end of sequence
		// // Reset
		// ADC1->ISR |= ADC_ISR_EOS;

    // if (ADC1->DR < 60){ 
    //   GPIOC->ODR |= (1 << 6); // Red high only
    //   GPIOC->ODR &= ~(1 << 7) |~(1 << 8)|~(1 << 9) ; // Turn off other LEDs
		// }
		// else if (ADC1->DR < 120){
    //   GPIOC->ODR |= (1 << 6) | (1<<7); // Red, Blue high
    //   GPIOC->ODR &= ~(1 << 8) |~(1 << 9); // Turn off green orange LED
		// }
		// else if (ADC1->DR < 180){
    //   GPIOC->ODR |= (1 << 6) | (1<<7)| (1 << 8); // Red, Green, Blue high
    //   GPIOC->ODR &= ~(1 << 9); // Turn off orange LED
		// }
		// else {
		// 	GPIOC->ODR |= (1 << 6) | (1<<7)| (1 << 8) | (1 << 9); // All LEDs high
		// }
    // ADC1->CFGR1 ^= ADC_CFGR1_SCANDIR;

		HAL_Delay(100);
	}

}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
