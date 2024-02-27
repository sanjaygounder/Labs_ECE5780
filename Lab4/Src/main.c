/**
 *
 * Brandon Mouser
 * U0962682
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
void _Error_Handler(char *file, int line);
volatile uint8_t receivedChar; // global variable holding the received characters
volatile int newData = 0;
volatile int count = 0; // count of characters (after 2 we will reset this variable)

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void transmitCharacter(char);
void transmitString(char *);
void USART3_IRQHandler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  HAL_Init();           // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); // Configure the system clock

  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
  // Set up a configuration struct to pass to the initialization function
  GPIO_InitTypeDef initStr = {GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6,7,8 & PC9
                                  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high (orange)

  // **** Alternate Function **** //
  // Set the selected pins into alternate function mode and program the correct alternate function number into the GPIO AFR registers
  // PC4 = USART3_TX (AF1) -- transmitting
  // PC5 = USART3_RX (AF1) -- receiving
  // Clear PC4/5 MODER4/5 and then set to alternate (10)
  GPIOC->MODER &= ~(1 << 8);  // clear PC4 bit
  GPIOC->MODER &= ~(1 << 9);  // clear PC4 bit
  GPIOC->MODER |= (1 << 9);   // set to alternate (10)
  GPIOC->MODER &= ~(1 << 10); // clear PC5 bit
  GPIOC->MODER &= ~(1 << 11); // clear PC5 bit
  GPIOC->MODER |= (1 << 11);  // set to alternate (10)
  // Assign AF1 to PC4 and PC5 -- 0001
  GPIOC->AFR[0] |= (1 << 16);  // AFSEL4 bit 16
  GPIOC->AFR[0] &= ~(1 << 17); // AFSEL4 bit 17 (clear)
  GPIOC->AFR[0] &= ~(1 << 18); // AFSEL4 bit 18 (clear)
  GPIOC->AFR[0] &= ~(1 << 19); // AFSEL4 bit 19 (clear)
  GPIOC->AFR[0] |= (1 << 20);  // AFSEL5 bit 20
  GPIOC->AFR[0] &= ~(1 << 21); // AFSEL5 bit 21 (clear)
  GPIOC->AFR[0] &= ~(1 << 22); // AFSEL5 bit 22 (clear)
  GPIOC->AFR[0] &= ~(1 << 23); // AFSEL5 bit 23 (clear)

  // **** Initialize the USART **** //
  // Enable the USART clock in the RCC
  RCC->APB1ENR = RCC_APB1ENR_USART3EN;
  // Set the Baud rate for communication to be 9600 bits/second.
  USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600;
  // Enable transmitter and receiver hardware
  USART3->CR1 |= (1 << 3); // Enable transmitter
  USART3->CR1 |= (1 << 2); // Enable receiver
  USART3->CR1 |= (1 << 5); // Enable RXNE interrupt
  USART3->CR1 |= (1 << 0); // Enable USART
  // **** Initialize the USART **** //

  //*****Enable and Set Priority of the USART3 Interrupt*****************//
  // Enable the selected EXTI interrupt that references USART3
  NVIC_EnableIRQ(USART3_4_IRQn);
  // Set the priority for the interrupt to 0
  NVIC_SetPriority(USART3_4_IRQn, 0);
  // transmitString("CMD? \0");

  while (1)
  {
    while (!(USART3->ISR & USART_ISR_RXNE)){}
    // ******** 4.2 General Configuration ******** //
    if ((newData == 1))
    {
      uint16_t LED_case = 0; // LED to update
      switch (receivedChar)
      {
      case 'r':
        LED_case = GPIO_PIN_6;
        transmitCharacter('r');
        break;
      case 'b':
        LED_case = GPIO_PIN_7;
        transmitCharacter('b');
        break;
      case 'o':
        LED_case = GPIO_PIN_8;
        transmitCharacter('o');
        break;
      case 'g':
        LED_case = GPIO_PIN_9;
        transmitCharacter('g');
        break;
      case '0':
        HAL_GPIO_WritePin(GPIOC, LED_case, GPIO_PIN_RESET);
        transmitCharacter('0');
        break;
      case '1':
        HAL_GPIO_WritePin(GPIOC, LED_case, GPIO_PIN_SET);
        transmitCharacter('1');
        break;
      case '2':
        HAL_GPIO_TogglePin(GPIOC, LED_case);
        transmitCharacter('2');
      default:
        transmitString("Error: Input needs to be 2 characters with correct criteria of r,b,o,g and 0-2\0");
        break;
      }
      newData = 0;
      count++;
      if (count >= 2)
        count = 0;
    }


    // ******** 4.2 General Configuration ******** //
    // uint8_t received_char = USART3->RDR;
    // switch (received_char)
    // {
    // case 'r':
    //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    //   transmitCharacter('r');
    //   break;
    // default:
    //   break;
    // }
    // transmitString(receivedChar);




  }
}

/**
 * USART3 Interrupt Handler
 */
void USART3_IRQHandler()
{
  // receivedChar = (char)(USART3->RDR & 0xFF); // set global variable and mask it to only 2 characters
  receivedChar = USART3->RDR;
  newData = 1;                  // Indicate new data is ready to be processed
}

/**
 *  Transmits a single character on the USART
 */
void transmitCharacter(char c)
{
  // Check and wait on the USART status flag that indicates the transmit register is empty
  while (!(USART3->ISR & USART_ISR_TXE))
  {
  }
  // Write the character into the transmit data register
  USART3->TDR = c;
}

/**
 *  Transmits a string on the USART
 */
void transmitString(char *s)
{
  for (int i = 0; s[i] != '\0'; i++)
  {
    transmitCharacter(s[i]);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

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
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
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
