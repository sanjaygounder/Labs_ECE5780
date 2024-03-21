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
void write(char val);
char read();
int16_t readX();
int16_t readY();

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
  __HAL_RCC_I2C1_CLK_ENABLE(); // Enable the I2C2 clock
  // Set up a configuration struct to pass to the initialization function (GPIOC w/out PC0) -- LEDs
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};
  GPIO_InitTypeDef initStr_PC0 = {GPIO_PIN_0 | GPIO_PIN_13 | GPIO_PIN_14};
  HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6,7,8 & PC9 (LEDs)
  HAL_GPIO_Init(GPIOC, &initStr_PC0); // Initialize pins PC0
  // Set up a configuration struct to pass to the initialization function (PB0)
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high (orange)

  // Set PB11 to alternate function mode, open-drain output type, and select I2C2_SDA as its alternate function.
  GPIOB->MODER |= (1 << 23); // PB11 to AF mode (10)
  GPIOB->OTYPER |= (1 << 11); // PB11 to open-drain output (1)
  GPIOB ->AFR[1] |= (1 << 12); // Set it to AF1 (0001)
  GPIOB->AFR[1] &= ~(1 <<13) | ~(1 <<14) | ~(1 << 15); // Set it to AF1 (0001)  

  // Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function
  GPIOB->MODER |= (1 << 27); // PB13 to AF mode (10)
  GPIOB->OTYPER |= (1 << 13); // PB13 to open-drain output(1)
  GPIOB->AFR[1] &= ~(1 << 21) | ~(1 << 23); // Set it to AF5 (0101)
  GPIOB->AFR[1] |= (1 << 20) | (1 << 22); // Set it to AF5 (0101)

  // Set PB14 to output mode, push-pull output type, and initialize/set the pin high.
  GPIOB->MODER |= (1 << 28); // PB14 to ouput mode (01)    
  GPIOB->OTYPER &= ~(1 << 14); // PB14 to push-pull output type (clear)   
  GPIOB->ODR |= (1 << 14); // PB14 pin high                            

  // Set PC0 to output mode, push-pull output type, and initialize/set the pin high.      
  GPIOC->MODER |= (1 << 0); // PC0 set to output mode         
  GPIOC->OTYPER &= ~(1 << 0); // PC0 to push-pull output type (clear)     
  GPIOC->ODR |= (1<< 0); // PC0 pin high, I2C mode on (make sure this is working, maybe that is the issue?)

  // Enable I2C2 clock
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Configuring the Bus Timing (use 100 kHz) 
  I2C2->TIMINGR = (0x1 << 28); 
  I2C2->TIMINGR = (0x13 << 0);
  I2C2->TIMINGR = (0xF << 8);
  I2C2->TIMINGR = (0x2 << 16);
  I2C2->TIMINGR = (0x4 << 20); 

  // Enabling the Peripheral (PE)
  I2C2->CR1 |= I2C_CR1_PE;

  // ******** 5.4 Reading the Register ******** //

  // // Set the transaction parameters in the CR2 register
  // // Slave address (SDO), I2C2->RXDR = 0x69
  // I2C2->CR2 |= (0x69 << 1);
  // // Number of bytes to be transferred/received is 1, for part 2 it'll be (2 << 16)
  // I2C2->CR2 |= (1 << 16); 

  // // Set RD_WRN to WRN
  // I2C2 -> CR2 &= ~I2C_CR2_RD_WRN; // 0 is a write

  // // Set the START bit to begin the address frame
  // I2C2->CR2 |= (1 << 13);
  // // Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave NotAcknowledge) flags are set.
  // while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)))
  // {
  //   GPIOC->ODR |= (1 << 6); // Set PC6 high 
  // }
  // // Extra check to see if NACKF was the flag set --> meaning slave did not respond to the address frame
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->ODR |= (1 << 6); // Set PC6 high 
  // }
  
  // GPIOC->ODR &= ~(1 << 6); // Clear PC6 low

  // // Write the address of the "WHO_AM_I" register into the I2C transmit register (TXDR)
  // I2C2->TXDR |= 0x0F;

  // // Wait until TC (Transfer Complete) flag is set
  // while (!(I2C2->ISR & (I2C_ISR_TC))) 
  // {
  //   GPIOC->ODR |= (1 << 8); // Set PC8 high 
  // }
  // GPIOC->ODR &= ~(1 << 8); // Clear PC8 low

  // // Reload CR2 register with the same parameters, but set the RD_WRN bit to indicate a read operation
  // I2C2->CR2 = 0; // resetting CR2 register
  // I2C2->CR2 |= (0x69 << 1);
  // I2C2->CR2 |= (1 << 16);
  // I2C2->CR2 |= (1 << 10); // 1 is a read
  // I2C2->CR2 |= (1 << 13); // Set the START bit again to perform a I2C restart condition

  // // Wait until either RXNE or NACKF flags are set
  // while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  // {
  //   GPIOC->ODR |= (1 << 6); // Set PC6 high (testing purposes)
  // }
  // GPIOC->ODR &= ~(1 << 6); // Clear PC6 low

  // // Extra check to see if NACKF was the flag set --> meaning slave did not respond to the address frame
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->ODR |= (1 << 6); // Set PC6 high (testing purposes)
  // }

  // // Check the contents of the RXDR register to see if it matches the expected value (0xD4)
  // if (I2C2->RXDR == 0xD3) 
  // {
  //   GPIOC->ODR |= (1 << 7); // Set PC7 (blue) high (testing purposes), shows the register value did match
  //   I2C2->CR2 |= (1 << 14); // Set STOP bit
  // }

  // while(1){

  // }

  // ******** 5.5 Initializing the Gyroscope ******** //
  write(0x20);

  // Write ctrlReg1Value to the CTRL_REG1 register of the gyroscope, 0x20 is address
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); 
	I2C2->CR2 &= ~(1 << 10); 
  I2C2->CR2 |= (0x69 << 1) | (2 << 16); 

  // Set the START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }
  
  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high
  }

  I2C2->TXDR = 0x20; 

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }
  
  //bit pattern to turn on Xen, Yen, and PD/Noraml mode, 0x0B = 0000 1011
  I2C2->TXDR = 0x0B;

  while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

	write(0x20);
	if (read() != 0x0b) {
		GPIOC->BSRR |= (1 << 6); // Set PC6 high, means it was not set correctly
	}

  // ******** 5.6 Exercise Specifications ******** //  

  // Clear all LEDs
  GPIOC->BSRR |= (1 << (6 + 16)); // Clear PC6 to turn off the red LED
  GPIOC->BSRR |= (1 << (7 + 16)); // Clear PC7 to turn off the blue LED
  GPIOC->BSRR |= (1 << (8 + 16)); // Clear PC8 to turn off the orange LED
  GPIOC->BSRR |= (1 << (9 + 16)); // Clear PC9 to turn off the green LED

  int16_t xAxis = 0;
	int16_t yAxis = 0;
	const int16_t threshold = 0x01FF;

  while (1) {
		xAxis = readX();
		yAxis = readY();
		
		if (xAxis > threshold) {
			GPIOC->BSRR |= (1 << 6); // Set PC6
		}
		else {
			GPIOC->BSRR |= (1 << (6 + 16)); // Clear PC6
		}
		
		if (yAxis < 0 - threshold) {
			GPIOC->BSRR |= (1 << 7); // Set PC7 
		}
		else {
			GPIOC->BSRR |= (1 << (7 + 16)); // Clear PC7 
		}
		
		if (xAxis < 0 - threshold) {
			GPIOC->BSRR |= (1 << 8); // Set PC8
		}
		else {
			GPIOC->BSRR |= (1 << (8 + 16)); // Clear PC8 
		}
		
		if (yAxis > threshold) {
			GPIOC->BSRR |= (1 << 9); // Set PC9
		}
		else {
			GPIOC->BSRR |= (1 << (9 + 16)); // Clear PC9 
		}
		
		HAL_Delay(100);
	}

}

void write(char val) {
  // Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// Set mode to write
	I2C2->CR2 &= ~(1 << 10);
	I2C2->CR2 |= (0x69 << 1) | (1 << 16);
	
  // Set the START bit 
  I2C2->CR2 |= I2C_CR2_START;

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }
  
  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  // Set register of CTRL_REG1
	I2C2->TXDR = val;
	
	while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  return 0;
}

char read() {
  // Reload CR2 register with the same parameters but set RD_WRN for read operation
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN;
	
  // Set the START bit 
  I2C2->CR2 |= I2C_CR2_START;

  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  // Extra check to see if NACKF was the flag set --> meaning slave did not respond to the address frame
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  char val = I2C2->RXDR;

  // Wait until TC (Transfer Complete) flag is set
  while (!(I2C2->ISR & (I2C_ISR_TC)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

	return val;
}

int16_t readX() {
	
  int16_t xAxis = 0;
	write(0xA8);
	I2C2->CR2 |= (1 << 14); // set STOP bit

	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (2 << 16) | I2C_CR2_RD_WRN;
	
  // Set the START bit 
  I2C2->CR2 |= I2C_CR2_START;

  // 1st 8-bit of data

  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  // Extra check to see if NACKF was the flag set --> meaning slave did not respond to the address frame
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }
	
	xAxis = I2C2->RXDR;
	
	// 2nd 8-bit of data

  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  // Extra check to see if NACKF was the flag set --> meaning slave did not respond to the address frame
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

	xAxis |= (I2C2->RXDR << 8);
	
  // Wait until TC (Transfer Complete) flag is set
  while (!(I2C2->ISR & (I2C_ISR_TC)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

	return xAxis;
}

int16_t readY() {
	
  int16_t yAxis = 0;
	write(0xAA);
	I2C2->CR2 |= (1 << 14);

	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (2 << 16) | I2C_CR2_RD_WRN;

	// Set the START bit 
  I2C2->CR2 |= I2C_CR2_START;
	
	// 1st 8-bit of data
	
  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  // Extra check to see if NACKF was the flag set --> meaning slave did not respond to the address frame
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high
  }
	
	yAxis = I2C2->RXDR;
	
	// 2nd 8-bit of data


  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

  // Extra check to see if NACKF was the flag set --> meaning slave did not respond to the address frame
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

	yAxis |= (I2C2->RXDR << 8);
	
  // Wait until TC (Transfer Complete) flag is set
  while (!(I2C2->ISR & (I2C_ISR_TC)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high 
  }

	return yAxis;
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
