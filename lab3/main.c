/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN 2 */
  
  /* Initialize all configured peripherals */

  // Enable Timers 2 and 3 in RCC
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Set up for Timer 3
  TIM3->CCMR1 &= ~(0x3 | 0x3<<8);
  TIM3->CCMR1 |= (0x3<<5 | 0x3 << 13);
  TIM3->CCER |= 0x1 | 0x1 << 4;
  // Set up timer 3 frequency
  TIM3->PSC = 0;
  TIM3->ARR = 10000;

  // Set timer 2 interrupts
  TIM2->DIER |= 0x1;
  // Set up timer 2 frequency
  TIM2->PSC = 2000;
  TIM2->ARR = 500;

  // Enable LEDs  
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER |= ((0x1 << 13) | (0x1 << 15) | (0x1 << 16) | (0x1 << 18));
  GPIOC->OTYPER &= ~((0x1 << 6) | (0x1 << 7) | (0x1 << 8) | (0x1 << 9)); 
  GPIOC->OSPEEDR &= ~((0x1 << 12) | (0x1 << 14) | (0x1 << 16) | (0x1 << 18)); 
  GPIOC->PUPDR &= ~((0x1 << 12) | (0x1 << 13) | (0x1 << 14) | (0x1 << 15) | (0x1 << 16) | (0x1 << 17) | (0x1 << 18) | (0x1 << 19)); 
  
  // Turn on single LED
  GPIOC->ODR |= GPIO_PIN_8;

  // Enable IRQ and priorities
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_SetPriority(TIM2_IRQn, 1);

  // Enable timers
  TIM3->CR1 |= 0x1<<7 | 0x1;
  TIM2->CR1 |= 0x1<<7 | 0x1;

  // Measure specific CCRx values
  //TIM3->CCR1 = 5000;
  //TIM3->CCR2 = 5000;
  /* USER CODE END 2 */

  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Hacky code to fade lights
  int count1 = 0;
  int count1mode = 0;
  int count2 = 10000;
  int count2mode = 0;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    // Current fade of the first light
    TIM3->CCR1 = count1;
    // Determine if fading or lighting up
    if (count1mode == 0) {
      // Increment light value
      count1+=5;
      // Switch modes when at ARR
      if (count1 >= 10000) {
        count1mode = 1;
      }
    } else {
      count1-=5;
      if (count1 <= 0) {
        count1mode = 0;
      }
    }
    
    // Current fade of the second light
    TIM3->CCR2 = count2;
    if (count2mode == 0) {
      count2+=5;
      if (count2 >= 10000) {
        count2mode = 1;
      }
    } else {
      count2-=5;
      if (count2 <= 0) {
        count2mode = 0;
      }
    }
    HAL_Delay(1);
  }
  
  /* USER CODE END 3 */

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
    Error_Handler();
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
    Error_Handler();
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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

