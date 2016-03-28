/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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


#include <Kernel.h>
#include "FirmwareDefs.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/* Private variables ---------------------------------------------------------*/
Kernel* kernel      = NULL;

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

void system_setup() {
  SCB_EnableICache();       /* Enable I-Cache */
  SCB_EnableDCache();       /* Enable D-Cache */

  SystemClock_Config();    /* Configure the system clock */

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
}


int main(void) {
  system_setup();   // Need to setup clocks and CPU...


  /* Initialize all configured peripherals */
  //MX_GPIO_Init();
  //MX_I2C1_Init();
  //TODO: Strike. MX_RNG_Init();
  //MX_SDMMC1_SD_Init();
  //MX_SPI1_Init();
  //MX_SPI2_Init();
  //MX_TIM1_Init();
  //MX_TIM2_Init();
  //MX_USART2_UART_Init();
  //MX_USB_OTG_FS_USB_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  kernel = new Kernel();  // Instance a kernel.

  kernel->bootstrap();

  /* Infinite loop */
  while (1) {
    kernel->procIdleFlags();

    // Move the kernel log to stdout.
    if (Kernel::log_buffer.count()) {
      if (!kernel->getVerbosity()) {
        Kernel::log_buffer.clear();
      }
      else {
        //printf("%s", Kernel::log_buffer.position(0));
        Kernel::log_buffer.drop_position(0);
      }
    }
  }
}


/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();  // Or this?  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


  #if defined(RUN_WITH_HSE)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    /* Digitabulum's 24MHz OSC... */
    #if HSE_VALUE == 24000000
      RCC_OscInitStruct.PLL.PLLM = 24;
    #endif

    /* STM32F7-Discovery. 25MHz OSC... */
    #if HSE_VALUE == 25000000
      RCC_OscInitStruct.PLL.PLLM = 25;
    #endif
  #elif defined(RUN_WITH_HSI)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;
  #endif
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;


  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while(1) { ; }
  }

  if(HAL_PWREx_ActivateOverDrive() != HAL_OK) {
    while(1) { ; }
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    while(1) { ; }
  }

  PeriphClkInitStruct.PeriphClockSelection =
                              RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1 |
                              RCC_PERIPHCLK_SDMMC1 | RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection  = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;

  /* We are going to setup the 48MHz source to be consistent regardless of CPU clock. */
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;


  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK) {
    while(1) { ; }
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

  while (1) {
  }
}

#endif

#ifdef __cplusplus
 }
#endif
