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

#include "FirmwareDefs.h"

#include <Kernel.h>
#include <Drivers/i2c-adapter/i2c-adapter.h>
#include <Drivers/ADP8866/ADP8866.h>

#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/RovingNetworks/RN4677/RN4677.h"
#include "Digitabulum/ManuLegend/ManuLegend.h"
#include "Digitabulum/IREmitter/IREmitter.h"
#include "Digitabulum/HapticStrap/HapticStrap.h"
#include "Digitabulum/SDCard/SDCard.h"
#include "Digitabulum/DigitabulumPMU/DigitabulumPMU.h"

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

#include "tm_stm32_usb_device.h"
#include "tm_stm32_usb_device_cdc.h"


Kernel* kernel      = NULL;
TIM_HandleTypeDef htim2;  // This is the timer for the CPLD clock.


// Milliseconds to vibrate, Pulse count.
const unsigned char MSG_ARGS_VIBRATE[] = {  UINT16_FM, UINT8_FM, 0  };

// Messages that are specific to Digitabulum.
const MessageTypeDef digitabulum_message_defs[] = {
  /*
    For messages that have arguments, we have the option of defining inline lables for each parameter.
    This is advantageous for debugging and writing front-ends. We case-off here to make this choice at
    compile time.
  */
  #if defined (__ENABLE_MSG_SEMANTICS)
  {  DIGITABULUM_MSG_GPIO_VIBRATE_0  , MSG_FLAG_EXPORTABLE,  "VIBRATE_0"            , MSG_ARGS_VIBRATE }, // Some class wants to trigger vibrator 0.
  {  DIGITABULUM_MSG_GPIO_VIBRATE_1  , MSG_FLAG_EXPORTABLE,  "VIBRATE_1"            , MSG_ARGS_VIBRATE }, // Some class wants to trigger vibrator 1.
  #else
  {  DIGITABULUM_MSG_GPIO_VIBRATE_0  , MSG_FLAG_EXPORTABLE,  "VIBRATE_0"            , MSG_ARGS_VIBRATE, NULL }, // Some class wants to trigger vibrator 0.
  {  DIGITABULUM_MSG_GPIO_VIBRATE_1  , MSG_FLAG_EXPORTABLE,  "VIBRATE_1"            , MSG_ARGS_VIBRATE, NULL }, // Some class wants to trigger vibrator 1.
  #endif
};


/* Function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void unused_gpio(void);

#define CMD_BUFF_SIZE 128
static char _cmd_buf[CMD_BUFF_SIZE];
static int _cmd_buf_ptr = 0;


void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  HAL_TIM_OC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM2) {
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();

    /**TIM2 GPIO Configuration
    PA15     ------> TIM2_CH1
    PB10     ------> TIM2_CH3
    PB11     ------> TIM2_CH4
    */
    // GPIO for haptic vibrators...
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if(htim_base->Instance==TIM2) {
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA15     ------> TIM2_CH1
    PB10     ------> TIM2_CH3
    PB11     ------> TIM2_CH4
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
  }
}


static void CDC_Device(void) {
	char ch;
	/* Check if we are ready to use, if drivers are OK installed on computer */
	if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) {
		/* We are ready */
		/* Check if anything received */
		while (TM_USBD_CDC_Getc(TM_USB_FS, &ch)) {
			//TM_USBD_CDC_Putc(TM_USB_FS, ch);
      if ((ch == '\r') || (ch == '\n') || (ch == '\0')) {
        TM_USBD_CDC_Putc(TM_USB_FS, '\n');
        kernel->accumulateConsoleInput((uint8_t*)_cmd_buf, strlen(_cmd_buf), true);
        _cmd_buf_ptr = 0;
        for (int i = 0; i < CMD_BUFF_SIZE; i++) *(_cmd_buf + i) = '\0';
      }
      else {
        _cmd_buf[_cmd_buf_ptr++] = ch;
      }
		}
	}
  else {
		/* We are not ready */
	}
}


bool _tx_in_progress = false;
bool _rx_ready = false;

void VCP_Rx_Notify(uint8_t*, int) {
  _rx_ready = true;
}

void VCP_Tx_Complete() {
  _tx_in_progress = false;
}



void system_setup() {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* These Port B pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 0     0      ~External OSC Enable
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_0;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  SCB_EnableICache();       /* Enable I-Cache */
  SCB_EnableDCache();       /* Enable D-Cache */

  SystemClock_Config();    /* Configure the system clock */

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  for (int i = 0; i < CMD_BUFF_SIZE; i++) *(_cmd_buf + i) = '\0';
}


/****************************************************************************************************
* Main function                                                                                     *
* TODO: We should sort-out what can be in CCM and what cannot be, and after we've allocated all the *
*         I/O buffers, switch over to CCM for our execution stacks.                                 *                                                                                 *
****************************************************************************************************/
int main(void) {
  system_setup();   // Need to setup clocks and CPU...

  unused_gpio();    // We don't use all the GPIO on this platform.
  MX_TIM2_Init();

  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  kernel = new Kernel();  // Instance a kernel.
  #if defined(__MANUVR_DEBUG)
    kernel->profiler(true);
  #endif

  CPLDDriver _cpld;
  kernel->subscribe(&_cpld);

  I2CAdapter i2c(1);
  kernel->subscribe(&i2c);

  // Pins 58 and 63 are the reset and IRQ pin, respectively.
  // This is translated to pins 10 and 13 on PortD.
  ADP8866 adp8866(58, 63, 0x27);
  kernel->subscribe((EventReceiver*) &adp8866);
  i2c.addSlaveDevice(&adp8866);

  LegendManager _legend_manager;
  kernel->subscribe(&_legend_manager);

  RN4677 bt;
  kernel->subscribe((EventReceiver*) &bt);

  SDCard sd;
  kernel->subscribe((EventReceiver*) &sd);

  IREmitter ir;
  kernel->subscribe((EventReceiver*) &ir);

  HapticStrap strap;
  kernel->subscribe((EventReceiver*) &strap);

  PMU pmu;
  kernel->subscribe((EventReceiver*) &pmu);

  kernel->bootstrap();

  /* Infinite loop */
  while (1) {
    kernel->procIdleFlags();
    if (_rx_ready) CDC_Device();

    if (!_tx_in_progress) {
      if (Kernel::log_buffer.count()) {
        if (!kernel->getVerbosity()) {
          Kernel::log_buffer.clear();
        }
        else {
          if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) {
            TM_USBD_CDC_Puts(TM_USB_FS, Kernel::log_buffer.position(0));
            _tx_in_progress = true;
      		  TM_USBD_CDC_Process(TM_USB_FS);
            Kernel::log_buffer.drop_position(0);
          }
        }
      }
    }
  }
}


/****************************************************************************************************
* Clock-tree config...                                                                              *
****************************************************************************************************/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();  // Or this?  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


  #if defined(RUN_WITH_HSE)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

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



#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1) {
  }
}
#endif

#ifdef __cplusplus
  }
#endif
