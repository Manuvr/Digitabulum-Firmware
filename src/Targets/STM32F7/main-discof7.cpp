/*
File:   main.cpp
Author: J. Ian Lindsay
Date:   2016.03.01

Copyright 2016 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

    .__       ,    .     .
    |  \* _ *-+- _.|_ . .|. .._ _
    |__/|(_]| | (_][_)(_||(_|[ | )
         ._|

Intended target is an STM32F7.
*/

#include <Kernel.h>
#include <Platform/Platform.h>
#include <Platform/Peripherals/I2C/I2CAdapter.h>
#include <Drivers/ADP8866/ADP8866.h>
#include <XenoSession/Console/ManuvrConsole.h>

#include "Digitabulum/USB/STM32F7USB.h"
#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/ManuLegend/ManuManager.h"
#include "Digitabulum/SDCard/SDCard.h"

#ifdef __cplusplus
  extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"


/* This global makes this source file read better. */
Kernel* kernel = nullptr;


volatile void _hack_sadvance() {
  if (kernel) kernel->advanceScheduler();
}


/* Function prototypes -----------------------------------------------*/
void SystemClock_Config();
void MX_FREERTOS_Init();
void unused_gpio();


void HAL_MspInit() {
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void unused_gpio() {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOI_CLK_ENABLE();

  // The USB driver takes care of this for us.
  ///*Configure GPIO pins : PA12 PA11 PA10 */
  //GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10;
  //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  //GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* These Port C pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 6     1      Expansion reset pin
  */

  /* Everything below represents a pin that is unused. */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin  = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
  // Digitabulum doesn't use these ports...
  __GPIOF_CLK_DISABLE();
  __GPIOG_CLK_DISABLE();
  __GPIOI_CLK_DISABLE();

  // GPIO_PIN_1 is the HSE input. If we aren't going to use it, disable the port.
  #if defined(HSE_VALUE)
    /*Configure GPIO pins : PH15 PH13 PH14 PH2
                             PH3 PH4 PH5 PH12
                             PH11 PH10 PH6 PH8
                             PH9 PH7 */
    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_2
                            |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12
                            |GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_8
                            |GPIO_PIN_9|GPIO_PIN_7;
  #endif
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  #if defined(HSE_VALUE)
    // TODO: Might-could disable this too? Only pin in-use is the EXT CLK input.
    __GPIOH_CLK_DISABLE();
  #endif

  /*Configure GPIO pins : PD7 PD0 PD5 PD1
                           PD4 PD15 PD14 PD12
                           PD11 PD9 PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_1
                          |GPIO_PIN_4|GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_12
                          |GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_8;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE13 PE8 PE9 PE7 PE10 PE12 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}


void system_setup() {
  SCB_EnableICache();       /* Enable I-Cache */
  SCB_EnableDCache();       /* Enable D-Cache */

  SystemClock_Config();    /* Configure the system clock */

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  unused_gpio();    // We don't use all the GPIO on this platform.

  /* Call init function for freertos objects (in freertos.c) */
  #if defined (__MANUVR_FREERTOS)
    MX_FREERTOS_Init();
  #endif
}


/****************************************************************************************************
* Clock-tree config...                                                                              *
****************************************************************************************************/
void SystemClock_Config() {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();  // Or this?  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* These Port C pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 0     0      ~External OSC Enable
  */
  //gpioDefine(32, OUTPUT);

  #if defined(RUN_WITH_HSE)
    //setPin(32, true);  // EXT OSC enabled.

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    /* Digitabulum:       24MHz OSC... */
    /* STM32F7-Discovery: 25MHz OSC... */
    RCC_OscInitStruct.PLL.PLLM = HSE_VALUE / 1000000;

  #elif defined(RUN_WITH_HSI)
    //setPin(32, false);  // EXT OSC disabled.

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;
  #else
    #error "You must specify which oscillator to start with."
  #endif
  //RCC_OscInitStruct.PLL.PLLN = 432;   // 216MHz
  RCC_OscInitStruct.PLL.PLLN = 400;   // 200MHz
  //RCC_OscInitStruct.PLL.PLLN = 384;   // 192MHz
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    while(1) { ; }
  }

  PeriphClkInitStruct.PeriphClockSelection =
                              RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1 |
                              RCC_PERIPHCLK_SDMMC1 | RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInitStruct.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInitStruct.Clk48ClockSelection  = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;

  /* We are going to setup the 48MHz source to be consistent regardless of CPU clock. */
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;

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
  while(1) { ; }
}
#endif



/*
* Pin defs given here assume a WROOM32 module.
*/
const CPLDPins cpld_pins(
  255, //17,  // CPLD's reset pin
  255, //18,  // Transfer request
  255, //19,  // CPLD's IRQ_WAKEUP pin
  255, //21,  // CPLD clock input
  255, //22,  // CPLD OE pin
  255, //23,  // CPLD GPIO
  255, //25,  // SPI1 CS
  255, //26,  // SPI1 CLK
  255, //27,  // SPI1 MOSI
  255, //32,  // SPI1 MISO
  255, //33,  // SPI2 CS
  255, //34,  // SPI2 CLK
  255  //35   // SPI2 MOSI
);


const I2CAdapterOptions i2c_opts(
  0,   // Device number
  13,  // IO13 (sda)
  14   // IO14 (scl)
);

/****************************************************************************************************
* Main function                                                                                     *
* TODO: We should sort-out what can be in CCM and what cannot be, and after we've allocated all the *
*         I/O buffers, switch over to CCM for our execution stacks.                                 *                                                                                 *
****************************************************************************************************/
int main() {
  system_setup();   // Need to setup clocks and CPU...

  /* Start scheduler */
  //osKernelStart();
  TM_USBD_CDC_Init(TM_USB_FS);
  TM_USBD_Start(TM_USB_FS);

  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform.platformPreInit();
  kernel = platform.kernel();

  CPLDDriver _cpld(&cpld_pins);
  kernel->subscribe(&_cpld);

  ManuManager _legend_manager(&_cpld);
  kernel->subscribe(&_legend_manager);

  I2CAdapter i2c(&i2c_opts);
  kernel->subscribe(&i2c);

  // Pins 30 and 31 are the reset and IRQ pin, respectively.
  // This is translated to pins D11 and D12 on the Disco's arduino harness.
  ADP8866 leds(255, 255);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  kernel->subscribe((EventReceiver*) &leds);

  SDCard sd;
  kernel->subscribe((EventReceiver*) &sd);

  platform.bootstrap();

  // TODO: Until smarter idea is finished, manually patch the USB-VCP into a
  //         BufferPipe that takes the place of the transport driver.
  STM32F7USB _console_patch;
  ManuvrConsole _console((BufferPipe*) &_console_patch);
  kernel->subscribe((EventReceiver*) &_console);
  kernel->subscribe((EventReceiver*) &_console_patch);

  platform.forsakeMain();
  return 0;
}

#ifdef __cplusplus
  }
#endif
