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

Alternate targets:
  EMU:   Firmware emulation and memory debugging under linux.
*/

#include <Kernel.h>
#include <Platform/Platform.h>
#include <Drivers/i2c-adapter/i2c-adapter.h>
#include <Drivers/ADP8866/ADP8866.h>
#include <XenoSession/Console/ManuvrConsole.h>

#include "Digitabulum/USB/STM32F7USB.h"
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

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"


/* This global makes this source file read better. */
Kernel* kernel = nullptr;

TIM_HandleTypeDef htim2;  // This is the timer for the CPLD clock.


volatile void _hack_sadvance() {
  if (kernel) kernel->advanceScheduler();
}


/* Function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void unused_gpio(void);


void HAL_MspInit() {
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

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
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
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
  //RCC_OscInitStruct.PLL.PLLN = 432;   // 216MHz
  RCC_OscInitStruct.PLL.PLLN = 400;   // 200MHz
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
  while (1) {
  }
}
#endif



/*******************************************************************************
* Functions that just print things.                                            *
*******************************************************************************/
// TODO: This is a kludge until proper fxn ptrs can be passed into the Console.
void printHelp() {
  Kernel::log("Help would ordinarily be displayed here.\n");
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
  TM_USBD_CDC_Init(TM_USB_FS);
  TM_USBD_Start(TM_USB_FS);

  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform.platformPreInit();
  kernel = platform.kernel();

  CPLDDriver _cpld;
  kernel->subscribe(&_cpld);

  LegendManager _legend_manager(&_cpld);
  kernel->subscribe(&_legend_manager);

  I2CAdapter i2c(1);
  kernel->subscribe(&i2c);

  // Pins 58 and 63 are the reset and IRQ pin, respectively.
  // This is translated to pins 10 and 13 on PortD.
  ADP8866 leds(58, 63, 0x27);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  kernel->subscribe((EventReceiver*) &leds);

  INA219 ina219(0x4A);
  i2c.addSlaveDevice(&ina219);


  RN4677Pins rn_pins;  // TODO: Still not happy about this. Needless stack burn. Const.
  rn_pins.reset = 68;
  rn_pins.ean   = 69; // WO, SystemConf. Pull-down.
  rn_pins.p24   = 70; // WO, SystemConf. Pull-up.

  rn_pins.sbt   = 51; // WO, SW_BTN
  rn_pins.swu   = 36; // WO, software wake-up.

  rn_pins.p20   = 21; // WO, SystemConf. Pull-up.

  rn_pins.p04   = 18; // RO, Status 0
  rn_pins.p15   = 19; // RO, Status 1
  rn_pins.led   = 35; // RO, LED State

  rn_pins.p05   = 54; // Configurable.
  rn_pins.p31   = 67; //
  rn_pins.p32   = 66; //
  rn_pins.p33   = 65; //
  rn_pins.p34   = 64; //
  rn_pins.p37   = 24; //

  RN4677 bt(&rn_pins);
  kernel->subscribe((EventReceiver*) &bt);


  SDCard sd;
  kernel->subscribe((EventReceiver*) &sd);

  IREmitter ir;
  kernel->subscribe((EventReceiver*) &ir);

  HapticStrap strap;
  kernel->subscribe((EventReceiver*) &strap);

  PMU pmu(&ina219);
  kernel->subscribe((EventReceiver*) &pmu);

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
