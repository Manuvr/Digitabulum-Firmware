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
#include "Digitabulum/RovingNetworks/RN4677/RN4677.h"
#include "Digitabulum/ManuLegend/ManuManager.h"
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

TIM_HandleTypeDef htim2;  // This is the timer for the IR LED and haptic strap.


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


void MX_TIM2_Init() {
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
  GPIO_InitStruct.Pin   = GPIO_PIN_6;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);


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
  MX_TIM2_Init();

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
  gpioDefine(32, OUTPUT);

  #if defined(RUN_WITH_HSE)
    setPin(32, true);  // EXT OSC enabled.

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    /* Digitabulum:       24MHz OSC... */
    /* STM32F7-Discovery: 25MHz OSC... */
    RCC_OscInitStruct.PLL.PLLM = HSE_VALUE / 1000000;

  #elif defined(RUN_WITH_HSI)
    setPin(32, false);  // EXT OSC disabled.

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;
  #else
    #error "You must specify which oscillator to start with."
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
  while(1) { ; }
}
#endif




/*
* Pin defs for this module.
*
* These Port B pins are push-pull outputs:
* #  Default  r1  Purpose
* -------------------------------------------------
* 9     0     25  ~CPLD Reset
* 14    0     30  SPI2_MISO  (SPI2 is slave and Rx-only)
*
* These Port C pins are inputs with a wakeup ISR attached to
*    the rising-edge.
* #  Default  r1  Purpose
* ---------------------------------------------------
* 13    0     45  IRQ_WAKEUP
*
* These Port E pins are inputs:
* #  Default  r1  Purpose
* ---------------------------------------------------
* 11    0     75  CPLD_GPIO_0
* 14    0     78  CPLD_GPIO_1
*
* These Port C pins are push-pull outputs:
* #  Default  r1  Purpose
* ---------------------------------------------------
* 2     1     33  DEN_AG_CARPALS
*/
const CPLDPins cpld_pins(
  25, // CPLD's reset pin
  30, // AKA: SPI2_MISO
  45, // CPLD's IRQ_WAKEUP pin
  75, // GPIO
  78, // GPIO
  33 // The DEN_AG pin on the carpals IMU.
);

const I2CAdapterOptions i2c_opts(
  1,   // Device number
  23, // sda
  22  // scl
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
