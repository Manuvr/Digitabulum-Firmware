/*
File:   RN4677.cpp
Author: J. Ian Lindsay
Date:   2016.04.17

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

*/

#include "RN4677.h"
#include <stm32f7xx_hal_usart.h>
#include <stm32f7xx_hal_gpio.h>


UART_HandleTypeDef huart2;


RN4677::RN4677() : RNBase() {
}


RN4677::~RN4677() {
  __USART2_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3);
}


/**
* Setup GPIO pins and their bindings to on-chip peripherals, if required.
*/
void RN4677::gpioSetup(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* These Port B pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 2     -      BT_PIO_04
  * 3     -      BT_PIO_15
  * 8     -      BT_PIO_37 (Tentative)
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8;
  GPIO_InitStruct.Mode       = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* These Port C pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 3     -      BT_LED_1
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_3;
  GPIO_InitStruct.Mode       = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* These Port D pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 6     -      BT_PIO_05
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_6;
  GPIO_InitStruct.Mode       = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* These Port E pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 0     -      BT_PIO_34 (Tentative)
  * 1     -      BT_PIO_33 (Tentative)
  * 2     -      BT_PIO_32 (Tentative)
  * 3     -      BT_PIO_31 (Tentative)
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode       = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


  /* These Port B pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 5     0      BT_PIO_20
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_5;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /* These Port C pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 4     0      WAKE_SW
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_4;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /* These Port D pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 3     0      BT_SW_BTN
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_3;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /* These Port E pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 4     0      ~BT_RESET
  * 5     1      BT_EAN
  * 6     1      BT_PIO_24
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);


  __USART2_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
}
