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


2016.04.30   Migration log
The RN42HID driver from which this class descended is possibly the least-portable
thing I ever wrote. This is the end of an extension of classes with a hierarchy like this...
  EventReceiver
    ManuvrXport
      RNBase
        This class
I will attempt to collect fail in this driver until it is properly abstracted and brought
up to par.

*/

#include "RN4677.h"
#include <stm32f7xx_hal_usart.h>
#include <stm32f7xx_hal_gpio.h>


UART_HandleTypeDef huart2;


RN4677::RN4677(uint8_t _rst_pin) : RNBase(_rst_pin) {
  setReceiverName("RN4677");
}


RN4677::~RN4677() {
  __USART2_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8);
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3 | GPIO_PIN_4);
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3 | GPIO_PIN_6);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
}


/**
* Setup GPIO pins and their bindings to on-chip peripherals, if required.
*/
void RN4677::gpioSetup() {
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
  GPIO_InitStruct.Speed      = GPIO_SPEED_HIGH;
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
  GPIO_InitStruct.Speed      = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /* These Port C pins are open-drain outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 4     1      WAKE_SW
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_4;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

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
  GPIO_InitStruct.Pin        = GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);


  // Setting up USART2 to deal with the module...
  __USART2_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART2_FORCE_RESET();

  GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_0;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}



/*
*/
void RN4677::set_bitrate(int _bitrate) {
//  enterCommandMode();
//  StringBuilder *temp = new StringBuilder("U,921K,N");
//  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
//  exitCommandMode();

  HAL_NVIC_DisableIRQ(USART2_IRQn);
  HAL_UART_DeInit(&huart2);

  configured_bitrate = _bitrate;

  /* Now the USART_InitStruct is used to define the
   * properties of USART2
   */
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = configured_bitrate;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  HAL_UART_Init(&huart2);
  USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_IDLEIE;

  HAL_UART_Init(&huart2);    // finally this enables the complete USART2 peripheral
}



/*
* Since the RN42 retains settings across power cycles, we might need the
*   ability to put the thing into a known state before trying to talk to it.
* This fxn should retrace the init procedures only to the point of config. Once
*   this class works reasonably well, we should never be running in this mode,
*   except for recovery from serious amnesia.
*/
void RN4677::force_9600_mode(bool force_low_speed) {
  // TODO: This convention and nomenclature are inverted.
  if (force_low_speed) {
    set_bitrate(9600);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  }
  else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    set_bitrate(115200);
  }

  reset();
}

void RN4677::factoryReset() {
  // TODO: This should probably be done...
}




/*******************************************************************************
* ######## ##     ## ######## ##    ## ########  ######
* ##       ##     ## ##       ###   ##    ##    ##    ##
* ##       ##     ## ##       ####  ##    ##    ##
* ######   ##     ## ######   ## ## ##    ##     ######
* ##        ##   ##  ##       ##  ####    ##          ##
* ##         ## ##   ##       ##   ###    ##    ##    ##
* ########    ###    ######## ##    ##    ##     ######
*
* These are overrides from EventReceiver interface...
*******************************************************************************/
/**
* If we find ourselves in this fxn, it means an event that this class built (the argument)
*   has been serviced and we are now getting the chance to see the results. The argument
*   to this fxn will never be NULL.
*
* Depending on class implementations, we might choose to handle the completed Event differently. We
*   might add values to event's Argument chain and return RECYCLE. We may also free() the event
*   ourselves and return DROP. By default, we will return REAP to instruct the Kernel
*   to either free() the event or return it to it's preallocate queue, as appropriate. If the event
*   was crafted to not be in the heap in its own allocation, we will return DROP instead.
*
* @param  event  The event for which service has been completed.
* @return A callback return code.
*/
int8_t RN4677::callback_proc(ManuvrRunnable *event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = event->kernelShouldReap() ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    case MANUVR_MSG_XPORT_SEND:
      event->clearArgs();
      break;
    default:
      break;
  }

  return return_value;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void RN4677::printDebug(StringBuilder *temp) {
  RNBase::printDebug(temp);
}



int8_t RN4677::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;

  switch (active_event->eventCode()) {
    default:
      return_value += RNBase::notify(active_event);
      break;
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}


#if defined(MANUVR_CONSOLE_SUPPORT)
void RN4677::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = 0;
  if (*(str) != 0) {
    temp_byte = atoi((char*) str+1);
  }

  switch (*(str)) {
    default:
      RNBase::procDirectDebugInstruction(input);
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT
