/*
File:   DigitabulumPMU.cpp
Author: J. Ian Lindsay
Date:   2016.03.31

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


#include "DigitabulumPMU.h"
#include <DataStructures/StringBuilder.h>

#include "stm32f7xx_hal.h"



// If your driver represents a statically-allocated unit, you can do something
//   like this...
//volatile PMU* PMU::INSTANCE = NULL;


PMU::PMU() {
  //INSTANCE = this;
}


PMU::~PMU() {
}


void PMU::gpioSetup() {
  GPIO_InitTypeDef GPIO_InitStruct;

  // We bind timer2-channel1 to the LED pin.
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/****************************************************************************************************
*  ▄▄▄▄▄▄▄▄▄▄▄  ▄               ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄        ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░░▌▐░▌             ▐░▌▐░░░░░░░░░░░▌▐░░▌      ▐░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀▀▀  ▐░▌           ▐░▌ ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌░▌     ▐░▌ ▀▀▀▀█░█▀▀▀▀ ▐░█▀▀▀▀▀▀▀▀▀
* ▐░▌            ▐░▌         ▐░▌  ▐░▌          ▐░▌▐░▌    ▐░▌     ▐░▌     ▐░▌
* ▐░█▄▄▄▄▄▄▄▄▄    ▐░▌       ▐░▌   ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌ ▐░▌   ▐░▌     ▐░▌     ▐░█▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░░▌    ▐░▌     ▐░▌    ▐░░░░░░░░░░░▌▐░▌  ▐░▌  ▐░▌     ▐░▌     ▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀▀▀      ▐░▌   ▐░▌     ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌   ▐░▌ ▐░▌     ▐░▌      ▀▀▀▀▀▀▀▀▀█░▌
* ▐░▌                ▐░▌ ▐░▌      ▐░▌          ▐░▌    ▐░▌▐░▌     ▐░▌               ▐░▌
* ▐░█▄▄▄▄▄▄▄▄▄        ▐░▐░▌       ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌     ▐░▐░▌     ▐░▌      ▄▄▄▄▄▄▄▄▄█░▌
* ▐░░░░░░░░░░░▌        ▐░▌        ▐░░░░░░░░░░░▌▐░▌      ▐░░▌     ▐░▌     ▐░░░░░░░░░░░▌
*  ▀▀▀▀▀▀▀▀▀▀▀          ▀          ▀▀▀▀▀▀▀▀▀▀▀  ▀        ▀▀       ▀       ▀▀▀▀▀▀▀▀▀▀▀
*
* These are overrides from EventReceiver interface...
****************************************************************************************************/

/**
* Debug support function.
*
* @return a pointer to a string constant.
*/
const char* PMU::getReceiverName() {  return "PMU";  }


/**
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void PMU::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
}



/**
* There is a NULL-check performed upstream for the scheduler member. So no need
*   to do it again here.
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t PMU::bootComplete() {
  EventReceiver::bootComplete();   // Call up to get scheduler ref and class init.
  gpioSetup();
  return 0;
}



/**
* If we find ourselves in this fxn, it means an event that this class built (the argument)
*   has been serviced and we are now getting the chance to see the results. The argument
*   to this fxn will never be NULL.
*
* Depending on class implementations, we might choose to handle the completed Event differently. We
*   might add values to event's Argument chain and return RECYCLE. We may also free() the event
*   ourselves and return DROP. By default, we will return REAP to instruct the EventManager
*   to either free() the event or return it to it's preallocate queue, as appropriate. If the event
*   was crafted to not be in the heap in its own allocation, we will return DROP instead.
*
* @param  event  The event for which service has been completed.
* @return A callback return code.
*/
int8_t PMU::callback_proc(ManuvrRunnable *event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = event->kernelShouldReap() ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->event_code) {
    default:
      break;
  }

  return return_value;
}



int8_t PMU::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;

  switch (active_event->event_code) {
    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
  return return_value;
}


// Pass 1 for low freq, 0 for max
int8_t PMU::cpu_scale(uint8_t _freq) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.AHBCLKDivider  = (_freq == 1 ? RCC_SYSCLK_DIV4 : RCC_SYSCLK_DIV1);
  RCC_ClkInitStruct.APB1CLKDivider = (_freq == 1 ? RCC_HCLK_DIV4   : RCC_HCLK_DIV16);
  RCC_ClkInitStruct.APB2CLKDivider = (_freq == 1 ? RCC_HCLK_DIV1   : RCC_HCLK_DIV4);
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    while(1) { ; }
  }

  switch (_freq) {
    case 0:
      local_log.concat("CPU now at 216MHz\n");
      break;
    case 1:
      local_log.concat("CPU now at 54MHz\n");
      break;
    default:
      local_log.concat("Invalid CPU freq.\n");
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
  return 0;
}



void PMU::procDirectDebugInstruction(StringBuilder *input) {
#ifdef __MANUVR_CONSOLE_SUPPORT
  char* str = input->position(0);

  switch (*(str)) {
    case 'f':
    case 'F':
      cpu_scale(*(str) == 'f' ? 0 : 1);
      break;
    default:
      #ifdef __MANUVR_DEBUG
      EventReceiver::procDirectDebugInstruction(input);
      #endif
      break;
  }
#endif
  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
}
