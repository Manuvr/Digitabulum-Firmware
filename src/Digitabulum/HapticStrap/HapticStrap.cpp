/*
File:   HapticStrap.cpp
Author: J. Ian Lindsay
Date:   2016.05.17

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


#include "HapticStrap.h"
#include <DataStructures/StringBuilder.h>

#include "stm32f7xx_hal.h"


/****************************************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here. Initializers first, functions second.
****************************************************************************************************/

volatile HapticStrap* HapticStrap::INSTANCE = NULL;

// Milliseconds to vibrate, Pulse count.
const unsigned char MSG_ARGS_VIBRATE[] = {  UINT16_FM, UINT8_FM, 0  };

const MessageTypeDef haptic_message_defs[] = {
  /*
    For messages that have arguments, we have the option of defining inline lables for each parameter.
    This is advantageous for debugging and writing front-ends. We case-off here to make this choice at
    compile time.
  */
  {  DIGITABULUM_MSG_GPIO_VIBRATE_0  , MSG_FLAG_EXPORTABLE,  "VIBRATE_0"            , MSG_ARGS_VIBRATE }, // Some class wants to trigger vibrator 0.
  {  DIGITABULUM_MSG_GPIO_VIBRATE_1  , MSG_FLAG_EXPORTABLE,  "VIBRATE_1"            , MSG_ARGS_VIBRATE }, // Some class wants to trigger vibrator 1.
};


/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/

HapticStrap::HapticStrap() : EventReceiver() {
  if (NULL == INSTANCE) {
    setReceiverName("StandardIO");
    INSTANCE = this;
    ManuvrMsg::registerMessages(
      haptic_message_defs,
      sizeof(haptic_message_defs) / sizeof(haptic_message_defs)
    );
  }
}


HapticStrap::~HapticStrap() {
}


void HapticStrap::gpioSetup() {
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO for haptic vibrators...
  // PB10     ------> TIM2_CH3
  // PB11     ------> TIM2_CH4
  GPIO_InitStruct.Pin       = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
* This is called when the kernel attaches the module.
* This is the first time the class can be expected to have kernel access.
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t HapticStrap::attached() {
  EventReceiver::attached();   // Call up to get scheduler ref and class init.
  return 0;
}


/**
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void HapticStrap::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
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
int8_t HapticStrap::callback_proc(ManuvrRunnable *event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = event->kernelShouldReap() ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    default:
      break;
  }

  return return_value;
}


int8_t HapticStrap::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;

  switch (active_event->eventCode()) {
    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  flushLocalLog();
  return return_value;
}


#if defined(MANUVR_CONSOLE_SUPPORT)
void HapticStrap::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  switch (*(str)) {
    case '[':  // Buzz vibrator 0
      break;
    case ']':  // Buzz vibrator 1
      break;
    default:
      #ifdef __MANUVR_DEBUG
      EventReceiver::procDirectDebugInstruction(input);
      #endif
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT
