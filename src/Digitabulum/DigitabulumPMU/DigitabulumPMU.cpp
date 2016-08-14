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

#include <Platform/Platform.h>


/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/
volatile static unsigned long _stat1_change_time = 0;
volatile static unsigned long _stat2_change_time = 0;
volatile static unsigned int  _stat1_prior_delta = 0;
volatile static unsigned int  _stat2_prior_delta = 0;


volatile PMU* PMU::INSTANCE = NULL;

int PMU::pmu_cpu_clock_rate(CPUFreqSetting _setting) {
  switch (_setting) {
    case CPUFreqSetting::CPU_27:         return 27;
    case CPUFreqSetting::CPU_54:         return 54;
    case CPUFreqSetting::CPU_216:        return 216;
    case CPUFreqSetting::CPU_CLK_UNDEF:  return 0;
  }
  return 0;
}

/**
* Debug and logging support.
*
* @return a const char* containing a human-readable representation of a fault code.
*/
const char* PMU::getChargeStateString(ChargeState code) {
  switch (code) {
    case ChargeState::FULL:      return "FULL";
    case ChargeState::CHARGING:  return "CHARGING";
    case ChargeState::DRAINING:  return "DRAINING";
    case ChargeState::TEST:      return "TEST";
    case ChargeState::ERROR:     return "ERROR";
    default:                     return "<UNDEF>";
  }
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

PMU::PMU(INA219* _cv_sense) : EventReceiver() {
  setReceiverName("PMU");
  INSTANCE   = this;
  _cpu_clock = CPUFreqSetting::CPU_CLK_UNDEF;
  _stat1_pin = 16;
  _stat2_pin = 17;
  _ina219 = _cv_sense;
}


PMU::~PMU() {
}


// Pass 1 for low freq, 0 for max
int8_t PMU::cpu_scale(uint8_t _freq) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  switch (_freq) {
    case 0:
      _cpu_clock = CPUFreqSetting::CPU_216;
      RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
      break;
    case 1:
      _cpu_clock = CPUFreqSetting::CPU_54;
      RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV4;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
      break;
    default:
      Kernel::log("Invalid CPU freq.\n");
      return -1;
  }

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    while(1) { ; }
  }
  _cpu_clock_rate = HAL_RCC_GetHCLKFreq();
  HAL_SYSTICK_Config(_cpu_clock_rate/8000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  local_log.concatf("CPU now at %dMHz\n", (_cpu_clock_rate/1000000));
  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
  return 0;
}



void PMU::gpioSetup() {
  /* These Port B pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 0     0      CHG_STAT_1
  * 1     0      CHG_STAT_2
  */
  // This will cause interrupts to be enabled for these pins.
  setPinFxn(_stat1_pin, CHANGE_PULL_UP, mcp73833_stat1_isr);
  setPinFxn(_stat2_pin, CHANGE_PULL_UP, mcp73833_stat2_isr);
  _er_set_flag(DIGITAB_PMU_FLAG_STAT1, readPin(_stat1_pin));
  _er_set_flag(DIGITAB_PMU_FLAG_STAT2, readPin(_stat2_pin));
}

/*
* Updates the local state if warranted, but always returns local state.
*/
ChargeState PMU::getChargeState() {
  uint8_t idx = 0;
  bool s1 = readPin(_stat1_pin);
  bool s2 = readPin(_stat2_pin);
  _er_set_flag(DIGITAB_PMU_FLAG_STAT1, s1);
  _er_set_flag(DIGITAB_PMU_FLAG_STAT2, s2);
  idx += s1 ? 1 : 0;
  idx += s2 ? 2 : 0;
  switch (idx) {
    case 0:
      _charge_state = ChargeState::TEST;
      break;
    case 1:
      _charge_state = ChargeState::FULL;
      break;
    case 2:
      _charge_state = ChargeState::CHARGING;
      break;
    case 3:
      // Both pins high. No inference possible.
    default:
      break;
  }
  return _charge_state;
}


void PMU::set_stat1_delta(unsigned int nu) {
  getChargeState();
  //local_log.concatf("STAT1 delta: %u\n", nu);
  //Kernel::log(&local_log);
}

void PMU::set_stat2_delta(unsigned int nu) {
  getChargeState();
  //local_log.concatf("STAT2 delta: %u\n", nu);
  //Kernel::log(&local_log);
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
* @param A pointer to a StringBuffer object to receive the output.
*/
void PMU::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
  output->concatf("-- CPU freq                  %d MHz\n",  _cpu_clock_rate);
  output->concatf("-- Charge state              %s\n",      getChargeStateString());
  output->concatf("-- STAT1                     %s\n",      (_er_flag(DIGITAB_PMU_FLAG_STAT1) ? "hi" : "lo"));
  output->concatf("-- STAT2                     %s\n",      (_er_flag(DIGITAB_PMU_FLAG_STAT2) ? "hi" : "lo"));
  output->concatf("-- _stat1_delta              %u\n",      _stat1_delta);
  output->concatf("-- _stat2_delta              %u\n",      _stat2_delta);

  output->concatf("-- _stat1_change_time        %lu\n",      _stat1_change_time);
  output->concatf("-- _stat2_change_time        %lu\n",      _stat2_change_time);
  output->concatf("-- _stat1_prior_delta        %lu\n",      _stat1_prior_delta);
  output->concatf("-- _stat2_prior_delta        %lu\n",      _stat2_prior_delta);
}


/**
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t PMU::bootComplete() {
  EventReceiver::bootComplete();   // Call up to get scheduler ref and class init.
  gpioSetup();
  cpu_scale(1);
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


#ifdef __MANUVR_CONSOLE_SUPPORT
void PMU::procDirectDebugInstruction(StringBuilder *input) {
  const char* str = (char *) input->position(0);
  char c    = *str;
  int temp_int = 0;

  if (input->count() > 1) {
    // If there is a second token, we proceed on good-faith that it's an int.
    temp_int = input->position_as_int(1);
  }
  else if (strlen(str) > 1) {
    // We allow a short-hand for the sake of short commands that involve a single int.
    temp_int = atoi(str + 1);
  }

  switch (c) {
    case 'f':
    case 'F':
      cpu_scale(*(str) == 'f' ? 0 : 1);
      break;

    case 'w':
      // Find the current wattage draw.
      break;

    case 'p':
    case 'P':
      // Start or stop the periodic sensor read.
      break;

    case 'e':
      // Read the present battery voltage.
      break;

    case 'm':   // Set the system-wide power mode.
      if (255 != temp_int) {
        ManuvrRunnable* event = Kernel::returnEvent(MANUVR_MSG_SYS_POWER_MODE);
        event->addArg((uint8_t) temp_int);
        EventReceiver::raiseEvent(event);
        local_log.concatf("Power mode is now %d.\n", temp_int);
      }
      else {
      }
      break;

    default:
      EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
}
#endif  // __MANUVR_CONSOLE_SUPPORT



/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/

/*
* This is an ISR.
*/
void mcp73833_stat1_isr() {
  unsigned long current = millis();
  unsigned int delta = (current >= _stat1_change_time) ? (current - _stat1_change_time) : (_stat1_change_time - current);
  _stat1_change_time = current;
  delta = delta >> 2;   // Filter small deviations in timing that are not significant.
  if (delta != _stat1_prior_delta) {
    ((PMU*) PMU::INSTANCE)->set_stat1_delta(delta);
    _stat1_prior_delta = delta;
  }
}

/*
* This is an ISR.
*/
void mcp73833_stat2_isr() {
  unsigned long current = millis();
  unsigned int delta = (current >= _stat2_change_time) ? (current - _stat2_change_time) : (_stat2_change_time - current);
  _stat2_change_time = current;
  delta = delta >> 2;   // Filter small deviations in timing that are not significant.
  if (delta != _stat2_prior_delta) {
    ((PMU*) PMU::INSTANCE)->set_stat2_delta(delta);
    _stat2_prior_delta = delta;
  }
}
