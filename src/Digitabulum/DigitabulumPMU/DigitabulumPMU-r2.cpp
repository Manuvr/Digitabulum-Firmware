/*
File:   DigitabulumPMU-r2.cpp
Author: J. Ian Lindsay
Date:   2017.06.31

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


#include "DigitabulumPMU-r2.h"
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

const MessageTypeDef pmu_message_defs[] = {
  {  DIGITABULUM_MSG_PMU_READ,   0x0000,    "PMU_READ"    , ManuvrMsg::MSG_ARGS_NONE }, //
};


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
void bq24155_stat_isr() {
}

/*
* This is an ISR.
*/
void ltc294x_alert_isr() {
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

PMU::PMU(BQ24155* charger, LTC294x* fuel_gauge) : EventReceiver("PMU") {
  if (nullptr == INSTANCE) {
    INSTANCE   = this;
    ManuvrMsg::registerMessages(pmu_message_defs, sizeof(pmu_message_defs) / sizeof(MessageTypeDef));
  }
  _cpu_clock = CPUFreqSetting::CPU_CLK_UNDEF;
  _stat1_pin = 16;
  _stat2_pin = 17;

  _bq24155 = charger;
  _ltc294x = fuel_gauge;

  _periodic_pmu_read.repurpose(DIGITABULUM_MSG_PMU_READ, (EventReceiver*) this);
  _periodic_pmu_read.incRefs();
  _periodic_pmu_read.specific_target = (EventReceiver*) this;
  _periodic_pmu_read.priority(1);
  _periodic_pmu_read.alterSchedulePeriod(100);
  _periodic_pmu_read.alterScheduleRecurrence(-1);
  _periodic_pmu_read.autoClear(false);
  _periodic_pmu_read.enableSchedule(false);
}


PMU::~PMU() {
  platform.kernel()->removeSchedule(&_periodic_pmu_read);
}


// Pass 1 for low freq, 0 for max
int8_t PMU::cpu_scale(uint8_t _freq) {

  switch (_freq) {
    case 0:
      _cpu_clock = CPUFreqSetting::CPU_216;
      break;
    case 1:
      _cpu_clock = CPUFreqSetting::CPU_54;
      break;
    default:
      Kernel::log("Invalid CPU freq.\n");
      return -1;
  }

  local_log.concatf("CPU now at %dMHz\n", (_cpu_clock_rate/1000000));
  flushLocalLog();
  return 0;
}



/*
* Updates the local state if warranted, but always returns local state.
*/
ChargeState PMU::getChargeState() {
  uint8_t idx = 0;
  return _charge_state;
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
int8_t PMU::attached() {
  if (EventReceiver::attached()) {
    cpu_scale(1);
    platform.kernel()->addSchedule(&_periodic_pmu_read);
    //_bq24155->init();
    //_ltc294x->init();
    return 1;
  }
  return 0;
}


/**
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void PMU::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
  output->concatf("-- CPU freq                  %d MHz\n",  _cpu_clock_rate);
  output->concatf("-- Charge state              %s\n",      getChargeStateString());
  _ltc294x->printDebug(output);
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
int8_t PMU::callback_proc(ManuvrMsg* event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = (0 == event->refCount()) ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    default:
      break;
  }

  return return_value;
}


int8_t PMU::notify(ManuvrMsg* active_event) {
  int8_t return_value = 0;

  switch (active_event->eventCode()) {
    case DIGITABULUM_MSG_PMU_READ:
      return_value++;
      break;
    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  flushLocalLog();
  return return_value;
}


#ifdef MANUVR_CONSOLE_SUPPORT
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
      if (temp_int) {
        _periodic_pmu_read.alterSchedulePeriod(temp_int * 10);
        local_log.concatf("_periodic_pmu_read set to %d ms period.\n", (temp_int * 10));
      }
      _periodic_pmu_read.enableSchedule(*(str) == 'P');
      local_log.concatf("%s _periodic_pmu_read.\n", (*(str) == 'p' ? "Stopping" : "Starting"));
      break;

    case 'e':
      // Read the present battery voltage.
      break;

    case 'm':   // Set the system-wide power mode.
      if (255 != temp_int) {
        ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_SYS_POWER_MODE, this);
        event->addArg((uint8_t) temp_int);
        EventReceiver::raiseEvent(event);
        local_log.concatf("Power mode is now %d.\n", temp_int);
      }
      else {
      }
      break;

    case 'd':
      switch (temp_int) {
        case 1:
          local_log.concat("Refreshing _bq24155.\n");
          _bq24155->refresh();
          break;
        case 2:
          local_log.concat("Refreshing _ltc294x.\n");
          _ltc294x->refresh();
          break;
        default:
          local_log.concat("Refreshing all.\n");
          _bq24155->refresh();
          _ltc294x->refresh();
          break;
      }
      break;

    case 'D':
      switch (temp_int) {
        case 1:
          _bq24155->printRegisters(&local_log);
          break;
        case 2:
          _ltc294x->printRegisters(&local_log);
          break;
      }
      break;

    case 'i':
      switch (temp_int) {
        case 1:
          _bq24155->printDebug(&local_log);
          break;
        case 2:
          _ltc294x->printDebug(&local_log);
          break;
      }
      break;

    case 'a':
      switch (temp_int) {
        case 0:
          break;
        case 1:
          _ltc294x->setVoltageThreshold(3.3f, 4.4f);
          break;
        case 2:
          _ltc294x->init();
          break;
        case 3:
          _ltc294x->sleep(true);
          break;
        case 4:
          _ltc294x->sleep(false);
          break;
        case 5:
          break;
      }
      break;

    case 'A':
      _ltc294x->regRead(temp_int);
      break;

    case 'B':
      _bq24155->regRead(temp_int);
      break;

    default:
      EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  flushLocalLog();
}
#endif  // MANUVR_CONSOLE_SUPPORT
