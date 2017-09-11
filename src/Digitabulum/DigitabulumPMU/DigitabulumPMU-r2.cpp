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

/**
* Constructor. All params are required.
*/
PMU::PMU(BQ24155* charger, LTC294x* fuel_gauge, const PowerPlantOpts* o, const BatteryOpts* bo) :
  EventReceiver("PMU"),
  _opts(o), _battery(bo),
  _bq24155(charger), _ltc294x(fuel_gauge) {
  _er_set_flag(_opts.flags);    // Set the initial flag state.
  if (nullptr == INSTANCE) {
    INSTANCE   = this;
    ManuvrMsg::registerMessages(pmu_message_defs, sizeof(pmu_message_defs) / sizeof(MessageTypeDef));
  }

  // For safety's sake, this pin init order is important.
  // Flags are reliable at this point. So if the caller set flags indicating
  //   a given initial state for the auxililary regulator, it will be honored
  //   if possible.
  // If no flags are given, the default behavior is for the aux regulator
  //   to remain powered down, with 3.3v as the default power output when it is
  //   enabled.
  // If a flag is set a certain way, but no pin control is possible, the flags
  //   related to that feature effectively become constants, and we will trust
  //   that they reflect the state of the hardware.
  if (_opts.useVSPin()) {
    gpioDefine(_opts.vs_pin, GPIOMode::OUTPUT);
    setPin(_opts.vs_pin, !auxRegLowPower());
  }
  if (_opts.useREPin()) {
    gpioDefine(_opts.re_pin, GPIOMode::OUTPUT);
    setPin(_opts.re_pin, auxRegEnabled());
  }

  // Now... we have battery details. So we derive some settings for the two
  //   I2C chips we are dealing with.
  _bq24155->batt_reg_voltage(_battery.voltage_max);
  _bq24155->batt_weak_voltage(_battery.voltage_weak);

  // We'll probably want to check the PMIC periodically. Setup the schedule.
  _periodic_pmu_read.repurpose(DIGITABULUM_MSG_PMU_READ, (EventReceiver*) this);
  _periodic_pmu_read.incRefs();
  _periodic_pmu_read.specific_target = (EventReceiver*) this;
  _periodic_pmu_read.priority(1);
  _periodic_pmu_read.alterSchedulePeriod(1000);
  _periodic_pmu_read.alterScheduleRecurrence(-1);
  _periodic_pmu_read.autoClear(false);
  _periodic_pmu_read.enableSchedule(false);
}


PMU::~PMU() {
  _periodic_pmu_read.enableSchedule(false);
  platform.kernel()->removeSchedule(&_periodic_pmu_read);
}



/*
* Updates the local state if warranted, but always returns local state.
*/
ChargeState PMU::getChargeState() {
  return _charge_state;
}



/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

/*
* Turns the regulator on or off.
*
* @return non-zero on error.
*/
int8_t PMU::auxRegEnabled(bool nu) {
  if (_opts.useREPin()) {
    // Shutdown is achieved by pulling pin low.
    setPin(_opts.re_pin, nu);
    _er_set_flag(DIGITAB_PMU_FLAG_ENABLED, nu);
    return 0;
  }
  return -1;
}


/*
* Sets the regulator voltage to 2.5v or 3.3v.
*
* @return non-zero on error.
*/
int8_t PMU::auxRegLowPower(bool nu) {
  if (_opts.useVSPin()) {
    // 2.5v mode is selected by pulling pin low.
    setPin(_opts.vs_pin, !nu);
    _er_set_flag(DIGITAB_PMU_FLAG_V_25, nu);
    return 0;
  }
  return -1;
}

/**
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void PMU::printBattery(StringBuilder* output) {
  output->concatf("-- Battery (%.2fV)\n", _ltc294x->batteryVoltage());
  output->concatf("\tCapacity %umAh\n", _battery.capacity);
  output->concatf("\tDead     %.2fV\n", _battery.voltage_min);
  output->concatf("\tWeak     %.2fV\n", _battery.voltage_weak);
  output->concatf("\tFloat    %.2fV\n", _battery.voltage_float);
  output->concatf("\tFull     %.2fV\n", _battery.voltage_max);
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
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void PMU::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
  const char* aux_reg_state = auxRegLowPower() ? "2.5v" : "3.3v";
  output->concatf("-- CPU freq            %.2f MHz\n", (double) platform.cpu_freq()/1000000.0);
  output->concatf("-- VS/RE pins          %u/%u\n", _opts.vs_pin, _opts.re_pin);
  output->concatf("-- Auxiliary regulator %s\n", auxRegEnabled() ? aux_reg_state : "Disabled");
  output->concatf("-- Charge state        %s\n", getChargeStateString());
  _ltc294x->printDebug(output);
  _bq24155->printDebug(output);
}


/**
* This is called when the kernel attaches the module.
* This is the first time the class can be expected to have kernel access.
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t PMU::attached() {
  if (EventReceiver::attached()) {
    platform.kernel()->addSchedule(&_periodic_pmu_read);

    // We want the gas guage to warn us if the voltage leaves the realm of safety.
    _ltc294x->setVoltageThreshold(_battery.voltage_weak, _battery.voltage_max);
    _ltc294x->init();
    _bq24155->init();
    return 1;
  }
  return 0;
}


int8_t PMU::erConfigure(Argument* conf) {
  local_log.concat("PMU::erConfigure(conf)\n");
  flushLocalLog();
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
    case MANUVR_MSG_SYS_POWER_MODE:
      return_value++;
      break;
    case DIGITABULUM_MSG_PMU_READ:
      {
        uint32_t ts = millis();
        _ltc294x->refresh();
        if (ts >= (_punch_timestamp + 29000)) {
          // One every 32 seconds, the charger will stop.
          _bq24155->punch_safety_timer();
        }
        else {
          _bq24155->refresh();
        }
      }
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
  // TODO: This function (and the open-scoping demands it makes on member classes)
  //         is awful. It is hasty until I can learn enough from some other PMU
  //         abstraction to do something more sensible.
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

    case 'm':   // Set the system-wide power mode.
      if (255 != temp_int) {
        ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_SYS_POWER_MODE, this);
        event->addArg((uint8_t) temp_int);
        EventReceiver::raiseEvent(event);
        local_log.concatf("Power mode is now %d.\n", temp_int);
      }
      break;


    ///////////////////////
    // Common
    ///////////////////////
    case 'X':
    case 'x':
      local_log.concatf(
        "%sabling auxiliary regulator... %s\n",
        (*(str) == 'X' ? "En" : "Dis"),
        (0 != auxRegEnabled(*(str) == 'X')) ? "failure" : "success"
      );
      break;

    case 'L':
    case 'l':
      local_log.concatf(
        "Setting auxiliary regulator to %.1fv... %s\n",
        (*(str) == 'L' ? 2.5f : 3.3f),
        (0 != auxRegLowPower(*(str) == 'L')) ? "failure" : "success"
      );
      break;

    case 'i':
      switch (temp_int) {
        case 1:
          _ltc294x->printDebug(&local_log);
          break;
        case 2:
          _bq24155->printDebug(&local_log);
          break;
        case 3:
          _ltc294x->printRegisters(&local_log);
          break;
        case 4:
          _bq24155->printRegisters(&local_log);
          break;
        case 5:
          printBattery(&local_log);
          break;

        default:
          printDebug(&local_log);
      }
      break;

    case 'a':
      switch (temp_int) {
        case 1:
          _ltc294x->init();
          break;
        case 2:
          _bq24155->init();
          break;
        case 3:
          _ltc294x->init();
          _bq24155->init();
          break;
        default:
          local_log.concat("1: _ltc294x->init()\n");
          local_log.concat("2: _bq24155->init()\n");
          break;
      }
      break;

    case 'd':
      switch (temp_int) {
        case 1:
          local_log.concat("Refreshing _ltc294x.\n");
          _ltc294x->refresh();
          break;
        case 2:
          local_log.concat("Refreshing _bq24155.\n");
          _bq24155->refresh();
          break;
        case 3:
          local_log.concat("Refreshing all.\n");
          _bq24155->refresh();
          _ltc294x->refresh();
          break;
        default:
          local_log.concat("1: _ltc294x->refresh()\n");
          local_log.concat("2: _bq24155->refresh()\n");
          break;
      }
      break;

    ///////////////////////
    // Gas guage
    ///////////////////////
    case 's':
      switch (temp_int) {
        case 1:
          _ltc294x->setVoltageThreshold(3.3f, 4.4f);
          break;
        case 2:
          _ltc294x->sleep(true);
          break;
        case 3:
          _ltc294x->sleep(false);
          break;
      }
      break;

    ///////////////////////
    // Charger
    ///////////////////////
    case 'E':
    case 'e':
      local_log.concatf("%sabling battery charge...\n", (*(str) == 'E' ? "En" : "Dis"));
      _bq24155->charger_enabled(*(str) == 'E');
      break;

    case 'T':
    case 't':
      local_log.concatf("%sabling charge current termination...\n", (*(str) == 'T' ? "En" : "Dis"));
      _bq24155->charger_enabled(*(str) == 'T');
      break;

    case '*':
      local_log.concat("Punching safety timer...\n");
      _bq24155->punch_safety_timer();
      break;

    case 'R':
      local_log.concat("Resetting charger parameters...\n");
      _bq24155->reset_charger_params();
      break;

    case 'u':   // USB host current limit.
      if (temp_int) {
        _bq24155->usb_current_limit((unsigned int) temp_int);
      }
      else {
        local_log.concatf("USB current limit: %dmA\n", _bq24155->usb_current_limit());
      }
      break;
    case 'v':   // Battery regulation voltage
      if (temp_int) {
        local_log.concatf("Setting battery regulation voltage to %.2fV\n", input->position_as_double(1));
        _bq24155->batt_reg_voltage(input->position_as_double(1));
      }
      else {
        local_log.concatf("Batt reg voltage:  %.2fV\n", _bq24155->batt_reg_voltage());
      }
      break;
    case 'w':   // Battery weakness voltage
      if (temp_int) {
        local_log.concatf("Setting battery weakness voltage to %.2fV\n", input->position_as_double(1));
        _bq24155->batt_weak_voltage(input->position_as_double(1));
      }
      else {
        local_log.concatf("Batt weakness voltage:  %.2fV\n", _bq24155->batt_weak_voltage());
      }
      break;


    default:
      EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  flushLocalLog();
}
#endif  // MANUVR_CONSOLE_SUPPORT
