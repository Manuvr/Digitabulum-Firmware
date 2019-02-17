/*
File:   Digitabulum.cpp
Author: J. Ian Lindsay
Date:   2019.02.10

Copyright 2019 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


This class is intended to be functional glue-logic abstracted from platform.
This way, we don't have to write application logic for every supported platform.

*/


#include "Digitabulum.h"
#include <Kernel.h>
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

Digitabulum* Digitabulum::INSTANCE = nullptr;

/* These options are fixed with respect to the application layer. */
const ATECC508Opts atecc_opts(
  (uint8_t) 255
);



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/
/*
* Constructor.
*/
Digitabulum::Digitabulum(I2CAdapter* i2c_adapter, const DigitabulumOpts* _o) :
    EventReceiver("Digitabulum"),
    leds(_o->adp_pins),
    cpld(_o->cpld_pins),
    manu(&cpld),
    atec(&atecc_opts),
    _opts(_o) {
  if (nullptr == Digitabulum::INSTANCE) {
    Digitabulum::INSTANCE = this;
  }
  Kernel* kernel = platform.kernel();

  kernel->subscribe(&leds);
  kernel->subscribe(&cpld);
  kernel->subscribe(&manu);
  i2c_adapter->addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  i2c_adapter->addSlaveDevice((I2CDevice*) &atec);
}


/*
* Destructor.
*/
Digitabulum::~Digitabulum() {
}


int8_t Digitabulum::init() {
  return 0;
}


/*
* Dump this item to the dev log.
*/
void Digitabulum::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
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
int8_t Digitabulum::attached() {
  if (EventReceiver::attached()) {
    //led_wrist_color(0x00000010);
    return 1;
  }
  return 0;
}


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
int8_t Digitabulum::callback_proc(ManuvrMsg* event) {
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


int8_t Digitabulum::notify(ManuvrMsg* active_event) {
  int8_t return_value = 0;

  switch (active_event->eventCode()) {
    case DIGITABULUM_MSG_CPLD_RESET_COMPLETE:
      //led_wrist_color(0x00050005);
      break;
    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  flushLocalLog();
  return return_value;
}


#if defined(MANUVR_CONSOLE_SUPPORT)
/*******************************************************************************
* Console I/O
*******************************************************************************/

static const ConsoleCommand console_cmds[] = {
  { "r", "Reset" }
};


uint Digitabulum::consoleGetCmds(ConsoleCommand** ptr) {
  *ptr = (ConsoleCommand*) &console_cmds[0];
  return sizeof(console_cmds) / sizeof(ConsoleCommand);
}


void Digitabulum::consoleCmdProc(StringBuilder* input) {
  const char* str = (char *) input->position(0);
  char c    = *str;
  int temp_int = 0;

  if (input->count() > 1) {
    temp_int = input->position_as_int(1);
  }

  switch (c) {
    case 'i':   // Debug prints.
      switch (temp_int) {
        case 5:
          local_log.concatf("\nsizeof(Digitabulum):   %u\n", sizeof(Digitabulum));
          local_log.concatf("  sizeof(ATECC508):    %u\n", sizeof(ATECC508));
          local_log.concatf("  sizeof(CPLDDriver):  %u\n", sizeof(CPLDDriver));
          local_log.concatf("  sizeof(ManuManager): %u\n", sizeof(ManuManager));
          local_log.concatf("  sizeof(ADP8866):     %u\n", sizeof(ADP8866));
          break;
        default:
          printDebug(&local_log);
          break;
      }
      break;

    case 'r':
      reset();
      break;

    case 'v':
      // The verbosity level given here will be propagated downward to all
      //   components involved with the sensor front-end board.
      break;

    default:
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT



/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

/*
* Perform a software reset.
*/
void Digitabulum::reset() {
  cpld.reset();
  //leds.reset();
}


int8_t Digitabulum::led_set_digit_brightness(DigitPort p, uint8_t brightness) {
  leds.set_brightness((uint8_t) p, brightness);
  return 0;
}

int8_t Digitabulum::led_wrist_color(uint8_t r, uint8_t g, uint8_t b) {
  leds.set_brightness(7, b);
  leds.set_brightness(8, r);
  leds.set_brightness(9, g);
  return 0;
}

int8_t Digitabulum::led_wrist_color(uint32_t color) {
  return led_wrist_color(
    (uint8_t) ((color >> 16) & 0xFF),
    (uint8_t) ((color >> 8) & 0xFF),
    (uint8_t) (color & 0xFF)
  );
}
