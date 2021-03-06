/*
File:   Digitabulum.h
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

*/


#ifndef __DIGITABULUM_INC_H_
#define __DIGITABULUM_INC_H_

#include <inttypes.h>
#include <stdint.h>
#include <Kernel.h>
#include <Drivers/ADP8866/ADP8866.h>
#include <Drivers/ATECC508/ATECC508.h>
#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/ManuLegend/ManuManager.h"
#include "Digitabulum/DigitabulumPMU/DigitabulumPMU-r2.h"

#ifdef MANUVR_CONSOLE_SUPPORT
  #include <XenoSession/Console/ManuvrConsole.h>
#endif

/* Callback for frame completion. */
typedef int8_t (*DigitabulumFrameFxnPtr)(ManuLegendPipe*);


/*
* Pin defs for this module.
* Set pin def to 255 to mark it as unused.
*/
class DigitabulumOpts {
  public:
    const uint32_t     flags;  // Flags
    const CPLDPins*    cpld_pins;
    const ADP8866Pins* adp_pins;

    DigitabulumOpts(const DigitabulumOpts* p) :
      flags(p->flags),
      cpld_pins(p->cpld_pins),
      adp_pins(p->adp_pins) {};

    DigitabulumOpts(const CPLDPins* _cp, const ADP8866Pins* _ap, uint16_t _f) :
      flags(_f),
      cpld_pins(_cp),
      adp_pins(_ap) {};

  private:
};



#ifdef MANUVR_CONSOLE_SUPPORT
class Digitabulum : public EventReceiver, public ConsoleInterface {
#else
class Digitabulum : public EventReceiver {
#endif
  public:
    Digitabulum(I2CAdapter*, PMU*, const DigitabulumOpts*);
    virtual ~Digitabulum();

    int8_t init();

    #ifdef MANUVR_CONSOLE_SUPPORT
      /* Overrides from ConsoleInterface */
      uint consoleGetCmds(ConsoleCommand**);
      inline const char* consoleName() { return getReceiverName();  };
      void consoleCmdProc(StringBuilder* input);
    #endif  //MANUVR_CONSOLE_SUPPORT

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    void printDebug(StringBuilder*);

    /* Application level control fxns regarding cryptography */
    int8_t authenticateHardware(StringBuilder*);

    /* Application level control fxns regarding data reporting */

    /* Application level control fxns regarding hardware management */
    void reset();


    static Digitabulum* INSTANCE;
    static DigitabulumFrameFxnPtr frame_cb;


  protected:
    int8_t attached();


  private:
    ADP8866     leds;
    CPLDDriver  cpld;
    ManuManager manu;
    ATECC508    atec;
    const PMU*  pmu;
    const DigitabulumOpts _opts;
    ManuLegendPipe _def_pipe;     // Data demand from the host.

    /* LED indicator functions */
    int8_t led_set_digit_brightness(DigitPort, uint8_t);
    int8_t led_wrist_color(uint8_t r, uint8_t g, uint8_t b);
    int8_t led_wrist_color(uint32_t);
    int8_t indicate_error(uint8_t, uint16_t);
    int8_t indicate_sleep();
    int8_t indicate_reading();
};

#endif  // __DIGITABULUM_INC_H_
