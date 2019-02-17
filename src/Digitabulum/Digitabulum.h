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

#ifdef MANUVR_CONSOLE_SUPPORT
  #include <XenoSession/Console/ManuvrConsole.h>
#endif


/*
* Pin defs for this module.
* Set pin def to 255 to mark it as unused.
*/
class DigitabulumOpts {
  public:
    const uint16_t flags;  // Flags

    DigitabulumOpts(const DigitabulumOpts* p) :
      flags(p->flags) {};

    DigitabulumOpts(uint16_t _f) :
      flags(_f) {};


  private:
};



#ifdef MANUVR_CONSOLE_SUPPORT
class Digitabulum : public EventReceiver, public ConsoleInterface {
#else
class Digitabulum : public EventReceiver {
#endif
  public:
    Digitabulum(I2CAdapter*, const DigitabulumOpts*);
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

    static Digitabulum* INSTANCE;


  protected:
    int8_t attached();


  private:
    const DigitabulumOpts _opts;

    void reset();
};

#endif  // __DIGITABULUM_INC_H_
