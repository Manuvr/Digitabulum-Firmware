/*
File:   ExpansionPort.h
Author: J. Ian Lindsay
Date:   2016.05.26

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


#ifndef __DIGITABULUM_EXP_PORT_H__
#define __DIGITABULUM_EXP_PORT_H__

#include <Kernel.h>


class ExpansionPort : public EventReceiver {
  public:
    ExpansionPort();
    virtual ~ExpansionPort();

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    void printDebug(StringBuilder*);
    #if defined(MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //MANUVR_CONSOLE_SUPPORT

    volatile static ExpansionPort* INSTANCE;


  protected:
    int8_t attached();


  private:
    void gpioSetup();
};


#endif  // __DIGITABULUM_EXP_PORT_H__
