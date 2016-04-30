/*
File:   RN4677.h
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

*/


#include "../RNBase.h"

#ifndef __RN4677_H__
#define __RN4677_H__




class RN4677 : public RNBase {

  public:
    RN4677();
    ~RN4677();

    /* Overrides from the Transport class. */


    /* Overrides from EventReceiver */
    //void procDirectDebugInstruction(StringBuilder *);
    const char* getReceiverName();
    //void printDebug(StringBuilder *);
    //int8_t notify(ManuvrRunnable*);
    //int8_t callback_proc(ManuvrRunnable *);

  protected:
    void factoryReset(void);   // Perform the sequence that will factory-reset the RN.
    void gpioSetup(void);
    void force_9600_mode(bool);   // Call with 'true' to force the module into 9600bps.
    void set_bitrate(int);    //


  private:
};


#endif
