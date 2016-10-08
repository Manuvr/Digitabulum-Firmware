/*
File:   SDCard.h
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

#ifndef __DIGITABULUM_SDCARD_H__
#define __DIGITABULUM_SDCARD_H__

#include <inttypes.h>
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_sd.h>

#include <Kernel.h>

/*
* This class represents an open comm session with a foreign device. That comm session might
*   be happening over USB, BlueTooth, WiFi, IRDa, etc. All we care about is the byte stream.
* Transport class instantiates us, and maintains a pointer to us.
*/
class SDCard : public EventReceiver {
  public:
    SDCard();
    ~SDCard();


    int8_t init(void);

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable *);
    const char* getReceiverName();
    void printDebug(StringBuilder*);
    #if defined(MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //MANUVR_CONSOLE_SUPPORT


  protected:
    int8_t attached();


  private:
    uint8_t buf_in[512];
    uint8_t buf_out[512];

    void gpioSetup();
};


#endif
