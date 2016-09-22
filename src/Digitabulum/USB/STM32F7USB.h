/*
File:   STM32F7USB.h
Author: J. Ian Lindsay
Date:   2016.07.29

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


This is the transport driver for the USB peripheral on the STM32F7.
*/


#ifndef __MANUVR_STM32F7USB_H__
#define __MANUVR_STM32F7USB_H__

#define MANUVR_USB_BUF_SIZE  255


extern "C" {
  #if defined(ENABLE_USB_VCP)
    #include "tm_stm32_usb_device.h"
    #include "tm_stm32_usb_device_cdc.h"
  #endif
}

#include <Transports/ManuvrXport.h>


class STM32F7USB : public ManuvrXport {
  public:
    STM32F7USB();
    ~STM32F7USB();

    /* Override from BufferPipe. */
    virtual int8_t toCounterparty(StringBuilder*, int8_t mm);

    /* Overrides from EventReceiver */
    int8_t bootComplete();
    void printDebug(StringBuilder *);
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable*);

    /* Overrides from ManuvrXport */
    int8_t connect();
    int8_t disconnect();
    int8_t listen();
    int8_t reset();

    virtual int8_t read_port();
    bool   write_port(uint8_t* out, int out_len);

    volatile static STM32F7USB* INSTANCE;


  protected:

  private:
};

#endif   // __MANUVR_STM32F7USB_H__
