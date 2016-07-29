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


This driver allows us to treat an STDIO set under linux as if it were
  any other transport. It should be extendable to any situation where
  there exists a physical UI on the platform, and it may therefore be
  renamed and further extended in the future.
*/


#ifndef __MANUVR_STM32F7USB_H__
#define __MANUVR_STM32F7USB_H__

#define MANUVR_USB_BUF_SIZE  255


#include <Transports/ManuvrXport.h>


class STM32F7USB : public ManuvrXport {
  public:
    STM32F7USB();
    ~STM32F7USB();

    /* Override from BufferPipe. */
    virtual int8_t toCounterparty(StringBuilder*, int8_t mm);
    virtual int8_t toCounterparty(uint8_t* buf, unsigned int len, int8_t mm);
    virtual int8_t fromCounterparty(uint8_t* buf, unsigned int len, int8_t mm);

    /* Overrides from EventReceiver */
    int8_t bootComplete();
    const char* getReceiverName();
    void printDebug(StringBuilder *);
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable*);


    int8_t connect();
    int8_t disconnect();
    int8_t listen();
    int8_t reset();

    int8_t read_port();
    bool   write_port(uint8_t* out, int out_len);

    volatile static STM32F7USB* INSTANCE;


  protected:
    void __class_initializer();


  private:
};

#endif   // __MANUVR_STM32F7USB_H__
