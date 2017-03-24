/*
File:   main.cpp
Author: J. Ian Lindsay
Date:   2017.02.09

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

    .__       ,    .     .
    |  \* _ *-+- _.|_ . .|. .._ _
    |__/|(_]| | (_][_)(_||(_|[ | )
         ._|

Intended target is an WROOM32 SoC module.
*/

#include <Kernel.h>
#include <Platform/Platform.h>
#include <Platform/Peripherals/I2C/I2CAdapter.h>
#include <Drivers/ADP8866/ADP8866.h>
#include <Drivers/ATECC508/ATECC508.h>
#include <XenoSession/Console/ManuvrConsole.h>

#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/ManuLegend/ManuManager.h"

#ifdef __cplusplus
  extern "C" {
#endif



/*
* Pin defs given here assume a WROOM32 module.
*/
const CPLDPins cpld_pins(
  15,  // Reset
  18,  // Transfer request
  255, // CPLD's IRQ_WAKEUP pin
  25,  // CPLD clock input
  255, //22,  // CPLD OE pin
  255, //23,  // CPLD GPIO
  19,  // SPI1 CS
  23,  // SPI1 CLK
  22,  // SPI1 MOSI
  21,  // SPI1 MISO
  33,  // IO33 (input-only) (SPI2 CS)
  34,  // IO34 (input-only) (SPI2 CLK)
  35   // IO35 (input-only) (SPI2 MOSI)
);


const I2CAdapterOptions i2c_opts(
  0,   // Device number
  14,  // IO14 (sda)
  13   // IO13 (scl)
);

const ATECC508Opts atecc_opts(
  (uint8_t) 0
);

const ADP8866Pins adp_opts(
  27,  // IO27 (Reset)
  12   // IO12 (IRQ)
);

/*
Pins
-------------
IO12
IO13
IO14
IO15
IO16
IO17
IO18
IO19
IO21
IO22
IO23
IO25
IO26
IO27
IO32
IO33
IO34
IO35
*/

#if defined (__MANUVR_FREERTOS)
/****************************************************************************************************
* Main function                                                                                     *
****************************************************************************************************/
void app_main() {
  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform.platformPreInit();
  Kernel* kernel = platform.kernel();

  CPLDDriver _cpld(&cpld_pins);
  kernel->subscribe(&_cpld);

  ManuManager _legend_manager(&_cpld);
  kernel->subscribe(&_legend_manager);

  I2CAdapter i2c(&i2c_opts);
  kernel->subscribe(&i2c);

  ATECC508 atec(&atecc_opts);
  i2c.addSlaveDevice((I2CDevice*) &atec);
  kernel->subscribe((EventReceiver*) &atec);

  ADP8866 leds(&adp_opts);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  kernel->subscribe((EventReceiver*) &leds);

  platform.bootstrap();

  gpioDefine(26, GPIOMode::OUTPUT);

  unsigned long ms_0 = millis();
  unsigned long ms_1 = ms_0;
  bool odd_even = false;

  while (1) {
    kernel->procIdleFlags();
    ms_1 = millis();
    kernel->advanceScheduler(ms_1 - ms_0);
    ms_0 = ms_1;
    setPin(26, odd_even);
    odd_even = !odd_even;
  }
}
#endif  // __MANUVR_FREERTOS



#ifdef __cplusplus
  }
#endif
