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
#include <XenoSession/Console/ManuvrConsole.h>

#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/ManuLegend/ManuManager.h"

#ifdef __cplusplus
  extern "C" {
#endif


/* This global makes this source file read better. */
Kernel* kernel = nullptr;

volatile void _hack_sadvance() {
  if (kernel) kernel->advanceScheduler();
}


/*
* Pin defs for this module.
*
* These Port B pins are push-pull outputs:
* #  Default  r1  Purpose
* -------------------------------------------------
* 9     0     25  ~CPLD Reset
* 14    0     30  SPI2_MISO  (SPI2 is slave and Rx-only)
*
* These Port C pins are inputs with a wakeup ISR attached to
*    the rising-edge.
* #  Default  r1  Purpose
* ---------------------------------------------------
* 13    0     45  IRQ_WAKEUP
*
* These Port E pins are inputs:
* #  Default  r1  Purpose
* ---------------------------------------------------
* 11    0     75  CPLD_GPIO_0
* 14    0     78  CPLD_GPIO_1
*
* These Port C pins are push-pull outputs:
* #  Default  r1  Purpose
* ---------------------------------------------------
* 2     1     33  DEN_AG_CARPALS
*/
const CPLDPins cpld_pins(
  25, // CPLD's reset pin
  30, // AKA: SPI2_MISO
  45, // CPLD's IRQ_WAKEUP pin
  75, // GPIO
  78, // GPIO
  33 // The DEN_AG pin on the carpals IMU.
);

const I2CAdapterOptions i2c_opts(
  1,   // Device number
  23, // sda
  22  // scl
);



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
  kernel = platform.kernel();

  CPLDDriver _cpld(&cpld_pins);
  kernel->subscribe(&_cpld);

  ManuManager _legend_manager(&_cpld);
  kernel->subscribe(&_legend_manager);

  I2CAdapter i2c(&i2c_opts);
  kernel->subscribe(&i2c);

  // Pins 58 and 63 are the reset and IRQ pin, respectively.
  // This is translated to pins 10 and 13 on PortD.
  ADP8866 leds(58, 63, 0x27);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  kernel->subscribe((EventReceiver*) &leds);

  platform.bootstrap();
}

#ifdef __cplusplus
  }
#endif
