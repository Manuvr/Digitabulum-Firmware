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
  17,  // CPLD's reset pin
  18,  // Transfer request
  19,  // CPLD's IRQ_WAKEUP pin
  21,  // CPLD clock input
  22,  // CPLD OE pin
  23,  // CPLD GPIO
  25,  // SPI1 CS
  26,  // SPI1 CLK
  27,  // SPI1 MOSI
  32,  // SPI1 MISO
  33,  // SPI2 CS
  34,  // SPI2 CLK
  35   // SPI2 MOSI
);


const I2CAdapterOptions i2c_opts(
  0,   // Device number
  13,  // IO13 (sda)
  14   // IO14 (scl)
);

/*
Pins
-------------
IO13  // i2c
IO14  // i2c
IO15  // LED
IO16  // LED
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
void loopTask(void *pvParameters) {
  printf("******************* loopTask()\n");
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

  //I2CAdapter i2c(&i2c_opts);
  //kernel->subscribe(&i2c);

  //ADP8866 leds(15, 16);
  //i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  //kernel->subscribe((EventReceiver*) &leds);

  printf("******************* micros()\t %lu\n", micros());
  printf("******************* millis()\t %lu\n", millis());
  platform.bootstrap();
  printf("******************* bootstrap()\n");

  while (1) {
    kernel->procIdleFlags();
    if (0 == millis() % 5000) {
      StringBuilder local_log;
      kernel->printDebug(&local_log);
      printf("%s\n", local_log.string());
    }
  }
}


/****************************************************************************************************
* Main function                                                                                     *
****************************************************************************************************/
void app_main() {
  xTaskCreatePinnedToCore(loopTask, "loopTask", 32768, NULL, 1, NULL, 1);
}

#endif  // __MANUVR_FREERTOS



#ifdef __cplusplus
  }
#endif
