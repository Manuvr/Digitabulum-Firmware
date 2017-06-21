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

#include <Platform/Platform.h>
#include <Platform/Peripherals/I2C/I2CAdapter.h>
#include <Drivers/ADP8866/ADP8866.h>
#include <Drivers/ATECC508/ATECC508.h>
#include <Drivers/PMIC/BQ24155/BQ24155.h>
#include <Drivers/PMIC/LTC294x/LTC294x.h>
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
  26,  // Reset
  21,  // Transfer request
  255, // IO33 (input-only) CPLD's IRQ_WAKEUP pin
  2,   // CPLD clock input
  25,  // CPLD OE pin
  255, // N/A (CPLD GPIO)
  5,   // SPI1 CS
  17,  // SPI1 CLK
  16,  // SPI1 MOSI
  4,   // SPI1 MISO
  35,  // IO35 (SPI2 CS)
  34,  // IO34 (input-only) (SPI2 CLK)
  32   // IO32 (input-only) (SPI2 MOSI)
);


const I2CAdapterOptions i2c_opts(
  0,   // Device number
  14,  // IO14 (sda)
  12   // IO12 (scl)
);

const ATECC508Opts atecc_opts(
  (uint8_t) 255
);

const ADP8866Pins adp_opts(
  19,  // IO19 (Reset)
  18   // IO18 (IRQ)
);

const LTC294xOpts gas_gauge_opts(
  13,     // IO13 (Alert pin)
  2600    // We will assume a common 18650 for now. 2600mAh capacity.
);

const BQ24155Opts charger_opts(
  68,  // Sense resistor is 68 mOhm.
  255, // N/A (STAT)
  23   // IO23 (ISEL)
);

#define ESP32_LED_PIN             15  // This is an LED.
#define ESP32_AUX_REGULATOR_PIN   27

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

  BQ24155 charger(&charger_opts);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &charger);
  kernel->subscribe((EventReceiver*) &charger);

  LTC294x gas_gauge(&gas_gauge_opts);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &gas_gauge);

  platform.bootstrap();


  // This is the ~SHUTDOWN signal to the secondary regulator. Setting it high
  //   will cause the CPLD board to be powered at 3.3v.
  gpioDefine(ESP32_AUX_REGULATOR_PIN, GPIOMode::OUTPUT);
  setPin(ESP32_AUX_REGULATOR_PIN, true);

  unsigned long ms_0 = millis();
  unsigned long ms_1 = ms_0;
  bool odd_even = false;

  gpioDefine(ESP32_LED_PIN, GPIOMode::OUTPUT);

  while (1) {
    kernel->procIdleFlags();
    ms_1 = millis();
    kernel->advanceScheduler(ms_1 - ms_0);
    ms_0 = ms_1;
    setPin(ESP32_LED_PIN, odd_even);
    odd_even = !odd_even;
  }
}
#endif  // __MANUVR_FREERTOS



#ifdef __cplusplus
  }
#endif
