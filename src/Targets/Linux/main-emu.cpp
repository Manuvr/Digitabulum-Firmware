/*
File:   main-emu.cpp
Author: J. Ian Lindsay
Date:   2016.12.03

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

Intended target is 32-bit linux.

This is the firmware emulation test-bench.
*/

#include <Platform/Platform.h>
#include <Kernel.h>
#include <Platform/Peripherals/I2C/I2CAdapter.h>
#include <Drivers/ADP8866/ADP8866.h>
#include <Drivers/ATECC508/ATECC508.h>
#include <Drivers/PMIC/BQ24155/BQ24155.h>
#include <Drivers/PMIC/LTC294x/LTC294x.h>
#include <XenoSession/Console/ManuvrConsole.h>
#include <XenoSession/Manuvr/ManuvrSession.h>

#include <Transports/ManuvrSocket/ManuvrTCP.h>
#include <Transports/StandardIO/StandardIO.h>

#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/ManuLegend/ManuManager.h"
#include "Digitabulum/DigitabulumPMU/DigitabulumPMU-r2.h"

/* This global makes this source file read better. */
Kernel* kernel = nullptr;


/*******************************************************************************
* BufferPipe strategies particular to this firmware.                           *
*******************************************************************************/
BufferPipe* _pipe_factory_2(BufferPipe* _n, BufferPipe* _f) {
  ManuvrSession* _ses = new ManuvrSession(_n);
  kernel->subscribe(_ses);
  return (BufferPipe*) _ses;
}


/*
* Pin defs
*/
const CPLDPins cpld_pins(
  255,  // CPLD's reset pin
  255,  // Transfer request
  255,  // CPLD's IRQ_WAKEUP pin
  255,  // CPLD clock input
  255,  // CPLD OE pin
  255,  // CPLD GPIO
  255,  // SPI1 CS
  255,  // SPI1 CLK
  255,  // SPI1 MOSI
  255,  // SPI1 MISO
  255,  // SPI2 CS
  255,  // SPI2 CLK
  255   // SPI2 MOSI
);

const I2CAdapterOptions i2c_opts(
  1,   // Device number
  255, // sda
  255  // scl
);

const ATECC508Opts atecc_opts(
  (uint8_t) 255
);

const ADP8866Pins adp_opts(
  24,   // Reset
  255   // IRQ
);

const BatteryOpts battery_opts (
  1400,    // Battery capacity (in mAh)
  3.60f,   // Battery dead (in volts)
  3.70f,   // Battery weak (in volts)
  4.15f,   // Battery float (in volts)
  4.2f     // Battery max (in volts)
);

const LTC294xOpts gas_gauge_opts(
  255,    // N/A (Alert pin)
  LTC294X_OPT_ACD_AUTO | LTC294X_OPT_INTEG_SENSE
);

const BQ24155Opts charger_opts(
  68,   // Sense resistor is 68 mOhm.
  255,  // N/A (STAT)
  255   // N/A (ISEL)
);

const PowerPlantOpts powerplant_opts(
  22,  // 2.5v select pin.
  23,  // Aux regulator enable pin.
  DIGITAB_PMU_FLAG_ENABLED  // Regulator enabled @3.3v
);


/*******************************************************************************
* The main function.                                                           *
*******************************************************************************/
int main(int argc, const char *argv[]) {
  Argument* opts = parseFromArgCV(argc, argv);
  Argument* temp_arg = nullptr;

  if (opts) {
    StringBuilder log;
    opts->printDebug(&log);
    printf("%s\n\n", (char*) log.string());
  }

  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform.platformPreInit(opts);
  kernel = platform.kernel();

  CPLDDriver _cpld(&cpld_pins);
  kernel->subscribe(&_cpld);

  ManuManager _legend_manager(&_cpld);
  kernel->subscribe(&_legend_manager);

  I2CAdapter i2c(&i2c_opts);
  kernel->subscribe((EventReceiver*) &i2c);

  ATECC508 atec(&atecc_opts);
  i2c.addSlaveDevice((I2CDevice*) &atec);
  kernel->subscribe((EventReceiver*) &atec);

  // Pins 58 and 63 are the reset and IRQ pin, respectively.
  // This is translated to pins 10 and 13 on PortD.
  ADP8866 leds(&adp_opts);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  kernel->subscribe((EventReceiver*) &leds);

  PMU pmu(&i2c, &charger_opts, &gas_gauge_opts, &powerplant_opts, &battery_opts);
  kernel->subscribe((EventReceiver*) &pmu);

  // Pipe strategy planning...
  const uint8_t pipe_plan_clients[] = {2, 0};

  if (0 != BufferPipe::registerPipe(2, _pipe_factory_2)) {
    printf("Failed to add client connection to the pipe registry.\n");
    exit(1);
  }

  printf("%s: Booting Digitabulum emulator (PID %u)....\n", argv[0], getpid());
  platform.bootstrap();


  #if defined(MANUVR_SUPPORT_TCPSOCKET)
    /*
    * Transports that listen need to be given instructions for building software
    *   pipes up to the application layer.
    * This is how to use pipe-strategies to instance a console session when a
    *   TCP client connects. Get a simulated connection to firmware by running...
    *       nc -t 127.0.0.1 2319
    */
    if (opts) {
      char* addr_str = (char*) "127.0.0.1";
      char* port_str = (char*) "2319";
      opts->getValueAs("tcp-port", &port_str);
      int port_num = atoi(port_str);

      if (0 == opts->getValueAs("tcp-srv", &addr_str)) {
        ManuvrTCP* tcp = new ManuvrTCP((const char*) addr_str, port_num);
        tcp->setPipeStrategy(pipe_plan_clients);
        kernel->subscribe(tcp);
        printf("%s: Listening on %s:%s (TCP)...\n", argv[0], addr_str, port_str);
        tcp->listen();
      }
      else if (0 == opts->getValueAs("tcp-cli", &addr_str)) {
        ManuvrTCP* tcp = new ManuvrTCP((const char*) addr_str, port_num);
        tcp->setPipeStrategy(pipe_plan_clients);
        kernel->subscribe(tcp);
        printf("%s: Connecting to %s:%s (TCP)...\n", argv[0], addr_str, port_str);
        tcp->connect();
      }
    }
  #endif


  //platform.forsakeMain();
  while (true) {
    kernel->procIdleFlags();
  }
  return 0;
}
