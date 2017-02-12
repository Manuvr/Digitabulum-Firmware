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
#include <XenoSession/Console/ManuvrConsole.h>
#include <XenoSession/Manuvr/ManuvrSession.h>

#include <Transports/ManuvrSocket/ManuvrTCP.h>
#include <Transports/StandardIO/StandardIO.h>
#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/ManuLegend/ManuManager.h"
#include "Digitabulum/DigitabulumPMU/DigitabulumPMU.h"


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
  255, // CPLD's reset pin
  255, // AKA: SPI2_MISO
  255, // CPLD's IRQ_WAKEUP pin
  255, // GPIO
  255, // GPIO
  255 // The DEN_AG pin on the carpals IMU.
);

const I2CAdapterOptions i2c_opts(
  0,   // Device number
  255, // sda
  255  // scl
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
    printf("%s\n\n\n", (char*) log.string());
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

  // Pins 58 and 63 are the reset and IRQ pin, respectively.
  // This is translated to pins 10 and 13 on PortD.
  ADP8866 leds(58, 63, 0x27);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  kernel->subscribe((EventReceiver*) &leds);

  INA219 ina219(0x4A);
  i2c.addSlaveDevice(&ina219);

  // Pipe strategy planning...
  const uint8_t pipe_plan_clients[] = {2, 0};

  if (0 != BufferPipe::registerPipe(2, _pipe_factory_2)) {
    printf("Failed to add client connection to the pipe registry.\n");
    exit(1);
  }


  #if defined(MANUVR_SUPPORT_TCPSOCKET)
    /*
    * Transports that listen need to be given instructions for building software
    *   pipes up to the application layer.
    * This is how to use pipe-strategies to instance a console session when a
    *   TCP client connects. Get a simulated connection to firmware by running...
    *       nc -t 127.0.0.1 2319
    */
    ManuvrTCP tcp_srv((const char*) "0.0.0.0", 2319);
    tcp_srv.setPipeStrategy(pipe_plan_clients);
    kernel->subscribe(&tcp_srv);
  #endif

  printf("%s: Booting Digitabulum emulator (PID %u)....\n", argv[0], getpid());
  platform.bootstrap();

  #if defined(MANUVR_SUPPORT_TCPSOCKET)
    tcp_srv.listen();
  #endif

  //platform.forsakeMain();
  while (true) {
    kernel->procIdleFlags();
  }
  return 0;
}
