/*
File:   host-driver.cpp
Author: J. Ian Lindsay
Date:   2017.08.07

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

This is the demonstation C++ host-side driver.
*/

#include <Platform/Platform.h>
#include <Kernel.h>
#include <XenoSession/Console/ManuvrConsole.h>
#include <XenoSession/Manuvr/ManuvrSession.h>

#include <Transports/ManuvrSocket/ManuvrTCP.h>
#include <Transports/StandardIO/StandardIO.h>


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

  // Pipe strategy planning...
  const uint8_t pipe_plan_clients[] = {2, 0};

  if (0 != BufferPipe::registerPipe(2, _pipe_factory_2)) {
    printf("Failed to add client connection to the pipe registry.\n");
    exit(1);
  }

  printf("%s: Digitabulum host driver (PID %u)....\n", argv[0], getpid());
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
      char* addr_str = "127.0.0.1";
      char* port_str = "2319";
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
