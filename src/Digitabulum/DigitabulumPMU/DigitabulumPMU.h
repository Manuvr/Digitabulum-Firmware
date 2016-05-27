/*
File:   DigitabulumPMU.h
Author: J. Ian Lindsay
Date:   2015.11.25

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


Our goal is to encapsulate power-supply concerns to this class. This
  class may later be used as the prototype for a generalized PMU kernel
  driver. Until then, we will hard-code i2c addresses, pin numbers, etc.

Digitabulum's power-management subsystem consists of...
  * A TI INA219 (Voltage monitor and consumption rate)
  * A Microchip MCP73833 (LiPo charge management)

*/

#ifndef __DIGITABULUM_PMU_DRIVER_H__
#define __DIGITABULUM_PMU_DRIVER_H__

#include <Kernel.h>

#include <Drivers/INA219/INA219.h>
#include <Drivers/MCP73833/MCP73833.h>

// Valid CPU frequencies.
enum class CPUFreqSetting {
  CPU_54,
  CPU_216,
  CPU_CLK_UNDEF
};


class PMU : public EventReceiver {
  public:
    PMU();
    ~PMU();

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable *);
    void procDirectDebugInstruction(StringBuilder *);
    const char* getReceiverName();
    void printDebug(StringBuilder*);

    static volatile PMU *INSTANCE;
    static int pmu_cpu_clock_rate(CPUFreqSetting);


  protected:
    int8_t bootComplete();


  private:
    CPUFreqSetting _cpu_clock;

    void gpioSetup();
    int8_t cpu_scale(uint8_t _freq);
};

#endif //__DIGITABULUM_PMU_DRIVER_H__
