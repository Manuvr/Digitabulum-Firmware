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


Regarding the battery in Digitabulum
================================================================================
r1 uses a 1485mAh LiPo.


Regarding the MCP73833 (charge controller)
================================================================================
Any other statX combinations are meaningless because it could mean many things.

stat1 stat2   Condition             This table is complicated by the fact that
---------------------------------     the stat lines were intended to be hooked
  L     H     Charge-in-progress      to LEDs for a human. So they pulse and do
  H     L     Charge complete         other things. For this reason, we track
  L     L     System test mode        the system time of falling edges.


Regarding the INA219 (fuel gauge)
================================================================================
In Digitabulum, the shunt resistor is Yageo part number RL1210FR-070R2L. It is
  0.2-ohm.

*/

#ifndef __DIGITABULUM_PMU_DRIVER_H__
#define __DIGITABULUM_PMU_DRIVER_H__

#include <Kernel.h>
#include <Drivers/INA219/INA219.h>
#include <Drivers/MCP73833/MCP73833.h>

/*
* These state flags are hosted by the EventReceiver. This may change in the future.
* Might be too much convention surrounding their assignment across inherritence.
*/
#define DIGITAB_PMU_FLAG_STAT1      0x01    // The state of STAT1.
#define DIGITAB_PMU_FLAG_STAT2      0x02    // The state of STAT2.


// Valid CPU frequencies.
enum class CPUFreqSetting {
  CPU_27,
  CPU_54,
  CPU_216,
  CPU_CLK_UNDEF
};

enum class ChargeState {
  FULL,      // Implies we are connected to a charging source.
  CHARGING,  // The battery is not full, but it is charging.
  DRAINING,  // No charging source. We are burning fuel..
  TEST,      // System test mode.
  ERROR,     // Charge error. This means a hardware fault. Absent battery?
  UNDEF
};

/* These fxns are out-of-class ISRs. */
void mcp73833_stat1_isr();
void mcp73833_stat2_isr();


class PMU : public EventReceiver {
  public:
    PMU(INA219*);
    ~PMU();

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable *);
    const char* getReceiverName();
    void printDebug(StringBuilder*);
    #if defined(__MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //__MANUVR_CONSOLE_SUPPORT

    /* These are called by ISR to keep track of the STAT pin timings. */

    ChargeState getChargeState();
    void set_stat1_delta(unsigned int);
    void set_stat2_delta(unsigned int);

    /* Inlines for object-style usage of static functions... */
    inline const char* getChargeStateString() {  return getChargeStateString(_charge_state); };

    static volatile PMU *INSTANCE;

  protected:
    int8_t bootComplete();


  private:
    uint32_t     _cpu_clock_rate;
    ManuvrRunnable _periodic_pmu_read;  // Read the INA219 regularly.

    /* Values for the MCP73833 charge controller. */
    unsigned int _stat1_delta;
    unsigned int _stat2_delta;
    uint8_t      _stat1_pin;
    uint8_t      _stat2_pin;

    CPUFreqSetting _cpu_clock;
    ChargeState    _charge_state = ChargeState::UNDEF;

    INA219*      _ina219;

    void gpioSetup();
    int8_t cpu_scale(uint8_t _freq);

    static const char* getChargeStateString(ChargeState);
    static int pmu_cpu_clock_rate(CPUFreqSetting);
};

#endif //__DIGITABULUM_PMU_DRIVER_H__
