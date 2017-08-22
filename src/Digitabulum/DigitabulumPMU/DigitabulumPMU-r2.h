/*
File:   DigitabulumPMU-r2.h
Author: J. Ian Lindsay
Date:   2017.06.31

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
*/

#ifndef __DIGITABULUM_PMU_DRIVER_H__
#define __DIGITABULUM_PMU_DRIVER_H__

#include <Kernel.h>
#include <Drivers/PMIC/BQ24155/BQ24155.h>
#include <Drivers/PMIC/LTC294x/LTC294x.h>

/*
* These state flags are hosted by the EventReceiver. This may change in the future.
* Might be too much convention surrounding their assignment across inherritence.
*/
#define DIGITAB_PMU_FLAG_STAT1      0x01    // The state of STAT1.
#define DIGITAB_PMU_FLAG_STAT2      0x02    // The state of STAT2.

#define DIGITABULUM_MSG_PMU_READ    0x5233  //


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
void bq24155_stat_isr();
void ltc294x_alert_isr();


class PMU : public EventReceiver {
  public:
    PMU(BQ24155*, LTC294x*);
    virtual ~PMU();

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    void printDebug(StringBuilder*);
    #if defined(MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //MANUVR_CONSOLE_SUPPORT

    /* These are called by ISR to keep track of the STAT pin timings. */

    ChargeState getChargeState();

    /* Inlines for object-style usage of static functions... */
    inline const char* getChargeStateString() {  return getChargeStateString(_charge_state); };

    static volatile PMU *INSTANCE;


  protected:
    int8_t attached();


  private:
    uint32_t     _cpu_clock_rate;
    ManuvrMsg _periodic_pmu_read;

    /* Values for the MCP73833 charge controller. */
    unsigned int _stat1_delta;
    unsigned int _stat2_delta;
    uint8_t      _stat1_pin;
    uint8_t      _stat2_pin;

    CPUFreqSetting _cpu_clock;
    ChargeState    _charge_state = ChargeState::UNDEF;

    BQ24155*     _bq24155;
    LTC294x*     _ltc294x;

    int8_t cpu_scale(uint8_t _freq);

    static const char* getChargeStateString(ChargeState);
    static int pmu_cpu_clock_rate(CPUFreqSetting);
};

#endif //__DIGITABULUM_PMU_DRIVER_H__
