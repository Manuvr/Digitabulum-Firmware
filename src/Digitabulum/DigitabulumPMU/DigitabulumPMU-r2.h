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
#define DIGITAB_PMU_FLAG_ENABLED    0x01    // Aux regulator is anabled.
#define DIGITAB_PMU_FLAG_V_25       0x02    // Aux regulator is set to 2.5v.

#define DIGITABULUM_MSG_PMU_READ    0x5233  //


enum class ChargeState {
  FULL,      // Implies we are connected to a charging source.
  CHARGING,  // The battery is not full, but it is charging.
  DRAINING,  // No charging source. We are burning fuel..
  TEST,      // System test mode.
  ERROR,     // Charge error. This means a hardware fault. Absent battery?
  UNDEF
};


/**
* Options for the PowerPlant.
*/
class PowerPlantOpts {
  public:
    const uint8_t vs_pin;  // Which pin is bound to aux voltage select?
    const uint8_t re_pin;  // Which pin is bound to aux regulator enable?
    const uint8_t flags;   // Flags that the class should start with.

    PowerPlantOpts(const PowerPlantOpts* o) :
      vs_pin(o->vs_pin),
      re_pin(o->re_pin),
      flags(o->flags) {};

    PowerPlantOpts(uint8_t _vspin, uint8_t _repin, uint8_t _f) :
      vs_pin(_vspin),
      re_pin(_repin),
      flags(_f) {};

    PowerPlantOpts(uint8_t _vspin, uint8_t _repin) :
      vs_pin(_vspin),
      re_pin(_repin),
      flags(0) {};

    PowerPlantOpts(uint8_t _f) :
      vs_pin(255),
      re_pin(255),
      flags(_f) {};

    inline bool useVSPin() const {
      return (255 != vs_pin);
    };

    inline bool useREPin() const {
      return (255 != re_pin);
    };
};



/* These fxns are out-of-class ISRs. */
//void bq24155_stat_isr();
//void ltc294x_alert_isr();


class PMU : public EventReceiver {
  public:
    PMU(BQ24155*, LTC294x*, const PowerPlantOpts*);
    virtual ~PMU();

    /* Overrides from EventReceiver */
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    void printDebug(StringBuilder*);
    #if defined(MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //MANUVR_CONSOLE_SUPPORT


    ChargeState getChargeState();

    /* Is the aux regulator enabled? */
    inline bool auxRegEnabled() {   return (_er_flag(DIGITAB_PMU_FLAG_ENABLED));  };
    inline bool auxRegLowPower() {  return (_er_flag(DIGITAB_PMU_FLAG_V_25));     };

    /* Control over the auxilary regulator, if the hardware supports it. */
    int8_t auxRegEnabled(bool);
    int8_t auxRegLowPower(bool);


    /* Inlines for object-style usage of static functions... */
    inline const char* getChargeStateString() {  return getChargeStateString(_charge_state); };

    static volatile PMU *INSTANCE;


  protected:
    int8_t attached();


  private:
    const PowerPlantOpts _opts;
    ChargeState _charge_state = ChargeState::UNDEF;
    BQ24155*    _bq24155;
    LTC294x*    _ltc294x;
    ManuvrMsg   _periodic_pmu_read;

    static const char* getChargeStateString(ChargeState);
};

#endif //__DIGITABULUM_PMU_DRIVER_H__
