/*
File:   IRQRouter.h
Author: J. Ian Lindsay
Date:   2017.10.05

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


*/


#ifndef __CPLD_IRQ_ROUTER_H__
#define __CPLD_IRQ_ROUTER_H__

#include <Platform/Peripherals/SPI/SPIBusOp.h>
#include <Platform/Platform.h>

#define CPLD_GUARD_BIT_VALUE  0x03

//TODO: These values are not correct, but the pattern is.
#define CPLD_IRQ_BITMASK_DRDY_MASK_0   0x11111111  //
#define CPLD_IRQ_BITMASK_INTM_MASK_0   0x22222222  //
#define CPLD_IRQ_BITMASK_INT1_MASK_0   0x44444444  //
#define CPLD_IRQ_BITMASK_INT2_MASK_0   0x88888888  //
#define CPLD_IRQ_BITMASK_DRDY_MASK_1   0x11111111  //
#define CPLD_IRQ_BITMASK_INTM_MASK_1   0x22222222  //
#define CPLD_IRQ_BITMASK_INT1_MASK_1   0x44444444  //
#define CPLD_IRQ_BITMASK_INT2_MASK_1   0x88888888  //
#define CPLD_IRQ_BITMASK_DRDY_MASK_2   0x01        //
#define CPLD_IRQ_BITMASK_INTM_MASK_2   0x02        //
#define CPLD_IRQ_BITMASK_INT1_MASK_2   0x04        //
#define CPLD_IRQ_BITMASK_INT2_MASK_2   0x08        //

#define CPLD_IRQ_BITMASK_IMU_0_MASK    0x0000000F  //
#define CPLD_IRQ_BITMASK_IMU_1_MASK    0x000000F0  //
#define CPLD_IRQ_BITMASK_IMU_2_MASK    0x00000F00  //
#define CPLD_IRQ_BITMASK_IMU_3_MASK    0x0000F000  //
#define CPLD_IRQ_BITMASK_IMU_4_MASK    0x000F0000  //
#define CPLD_IRQ_BITMASK_IMU_5_MASK    0x00F00000  //
#define CPLD_IRQ_BITMASK_IMU_6_MASK    0x0F000000  //
#define CPLD_IRQ_BITMASK_IMU_7_MASK    0xF0000000  //
#define CPLD_IRQ_BITMASK_IMU_8_MASK    0x0000000F  //
#define CPLD_IRQ_BITMASK_IMU_9_MASK    0x000000F0  //
#define CPLD_IRQ_BITMASK_IMU_10_MASK   0x00000F00  //
#define CPLD_IRQ_BITMASK_IMU_11_MASK   0x0000F000  //
#define CPLD_IRQ_BITMASK_IMU_12_MASK   0x000F0000  //
#define CPLD_IRQ_BITMASK_IMU_13_MASK   0x00F00000  //
#define CPLD_IRQ_BITMASK_IMU_14_MASK   0x0F000000  //
#define CPLD_IRQ_BITMASK_IMU_15_MASK   0xF0000000  //
#define CPLD_IRQ_BITMASK_IMU_16_MASK   0x0F        //


class IRQRouterOptions {
  public:
};


/**
* The CPLD driver class.
*/
class IRQRouter {
  public:
    IRQRouter(const CPLDPins*);
    ~IRQRouter();

    bool digitExists(DigitPort);

    /* Nice API break-out to CONFIG register bits. */
    inline void setIRQ74(bool x) {           setCPLDConfig(CPLD_CONF_BIT_IRQ_74, x);     };
    inline void setGPIO(bool x) {            setCPLDConfig(CPLD_CONF_BIT_GPIO, x);       };
    inline void conserveDigitDrive(bool x) { setCPLDConfig(CPLD_CONF_BIT_PWR_CONSRV, x); };
    inline void disableIRQScan(bool x) {     setCPLDConfig(CPLD_CONF_BIT_IRQ_SCAN, x);   };

    inline int8_t setWakeupSignal(uint8_t _val) {
      return writeRegister(CPLD_REG_WAKEUP_IRQ, _val | 0x80);
    };



  private:
    /* These values represent where in the IRQ buffer this IIU's bits lie. */
    inline uint8_t _irq_offset_byte(int idx) {  return (idx >> 1);     };
    inline uint8_t _irq_offset_bit(int idx) {   return (idx << 2);     };

    void _digit_irq_force(uint8_t digit, bool state);

    static uint32_t _irq_frames_rxd;     // How many IRQ frames have arrived.
};

#endif   // __CPLD_IRQ_ROUTER_H__
