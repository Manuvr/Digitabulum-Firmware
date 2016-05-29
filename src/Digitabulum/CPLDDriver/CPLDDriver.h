/*
File:   CPLDDriver.h
Author: J. Ian Lindsay
Date:   2014.07.01

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



Overview:
================================================================================
This class contains implementation details for the CPLD. So this is probably the
  best place for an overview of the CPLD as it applies to software.

The first actions on power-up ought to include reading the version register.
  Versions start at 1. If the CPLD version is ever zero, it means that the CPLD
  is not initialized, and the class will refuse to operate for safety reasons.

This class was not written with portability in mind. The particular CPLD (or
  FPGA) is unimportant, but this class will make assumptions about memory
  regions and hardware layers, and was not intended to be a pattern for porting.

This driver decends from Digitabulum's r0 CPLD driver, which ran on an STM32F4.

The CPLD is responsible for these tasks:
  1) Managing the SPI buses for each of the IMUs in the system.
  2) Aggregating interrupts from the IMUs and making them accessible to the CPU.
  3) Storing its own revision number in a register accessible via SPI.
  4) Detecting the presence or absense of a digit.

It is NOT responsible for having a concept of chirality. Digit identifiers used
  in this context refer to a specific connector on the main PCB. Chirality is a
  software notion that is introduced in ManuLegend, where digits are numbered
  1-5 following anatomical convention. Chirality is discussed there.

The metacarpals position is alternately known as "Digit 0", since it is
  addressed as if it were the first-of-six digits.

The CPLD is bus master on both SPI1 and SPI2.


IRQs:
--------------------------------------------------------------------------------
This class is the first class to become aware of IRQs from the sensor package.
Potential sources of interrupt...
  * Four IRQ signals for each IMU in a fully-populated glove (68 total signals)
  * IRQs related to the loss or detection of a digit (6 signals)
  * Any of the 4 user-assignable IRQs

We have up-to 68 IMU IRQs to manage (4 IRQs per-sensor, with 17 sensors). Since
  we don't want to send 68 wires back to the CPU, we serialize them and send an
  80-bit message back to the CPU. That message is handled when the SPI2_CS line
  rises. From that point a comparison can be made against the last reported IRQ
  message and signals delivered to the classes responsible for handling them.

IRQs are aggregated and checked for validity by internal CPLD logic. If a
  difference is found, the entire IRQ signal set is sent back to the CPU via
  SPI2. It is the responsibility of the software to make sense of the
  differences. But if everything is operating correctly, each received IRQ
  message will be different from its predecesor by at least one bit.

To facilitate power-management, any given IRQ signal can be mapped to the CPU's
  special WAKEUP interrupt pin.


Changing between clock sources:
--------------------------------------------------------------------------------
At the time of writing, the CPLD can be driven either by its own internal
  oscillator, or by the CPU using a timer channel. When driven externally, the
  clock is adjustable within the range of DC to 20MHz. The clock is divided once
  internally to arrive at the 10MHz maximum imposed by the sensors.

The internal oscilator is the default source to ensure proper startup and reset
  behavior. When switching to the external clock source, the CPU should first
  enable the timer output, and only after it is running, send the command to the
  CPLD to switch to it. The inverse order should be followed when moving from
  EXT to INT clocks.

Most cases would want to drive the clock signal in direct proportion to their
  data demands, thus only using the amount of power required. But the use of the
  internal oscillator allows the CPU (and its timer peripheral) to be shutdown
  while still retaining sensitivity to interrupts. Combined with WAKEUP routing.
  this allows the program to put the glove into very low power states with
  runtime-selectable wakeup conditions.

The clock source can be changed at any time. Bus operations can continue
  uninterrupted throughout the transition between clock domains.


These are the hardware pin assignments and descriptions for the CPLD:
--------------------------------------------------------------------------------
  CPLD Pin     | CPU Port/Pin | Description
  =============|==============|===========================================
  SPI1_CS      | PA4          | SPI1 is the main bus. Data and requests
  SPI1_CLK     | PA5          |   traverse this path. CPLD is master.
  SPI1_MISO    | PA6          |
  SPI1_MOSI    | PA7          |
  SPI2_CS      | PB12         | SPI2 is for the exclusive purpose
  SPI2_CLK     | PB13         |   moving interrupt data. CPLD is
  SPI2_MISO    | PB14         |   the master.
  SPI2_MOSI    | PB15         |
  CPLD_EXT_CLK | PA8          | External clock for the CPLD.
  CPLD_RESET   | PB9          | Resets the CPLD. Active low.
  CPLD_GPIO_0  | PE11         | Reserved. CPLD output.
  CPLD_GPIO_1  | PE14         | Reserved. CPLD output.
  IRQ_WAKEUP   | PC13         | CPLD output.



CPLD register access procedure:
================================================================================
Despite being the bus slave, the CPU is responsible for initiating the transfer
  by pulling the SPI1_CS line low. The CPU must hold the pin low for at least
  one complete CPLD clock cycle. After that, the CPU should release the pin and
  wait for the SPI1_CS line to rise. This indicates transfer completion.

The CPLD registers were constructed in the same fasion as those of the IMUs. The
  most-significant bit is the R/~W bit.


CPLD internal register access procedure:
--------------------------------------------------------------------------------
  Register | R/~W  | Internal | Width | Description
  =========|=======|==========|=======|==================================
  0x28     | R     | (CS_0)   |  8    | VERSION
  0x29     | R/W   | (CS_1)   |  8    | CONFIG
  0x2A     | R     | (CS_2)   |  8    | STATUS
  0x2B     | R     | (CS_3)   |  8    | DIGIT_PRESENT
  0x2C     | R     | (CS_4)   |  8    | Reserved
  0x2D     | R     | (CS_5)   |  8    | Reserved
  0x2E     | R     | (CS_6)   |  8    | Reserved
  0x2F     | R     | (CS_7)   |  8    | Reserved

  /--------------< 1 byte >----\
  | Register  |     DATA       |
  \----------------------------/

VERSION register is read-only, and stores an 8-bit integer reflecting the
  version of the currently-burned CPLD.

CONFIG register
  Bit | Function
  ----|------------
  0   | IRQ_RATE_0  // These two bits set the IRQ scan prescaler.
  1   | IRQ_RATE_1     0: Scan off   1: 1x      2: 2x      3: 4x
  2   | Reserved
  3   | Reserved
  4   | Reserved
  5   | Reserved
  6   | OSC_SEL     // 0: Internal oscillator (default)   1: External clock
  7   | IRQ_XFER    // Set when an IRQ transfer is in-progress. Set to initiate.


DIGIT_PRESENT register (read-only)
  Bit | Function
  ----|------------
  0   | Metacarpals
  1   | Digit 1
  2   | Digit 2
  3   | Digit 3
  4   | Digit 4
  5   | Digit 5
  6   | 0
  7   | 0

The address of the desired internal register should prefix the desired value (in
  the case of a write), or (if reading) will be followed by the present value
  stored in that register.
All CPLD registers should be accessed byte-wise. All traffic to an internal
  register therefore occupies two bytes on the SPI1 bus.



IMU register access procedure:
--------------------------------------------------------------------------------
  DEV_ADDR | Digit | Position | Aspect
  =========|=======|==========|==========================================
  0x00     |   0   | Proximal | Inertial
  0x01     |   0   | Distal   | Inertial
  0x02     |   1   | Proximal | Inertial
  0x03     |   1   | Intrmedt | Inertial
  0x04     |   1   | Distal   | Inertial
  0x05     |   2   | Proximal | Inertial
  0x06     |   2   | Intrmedt | Inertial
  0x07     |   2   | Distal   | Inertial
  0x08     |   3   | Proximal | Inertial
  0x09     |   3   | Intrmedt | Inertial
  0x0A     |   3   | Distal   | Inertial
  0x0B     |   4   | Proximal | Inertial
  0x0C     |   4   | Intrmedt | Inertial
  0x0D     |   4   | Distal   | Inertial
  0x0E     |   5   | Proximal | Inertial
  0x0F     |   5   | Intrmedt | Inertial
  0x10     |   5   | Distal   | Inertial
  0x11     |   0   | Proximal | Magnetic
  0x12     |   0   | Distal   | Magnetic
  0x13     |   1   | Proximal | Magnetic
  0x14     |   1   | Intrmedt | Magnetic
  0x15     |   1   | Distal   | Magnetic
  0x16     |   2   | Proximal | Magnetic
  0x17     |   2   | Intrmedt | Magnetic
  0x18     |   2   | Distal   | Magnetic
  0x19     |   3   | Proximal | Magnetic
  0x1A     |   3   | Intrmedt | Magnetic
  0x1B     |   3   | Distal   | Magnetic
  0x1C     |   4   | Proximal | Magnetic
  0x1D     |   4   | Intrmedt | Magnetic
  0x1E     |   4   | Distal   | Magnetic
  0x1F     |   5   | Proximal | Magnetic
  0x20     |   5   | Intrmedt | Magnetic
  0x21     |   5   | Distal   | Magnetic

  /-------------------------------------------------------------------------\
  | DEV_ADDR  | XFER_LEN  | DEV_COUNT | REG_ADDR  |          DATA           |
  \-------------------------------------------------------------------------/
        1           1           1           1       <XFER_LEN * DEV_COUNT>

  DEV_ADDR:  The address of the desired aspect of the desired IMU. One byte.
  XFER_LEN:  The size of the I/O operation WRT to each sensor. One byte.
  DEV_COUNT: The number of devices to traverse in this operation. One byte.
  REG_ADDR:  The address of the desired register within the IMU. One byte.
  DATA:      dialog with IMU. <XFER_LEN * DEV_COUNT> bytes.

The XFER_LEN byte dictates how many bytes we transfer per address not including
  the register selection byte.

IE, to read 6-bytes of vector data from the magnetometer at the distal position
  of digit-1, the transfer would be...
  /-------------------------------------------------------< 6 bytes >-------\
  |   0xA5    |   0x06    |   0x01    |   0xA8    |     0x00 .... 0x00      |
  \-------------------------------------------------------------------------/

DEV_COUNT is the number of times we wish this access to be repeated over
  successive devices.

REG_ADDR is the first byte of IMU-directed bus traffic. In the case of accesses
  that span many devices, it is captured as the fourth byte and repeated
  transparently at device boundaries. Modifying the example above to read the
  magnetometer vectors from each sensor on digit-1 (3 total)...
  /------------------------------------------------------< 18 bytes >-------\
  |   0xA5    |   0x06    |   0x03    |   0xA8    |     0x00 .... 0x00      |
  \-------------------------------------------------------------------------/

The real gains come from the extension of read operations across ALL sensors.
  To read an entire frame of inertial data (2 Vector3<int16> from 17 sensors)...
  /------------------------------------------------------< 204 bytes >------\
  |   0x80    |   0x0C    |   0x11    |   0xD8    |     0x00 .... 0x00      |
  \-------------------------------------------------------------------------/

Combined with the full-frame magnetometer read, we get a full frame of sensor
  data in two bus operations, thereby minimizing the bureaucracy associated with
  transfer-management.
  /------------------------------------------------------< 102 bytes >------\
  |   0x91    |   0x06    |   0x11    |   0xA8    |     0x00 .... 0x00      |
  \-------------------------------------------------------------------------/


Ranked IMU Access
--------------------------------------------------------------------------------
In CPLD r1, ranked-access is only useful for write operations. It allows for
  synchronous configuration of IMUs sharing a rank, and saves software the
  burden of tracking delta-T independently for each sensor. Apart from being
  write-only, the transfer rules are the same as for individual IMU access.

  DEV_ADDR | Digit | Position | Aspect
  =========|=======|==========|==========================================
  0x22     |  ALL  | Proximal | Magnetic  (6 sensors)
  0x23     |  ALL  | Intrmedt | Magnetic  (Note: Only 5 sensors here!)
  0x24     |  ALL  | Distal   | Magnetic  (6 sensors)
  0x25     |  ALL  | Proximal | Inertial  (6 sensors)
  0x26     |  ALL  | Intrmedt | Inertial  (Note: Only 5 sensors here!)
  0x27     |  ALL  | Distal   | Inertial  (6 sensors)

The metacarpals and wrist complex (digit-0) does not have an intermediate
  position. This is why there are only 5 sensors in that rank. Fortunately,
  this is largely irrelevant, as each rank is treated as a single device.

Assuming the configuration of the IMUs is identical, configuring the entire
  sensor package can be done with two bus operations; one for each aspect.

As is the case with individual IMU access, REG_ADDR will be captured in an
  internal register and replicated to each IMU as the transfer crosses device
  boundaries.



CPLD Version notes:
================================================================================

Revision 1:
--------------------------------------------------------------------------------
Initial version. Internal registers do not follow spec to aid debug.
IRQ agg and addressing system is complete. At least: it passes simulation.

*/

#ifndef __CPLD_DRIVER_H__
#define __CPLD_DRIVER_H__

#include "SPIBusOp.h"
#include "SPIOpCallback.h"
#include "SPIBusOp.h"
#include <Kernel.h>
#include "SPIDeviceWithRegisters.h"

#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_spi.h>
#include <stm32f7xx_hal.h>


class LSM9DS1_Common;
class IIU;

#define CPLD_SPI_MAX_QUEUE_PRINT 3     // How many SPI queue items should we print for debug?
#define PREALLOCATED_SPI_JOBS    10    // How many SPI queue items should we have on-tap?

/*
* These state flags are hosted by the EventReceiver. This may change in the future.
* Might be too much convention surrounding their assignment across inherritence.
*/
#define CPLD_FLAG_INT_OSC      0x01    // Running on internal oscillator.
#define CPLD_FLAG_EXT_OSC      0x02    // Running on external oscillator.
#define CPLD_FLAG_SVC_IRQS     0x04    // Should the CPLD respond to IRQ signals?
#define CPLD_FLAG_QUEUE_IDLE   0x08    // Is the SPI queue idle?
#define CPLD_FLAG_QUEUE_GUARD  0x10    // Prevent bus queue floods?


/* Codes that are specific to Digitabulum's CPLD */
  #define DIGITABULUM_MSG_IMU_LEGEND           0x0600 // No args? Asking for this legend. One arg: Legend provided.
  #define DIGITABULUM_MSG_IMU_IRQ_RAISED       0x0602 // IRQ asserted by CPLD.

  #define DIGITABULUM_MSG_IMU_INIT             0x0604 //
  #define DIGITABULUM_MSG_IMU_READ             0x0605 // Signal to read a given set of IMUs.
  #define DIGITABULUM_MSG_IMU_MAP_STATE        0x0606
  #define DIGITABULUM_MSG_CPLD_RESET_COMPLETE  0x0607 // The CPLD reset is ready for disassertion.
  #define DIGITABULUM_MSG_CPLD_RESET_CALLBACK  0x0608 // The CPLD reset is ready for disassertion.
  #define DIGITABULUM_MSG_IMU_QUAT_CRUNCH      0x0609 // The given IMU has samples to grind into a quat.
  #define DIGITABULUM_MSG_IMU_TAP              0x060A // The given IMU experienced a tap.
  #define DIGITABULUM_MSG_IMU_DOUBLE_TAP       0x060B // The given IMU experienced a double tap.

/* Vibrator codes */
  #define DIGITABULUM_MSG_GPIO_VIBRATE_0       0xA000 // Some class wants to trigger vibrator 0.
  #define DIGITABULUM_MSG_GPIO_VIBRATE_1       0xA001 // Some class wants to trigger vibrator 1.


/* CPLD register map ***************************************/
#define CPLD_REG_IMU_DM_P_I    0x00  // |
#define CPLD_REG_IMU_DM_D_I    0x01  // | These are pseudo registers. If the first byte in an SPI transaction is equal
#define CPLD_REG_IMU_D1_P_I    0x02  // |   to one of these values, the corresponding IMU will be selected, and its bus
#define CPLD_REG_IMU_D1_I_I    0x03  // |   connected to the CPU's SPI. Every bus operation that targets an individual
#define CPLD_REG_IMU_D1_D_I    0x04  // |   IMU must be immediately preceeded by one of these bytes.
#define CPLD_REG_IMU_D2_P_I    0x05  // |
#define CPLD_REG_IMU_D2_I_I    0x06  // | The resulting bus connection will be retained until the CPU-facing ~CS line
#define CPLD_REG_IMU_D2_D_I    0x07  // |   goes high.
#define CPLD_REG_IMU_D3_P_I    0x08  // |
#define CPLD_REG_IMU_D3_I_I    0x09  // |
#define CPLD_REG_IMU_D3_D_I    0x0A  // |
#define CPLD_REG_IMU_D4_P_I    0x0B  // |
#define CPLD_REG_IMU_D4_I_I    0x0C  // |
#define CPLD_REG_IMU_D4_D_I    0x0D  // |
#define CPLD_REG_IMU_D5_P_I    0x0E  // |
#define CPLD_REG_IMU_D5_I_I    0x0F  // |
#define CPLD_REG_IMU_D5_D_I    0x10  // |
#define CPLD_REG_IMU_DM_P_M    0x11  // |
#define CPLD_REG_IMU_DM_D_M    0x12  // |
#define CPLD_REG_IMU_D1_P_M    0x13  // |
#define CPLD_REG_IMU_D1_I_M    0x14  // |
#define CPLD_REG_IMU_D1_D_M    0x15  // |
#define CPLD_REG_IMU_D2_P_M    0x16  // |
#define CPLD_REG_IMU_D2_I_M    0x17  // |
#define CPLD_REG_IMU_D2_D_M    0x18  // |
#define CPLD_REG_IMU_D3_P_M    0x19  // |
#define CPLD_REG_IMU_D3_I_M    0x1A  // |
#define CPLD_REG_IMU_D3_D_M    0x1B  // |
#define CPLD_REG_IMU_D4_P_M    0x1C  // |
#define CPLD_REG_IMU_D4_I_M    0x1D  // |
#define CPLD_REG_IMU_D4_D_M    0x1E  // |
#define CPLD_REG_IMU_D5_P_M    0x1F  // |
#define CPLD_REG_IMU_D5_I_M    0x20  // |
#define CPLD_REG_IMU_D5_D_M    0x21  // |

#define CPLD_REG_RANK_P_I      0x22  // | These registers are for ranked access.
#define CPLD_REG_RANK_I_I      0x23  // | This is only useful for write operations.
#define CPLD_REG_RANK_D_I      0x24  // | Buffers for these transfers must be distinct
#define CPLD_REG_RANK_P_M      0x25  // |   from those for individual access.
#define CPLD_REG_RANK_I_M      0x26  // |
#define CPLD_REG_RANK_D_M      0x27  // |

#define CPLD_REG_VERSION       0x28  // | Holds CPLD revision number.
#define CPLD_REG_CONFIG        0x29  // | CPLD operating parameters
#define CPLD_REG_STATUS        0x2A  // | Status
#define CPLD_REG_WAKEUP_IRQ    0x2B  // | WAKEUP mapping
#define CPLD_REG_CS_4          0x2C  // | RESERVED
#define CPLD_REG_CS_5          0x2D  // | RESERVED
#define CPLD_REG_CS_6          0x2E  // | RESERVED
#define CPLD_REG_CS_7          0x2F  // | RESERVED

/* Bitmask defs for the CONFIG register. */
#define CPLD_CONF_BIT_EXT_CLK  0x01
#define CPLD_CONF_BIT_GPIO_1   0x20
#define CPLD_CONF_BIT_GPIO_0   0x40
#define CPLD_CONF_BIT_DEN_AG_0 0x80


/*
* The CPLD driver class.
*/
class CPLDDriver : public EventReceiver, public SPIOpCallback {
  public:
    SPIBusOp* current_queue_item = NULL;

    CPLDDriver();
    ~CPLDDriver();       // Should never be called. Here for the sake of completeness.

    /* Overrides from the SPICallback interface */
    virtual int8_t spi_op_callback(SPIBusOp*);
    int8_t queue_spi_job(SPIBusOp*);

    /* Overrides from EventReceiver */
    const char* getReceiverName();
    void printDebug(StringBuilder*);
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable *);
    #if defined(__MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //__MANUVR_CONSOLE_SUPPORT

    /* Members related to the work queue... */
    int8_t advance_work_queue();
    bool step_queues(bool);
    SPIBusOp* issue_spi_op_obj();

    void reset(void);                 // Causes the CPLD to be reset.
    uint8_t getCPLDVersion();         // Read the version code in the CPLD.

    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    uint32_t read_imu_irq_pins();     // TODO: Can this be optimized down at all?

    // These are interrupt service routines...
    volatile static void irqService_vect_0(void);


  protected:
    int8_t bootComplete();      // This is called from the base notify().


  private:
    uint8_t   _irq_data_0[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // IRQ data is double-buffered
    uint8_t   _irq_data_1[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  //   in these arrays.
    uint8_t*  _irq_data       = _irq_data_0;                     // Used for paging the above buffers.

    ManuvrRunnable _irq_data_arrival;
    ManuvrRunnable event_spi_queue_ready;
    ManuvrRunnable event_spi_callback_ready;
    ManuvrRunnable event_spi_timeout;

    uint32_t  bus_timeout_millis = 5;

    uint8_t   cpld_version       = 0; // CPLD Register. If zero, than the CPLD has not been initialized.
    uint8_t   cpld_conf_value    = 0; // CPLD register. Configuration.
    uint8_t   cpld_status_value  = 0; // CPLD register. Status.
    uint8_t   cpld_wakeup_source = 0; // CPLD register. WAKEUP mapping.
    uint8_t   spi_cb_per_event   = 3; // Used to limit the number of callbacks processed per event.

    /* SPI and work queue related members */
    int  cpld_max_bus_queue_depth     = 300;     // Debug

    PriorityQueue<SPIBusOp*> work_queue;
    PriorityQueue<SPIBusOp*> callback_queue;
    PriorityQueue<SPIBusOp*> preallocated;

    uint32_t preallocation_misses = 0;        // How many times have we starved the preallocation queue?
    uint32_t specificity_burden   = 0;        // How many queue items have new deleted?

    /* Inlines for deriving address and IRQ bit offsets from index. */
    // Address of the inertial half of the LSM9DS1.
    inline uint8_t _intertial_addr(int idx) {   return ((idx % 17) + 0x00);   };
    // Address of the magnetic half of the LSM9DS1.
    inline uint8_t _magnetic_addr(int idx) {    return ((idx % 17) + 0x11);   };
    // These values represent where in the IRQ buffer this IIU's bits lie.
    inline uint8_t _irq_offset_byte(int idx) {  return (idx >> 1);            };
    inline uint8_t _irq_offset_bit(int idx) {   return (idx << 2);            };

    int8_t readRegister(uint8_t reg_addr);
    int8_t writeRegister(uint8_t reg_addr, uint8_t val);

    void purge_queued_work();     // Flush the work queue.
    void purge_queued_work_by_dev(SPIOpCallback *dev);   // Flush the work queue by callback match
    void purge_stalled_job();     // TODO: Misnomer. Really purges the active job.
    int8_t service_callback_queue();
    void reclaim_queue_item(SPIBusOp*);


    /* Setup and init fxns. */
    void gpioSetup(void);
    void init_spi(uint8_t cpol, uint8_t cpha);  // Pass 0 for CPHA 0.


    /* Low-level CPLD register stuff */
    void externalOscillator(bool on);    // Enable or disable the CPLD external oscillator.
    void internalOscillator(bool on);    // Enable or disable the CPLD internal oscillator.
    void setCPLDConfig(uint8_t mask, bool enable);

    int8_t iiu_group_irq();

    IIU* fetch_iiu_by_bus_addr(uint8_t);
    int8_t fetch_iiu_index_by_bus_addr(uint8_t);

    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/

    uint16_t readInternalStates(void);

    static SPIBusOp preallocated_bus_jobs[PREALLOCATED_SPI_JOBS];// __attribute__ ((section(".ccm")));
};

#endif
