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
  addressed as if it were the first-of-six digits. Digit 0 lacks an intermediate
  sensor position. This must be considered when doing ranked-access.

The CPLD is bus master on both SPI1 and SPI2.


IRQs:
--------------------------------------------------------------------------------
This class is the first class to become aware of IRQs from the sensor package.
Potential sources of interrupt...
  * Four IRQ signals for each IMU in a fully-populated glove (68 total signals)
  * IRQs related to the loss or detection of a digit (6 signals)
  * IRQ 74 is assigned to a CONFIG register bit for manual re-send.
  * IRQ 75 is assigned to a pin on the expansion port. It is active-low.

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

The external oscilator is the default source to ensure proper startup and reset
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
  by pulling the SPI2_MISO line low. The CPU must hold the pin low for at least
  one complete CPLD clock cycle.

The CPLD registers were constructed in the same fasion as those of the IMUs. The
  most-significant bit is the R/~W bit.


CPLD internal register access procedure:
--------------------------------------------------------------------------------
  Register | R/~W  | Internal | Width | Description
  =========|=======|==========|=======|==================================
  0xA8     | R     | (CS_0)   |  8    | VERSION
  0x28     | W     | (CS_0)   |  8    | CONFIG
  0x29     | R     | (CS_1)   |  8    | WAKEUP_SOURCE
  0x2A     | R     | (CS_2)   |  5    | FORSAKEN_DIGITS

The address of the desired internal register should prefix the desired value (in
  the case of a write), or (if reading) will be followed by the present value
  stored in that register.
All CPLD registers should be accessed byte-wise. All traffic to an internal
  register therefore occupies two bytes on the SPI1 bus.

  /--------------< 1 byte >----\
  | Register  |     DATA       |
  \----------------------------/

VERSION register is read-only, and stores an 8-bit integer reflecting the
  version of the currently-burned CPLD.


CONFIG register is write-only, and all bits default to zero.
Bit | Function
----|-----------------
0   | OSC_SEL             // 0: External (default)       1: Internal
1   | IRQ_SCAN_DISABLE    // 0: Scan on (default)        1: Scan disabled
2   | IRQ_74              // IRQ signal level
3   | DIGIT_POWER_SAVE    // 0: Drive all signals        1: Selective drive
4   | IRQ_CONSTANT_SEND   // 0: IRQ send on diff         1: Constant send
5   | GPIO_0_SOURCE       // 0: SPI1_CS                  1: DEVS_REMAIN
6   | GPIO_1_STATE        // State of the GPIO_1 pin
7   | DEN_AG_MC           // State of the DEN_AG pin on the metacarpals IMU.


FORSAKEN_DIGITS register is write-only, and all bits default to 0. Setting a bit
  to 1 will cause the IRQ aggregator to cease collecting IRQ data from that
  digit. If there were any bits set for that digit, they will revert to 0.
  Combined with the DIGIT_POWER_SAVE bit in the CONFIG register, the SPI signals
  will also be disabled.
Bit | Function
----|------------
0   | Digit 1
1   | Digit 2
2   | Digit 3
3   | Digit 4
4   | Digit 5


WAKEUP_SOURCE register is write-only, and all bits default to zero. By writing a
  number to this register, that IRQ signal will directly drive the CPUs WAKEUP
  interrupt. The register will only be observed if the seventh bit is 1.
Bit | Function
----|--------------------
0   | Signal address 0
1   | Signal address 1
2   | Signal address 2
3   | Signal address 3
4   | Signal address 4
5   | Signal address 5
6   | Signal address 6
7   | Wakeup IRQ enabled.




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
#include <Platform/Platform.h>

#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_spi.h>
#include <stm32f7xx_hal.h>


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
#define CPLD_FLAG_SPI1_READY   0x20    // Is SPI1 initialized?
#define CPLD_FLAG_SPI2_READY   0x40    // Is SPI2 initialized?
#define CPLD_FLAG_DEN_AG_STATE 0x80    // DEN_AG state.


/* Event codes that are specific to Digitabulum's IMU apparatus. */
  #define DIGITABULUM_MSG_IMU_IRQ_RAISED       0x0602 // IRQ asserted by CPLD.
  #define DIGITABULUM_MSG_IMU_LEGEND           0x0600 // No args? Asking for this legend. One arg: Legend provided.
  #define DIGITABULUM_MSG_IMU_INIT             0x0604 //
  #define DIGITABULUM_MSG_IMU_READ             0x0605 // Signal to read a given set of IMUs.
  #define DIGITABULUM_MSG_IMU_MAP_STATE        0x0606
  #define DIGITABULUM_MSG_IMU_QUAT_CRUNCH      0x0609 // The given IMU has samples to grind into a quat.
  #define DIGITABULUM_MSG_IMU_TAP              0x060A // The given IMU experienced a tap.
  #define DIGITABULUM_MSG_IMU_DOUBLE_TAP       0x060B // The given IMU experienced a double tap.

/* Event codes for Digitabulum's CPLD. */
#define DIGITABULUM_MSG_SPI_QUEUE_READY      0x0230 // There is a new job in the SPI bus queue.
#define DIGITABULUM_MSG_SPI_CB_QUEUE_READY   0x0231 // There is something ready in the callback queue.
#define DIGITABULUM_MSG_CPLD_RESET_COMPLETE  0x0607 // The CPLD reset is ready for disassertion.
#define DIGITABULUM_MSG_CPLD_RESET_CALLBACK  0x0608 // The CPLD reset is ready for disassertion.
#define DIGITABULUM_MSG_CPLD_DIGIT_DROP      0x0609 // A digit was lost.


/* CPLD register map ***************************************/
#define CPLD_REG_IMU_DM_P_I    0x00  // |
#define CPLD_REG_IMU_DM_D_I    0x01  // | These are pseudo registers.
#define CPLD_REG_IMU_D1_P_I    0x02  // | If the first byte in an SPI
#define CPLD_REG_IMU_D1_I_I    0x03  // |   transaction is equal to one of these
#define CPLD_REG_IMU_D1_D_I    0x04  // |   values, the corresponding IMU will
#define CPLD_REG_IMU_D2_P_I    0x05  // |   be selected, and its bus connected
#define CPLD_REG_IMU_D2_I_I    0x06  // |   to the CPU's SPI. Each bus operation
#define CPLD_REG_IMU_D2_D_I    0x07  // |   that targets an individual IMU must
#define CPLD_REG_IMU_D3_P_I    0x08  // |   be immediately preceeded by one of
#define CPLD_REG_IMU_D3_I_I    0x09  // |   these bytes.
#define CPLD_REG_IMU_D3_D_I    0x0A  // |
#define CPLD_REG_IMU_D4_P_I    0x0B  // | The resulting bus connection will be
#define CPLD_REG_IMU_D4_I_I    0x0C  // |   retained until the CPU-facing ~CS
#define CPLD_REG_IMU_D4_D_I    0x0D  // |   line goes high.
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

#define CPLD_REG_CONFIG        0x28  // | CPLD operating parameters
#define CPLD_REG_WAKEUP_IRQ    0x29  // | WAKEUP mapping
#define CPLD_REG_DIGIT_FORSAKE 0x2A  // | Forsaken digit register.
#define CPLD_REG_VERSION       0xA8  // | Holds CPLD revision number.

/* Bitmask defs for the CONFIG register. */
#define CPLD_CONF_BIT_INT_CLK    0x01  // Internal clock enable
#define CPLD_CONF_BIT_IRQ_SCAN   0x02  // Disable IRQ scanning
#define CPLD_CONF_BIT_IRQ_74     0x04  // Set IRQ bit-74
#define CPLD_CONF_BIT_PWR_CONSRV 0x08  // Prevent bus driving on absent digits.
#define CPLD_CONF_BIT_IRQ_STREAM 0x10  // Constantly stream IRQ data
#define CPLD_CONF_BIT_GPIO_0     0x20  // Set GPIO_0 source
#define CPLD_CONF_BIT_GPIO_1     0x40  // Set GPIO_1 state
#define CPLD_CONF_BIT_DEN_AG_0   0x80  // Set The MC IMU DEN_AG pin


/*
* The CPLD driver class.
*/
class CPLDDriver : public EventReceiver, public BusAdapter<SPIBusOp> {
  public:
    CPLDDriver();
    ~CPLDDriver();       // Should never be called. Here for the sake of completeness.

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);
    int8_t advance_work_queue();
    SPIBusOp* new_op();
    SPIBusOp* new_op(BusOpcode, BusOpCallback*);

    /* Overrides from EventReceiver */
    void printDebug(StringBuilder*);
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    int8_t attached();      // This is called from the base notify().
    #if defined(MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //MANUVR_CONSOLE_SUPPORT

    /* High-level hardware control and discovery. */
    void     reset();                  // Causes the CPLD to be reset.
    uint8_t  getCPLDVersion();         // Read the version code in the CPLD.
    inline bool digitExists(uint8_t x) {   return false;   };   // TODO: When digits arrive.
    inline int8_t digitSleep(uint8_t x) {  return 0;       };   // TODO: When digits arrive.

    /* The wrist-moun */
    inline void enableCarpalAG(bool x) {     setPin(33, x);                             };
    inline void enableMetacarpalAG(bool x) { setCPLDConfig(CPLD_CONF_BIT_DEN_AG_0, x);  };

    /* Power vs performance */
    int      setCPLDClkFreq(int);      // Set the CPLD external clock frequency.
    inline int8_t setWakeupSignal(uint8_t _val) {
      return writeRegister(CPLD_REG_WAKEUP_IRQ, _val | 0x80);
    };

    /* Members related to the work queue... */
    inline void step_queues(){  Kernel::isrRaiseEvent(&event_spi_queue_ready); }

    static SPIBusOp* current_queue_item;



  private:
    ManuvrMsg event_spi_queue_ready;
    ManuvrMsg event_spi_callback_ready;
    ManuvrMsg event_spi_timeout;
    ManuvrMsg _periodic_debug;

    /* List of pending callbacks for bus transactions. */
    PriorityQueue<SPIBusOp*> callback_queue;
    uint32_t  bus_timeout_millis = 5;  // How long to spend in IO_WAIT?
    uint32_t  specificity_burden = 0;  // How many queue items have been deleted?
    uint8_t   spi_cb_per_event   = 3;  // Limit the number of callbacks processed per event.
    uint16_t  _digit_flags       = 0;  // Digit sleep state tracking flags.

    /* These values represent where in the IRQ buffer this IIU's bits lie. */
    inline uint8_t _irq_offset_byte(int idx) {  return (idx >> 1);     };
    inline uint8_t _irq_offset_bit(int idx) {   return (idx << 2);     };

    int8_t readRegister(uint8_t reg_addr);
    int8_t writeRegister(uint8_t reg_addr, uint8_t val);

    void purge_queued_work();     // Flush the work queue.
    void purge_queued_work_by_dev(BusOpCallback *dev);   // Flush the work queue by callback match
    void purge_stalled_job();     // TODO: Misnomer. Really purges the active job.
    int8_t service_callback_queue();
    void reclaim_queue_item(SPIBusOp*);

    /* Setup and init fxns. */
    void gpioSetup();
    bool _set_timer_base(uint16_t);
    void init_ext_clk();
    void init_spi(uint8_t cpol, uint8_t cpha);  // Pass 0 for CPHA 0.
    void init_spi2(uint8_t cpol, uint8_t cpha);  // Pass 0 for CPHA 0.

    /* Low-level CPLD register stuff */
    void externalOscillator(bool on);    // Enable or disable the CPLD external oscillator.
    void internalOscillator(bool on);    // Enable or disable the CPLD internal oscillator.
    void setCPLDConfig(uint8_t mask, bool enable);

    void _process_conf_update(uint8_t nu);

    int8_t iiu_group_irq();


    static SPIBusOp preallocated_bus_jobs[PREALLOCATED_SPI_JOBS];// __attribute__ ((section(".ccm")));

    /* Register representations. */
    static uint8_t cpld_version;        // CPLD version byte.
    static uint8_t cpld_conf_value;     // Configuration.
    static uint8_t forsaken_digits;     // Forsaken digits.
    static uint8_t cpld_wakeup_source;  // WAKEUP mapping.
};

#endif
