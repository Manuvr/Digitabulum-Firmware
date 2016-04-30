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




This class contains implementation details for the CPLD. So this is probably the
  best place for an overview of the CPLD as it applies to software. Revisions start at
  1. If the CPLD version is ever zero, it means that the CPLD is not initialized, and
  the class will refuse to operate for safety reasons (think: bus contention).

This class was not written with portability in mind. The particular CPLD (or FPGA) is
  unimportant, but this class will probably not be directly useful in a project other
  than Digitabulum on an STM32F7.


Overview:
========================================================================================
The CPLD is responsible for these tasks:
  1) Managing the SPI bus for each of the IMUs in the system.
  2) Aggregating interrupts from the IMUs, and making them accessible to the CPU.
  3) Storing its own revision number in a register accessible via SPI.

At the time of writing, the CPLD can be driven either by its own internal oscillator, or
  by the CPU itself using a timer channel. It is not responsible for having any concept
  of chirality. That is presently a software task.

This class is the first class to become aware of IRQs outside of the main PCB. They are...
  * IRQs related to digit function.
  * IRQs related to the loss or detection of a digit.
  * Any of the four user-assignable IRQs.

Digression into IRQs:
----------------------------------------------------------------------------------------
We have (at minimum) 20 IRQs to manage. Since we don't want to send 20 wires back to the
main PCB, we have two strategies for communicating IRQs to the CPU.


Revision 1:
----------------------------------------------------------------------------------------


Actual hardware pin assignments and descriptions for the CPLD:
--------------------------------------------------------------------
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
CPLD_GPIO_0  | PE11         | Reserved. CPLD input.
CPLD_GPIO_1  | PE14         | Reserved. CPLD input.
IRQ_WAKEUP   | PC13         | CPLD output.



CPLD register map
The CPLD registers were constructed in the same fasion as those of the IMUs. The
  most-significant bit is the R/~W bit.

Vanilla IMU Access
-----------------------------------------------------------------------
Register | Digit | Position | Description
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

Ranked IMU Access
-----------------------------------------------------------------------
Register | Digit | Position | Description
=========|=======|==========|==========================================
0x22     |   *   | Proximal | Magnetic  (6 sensors)
0x23     |   *   | Intrmedt | Magnetic  (Note: Only 5 sensors here!)
0x24     |   *   | Distal   | Magnetic  (6 sensors)
0x25     |   *   | Proximal | Inertial  (6 sensors)
0x26     |   *   | Intrmedt | Inertial  (Note: Only 5 sensors here!)
0x27     |   *   | Distal   | Inertial  (6 sensors)

Ranked access carries some complications...
The data on the wire must be sent this way:
byte 0: Rank access register
byte 1: transfer length per-sensor
byte 2: the sensor count requested

byte 3 and onward would be the normal content of the dialog with the sensor,
  starting with the target address within the sensor. Byte 3 will be captured
  in an internal register and replicated to


Internal registers
-----------------------------------------------------------------------
Register | R/~W  | Internal | Width | Description
=========|=======|==========|=======|==================================
0x28     | R     | (CS_0)   |  8    | VERSION
0x29     | R/W   | (CS_1)   |  8    | CONFIG
0x2A     | R/W   | (CS_2)   |  8    | STATUS
0x2B     | R/W   | (CS_3)   |  8    | DIGIT_INCLUSION
0x2C     | R/W   | (CS_4)   |  8    | Reserved
0x2D     | R/W   | (CS_5)   |  8    | Reserved
0x2E     | R     | (CS_6)   |  8    | Reserved
0x2F     | R     | (CS_7)   |  8    | Reserved


VERSION register is read-only, and stores an 8-bit integer reflecting the
  revision of the currently-burned CPLD.



CONFIG register
Bit | Function
----|------------
0   | IRQ_RATE_0  // These two bits set the IRQ scan prescaler.
1   | IRQ_RATE_1     0: Scan off   1: 1x      2: 2x      3: 4x
2   | Reserved
3   | Reserved
4   | Reserved
5   | Reserved
6   | OSC_SEL     // 0: Internal oscillator (default)   1: Externally-applied clock
7   | IRQ_XFER    // Set when an IRQ transfer is in-progress. Set to initiate.


DIGIT_IRQ register
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

DIGIT_INCLUSION register
Allows us to include or exclude digit data in automated
  bus operations. This is a concern independent of power-control and presence.
  Generally, you would want this register to match the value of the DIGIT_PRESENT
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

The TRANSFER_LEN register dictates how many bytes we transfer per IMU-access,
  without regard to the register selection byte.
  IE, to read 6-bytes of vector data, this register would contain the value 6.



*/




#ifndef __CPLD_DRIVER_H__
#define __CPLD_DRIVER_H__

#include "SPIBusOp.h"
#include "SPIOpCallback.h"
#include "SPIBusOp.h"
#include <Kernel.h>
#include "SPIDeviceWithRegisters.h"
//#include "Drivers/LSM9DS1/IIU.h"

#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_spi.h>
#include <stm32f7xx_hal.h>


class LSM9DS1_Common;
class IIU;

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
#define MANUS_CPLD_REG_IMU_00_X  0x80  // |
#define MANUS_CPLD_REG_IMU_00_G  0x81  // | These are pseudo registers. If the first byte in an SPI transaction is equal
#define MANUS_CPLD_REG_IMU_01_X  0x82  // |   to one of these values, the corresponding IMU will be selected, and its bus
#define MANUS_CPLD_REG_IMU_01_G  0x83  // |   connected to the CPU's SPI. Every bus operation that targets an IMU must be
#define MANUS_CPLD_REG_IMU_02_X  0x84  // |   immediately preceeded by one of these bytes.
#define MANUS_CPLD_REG_IMU_02_G  0x85  // |
#define MANUS_CPLD_REG_IMU_03_X  0x86  // | The resulting bus connection will be retained until the CPU-facing ~CS line
#define MANUS_CPLD_REG_IMU_03_G  0x87  // |   goes high.
#define MANUS_CPLD_REG_IMU_04_X  0x88  // |
#define MANUS_CPLD_REG_IMU_04_G  0x89  // |
#define MANUS_CPLD_REG_IMU_05_X  0x8A  // |
#define MANUS_CPLD_REG_IMU_05_G  0x8B  // |
#define MANUS_CPLD_REG_IMU_06_X  0x8C  // |
#define MANUS_CPLD_REG_IMU_06_G  0x8D  // |
#define MANUS_CPLD_REG_IMU_07_X  0x8E  // |
#define MANUS_CPLD_REG_IMU_07_G  0x8F  // |
#define MANUS_CPLD_REG_IMU_08_X  0x90  // |
#define MANUS_CPLD_REG_IMU_08_G  0x91  // |
#define MANUS_CPLD_REG_IMU_09_X  0x92  // |
#define MANUS_CPLD_REG_IMU_09_G  0x93  // |
#define MANUS_CPLD_REG_IMU_0A_X  0x94  // |
#define MANUS_CPLD_REG_IMU_0A_G  0x95  // |
#define MANUS_CPLD_REG_IMU_0B_X  0x96  // |
#define MANUS_CPLD_REG_IMU_0B_G  0x97  // |
#define MANUS_CPLD_REG_IMU_0C_X  0x98  // |
#define MANUS_CPLD_REG_IMU_0C_G  0x99  // |
#define MANUS_CPLD_REG_IMU_0D_X  0x9A  // |
#define MANUS_CPLD_REG_IMU_0D_G  0x9B  // |
#define MANUS_CPLD_REG_IMU_0E_X  0x9C  // |
#define MANUS_CPLD_REG_IMU_0E_G  0x9D  // |
#define MANUS_CPLD_REG_IMU_0F_X  0x9E  // |
#define MANUS_CPLD_REG_IMU_0F_G  0x9F  // |
#define MANUS_CPLD_REG_IMU_10_X  0xA0  // |
#define MANUS_CPLD_REG_IMU_10_G  0xA1  // |


#define MANUS_CPLD_REG_CONFIG    0x00  // CPLD operating parameters
#define MANUS_CPLD_REG_VERSION   0x03  // Holds CPLD revision number.


/* Configuration register bits *****************************/

/***********************************************************/


/*
* This type maps the bus addresses of a given IMU chip to its location on the hand.
*   These are constants, but this data may vary with CPLD version.
* This struct will probably become obsolete once the K-map class is smart enough to guess
*   what IMU is at which location.
*/
typedef struct {
  uint8_t  imu_addr;    // The address of the inertial half of the LSM9DS1.
  uint8_t  mag_addr;    // The address of the magnetic half of the LSM9DS1.
  uint32_t irq_mask;    // This value represents where in the CPLD IRQ register this IIUs bits lie.
  //uint8_t  location;    // This is an integer that represents our location code. This might need refinement.
} IMUBusMap;


#define CPLD_SPI_MAX_QUEUE_PRINT 3     // How many SPI queue items should we print for debug?
#define PREALLOCATED_SPI_JOBS    80    // How many SPI queue items should we have on-tap?

/*
* The actual CPLD driver class. Might could implement this as a singleton.
*/
class CPLDDriver : public EventReceiver, public SPIDeviceWithRegisters {
  public:
    CPLDDriver();
    ~CPLDDriver(void);       // Should never be called. Here for the sake of completeness.

    /* Overrides from the SPICallback interface */
    virtual int8_t spi_op_callback(SPIBusOp*);
    int8_t queue_spi_job(SPIBusOp*);


/* RESCOPE THESE??? */
    void reset(void);                  // Causes the CPLD to be reset.

    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/
    /***EVERYTHING BELOW THIS LINE MUST JUSTIFY ITS EXISTANCE OR DIAF ****/

    /* This is a temporary fix until I can migrate SPI control away from the CPLD. */
    bool step_queues(bool);

    // Members related to the work queue...
    SPIBusOp* current_queue_item = NULL;
    int8_t advance_work_queue();
    SPIBusOp* issue_spi_op_obj();

    // These are interrupt service routines...
    volatile static void irqService_vect_0(void);
    uint32_t read_imu_irq_pins(void);  // TODO: Can this be optimized down at all?


    /* Overrides from EventReceiver */
    const char* getReceiverName();
    void printDebug(StringBuilder*);
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable *);
    void procDirectDebugInstruction(StringBuilder*);


    uint8_t getCPLDVersion(void);      // Read the version code in the CPLD.

    static IMUBusMap imu_map[17];     // This is how the CPLD keeps track of IIUs and their addresses.


  protected:
    int8_t bootComplete();      // This is called from the base notify().


  private:
    ManuvrRunnable event_spi_queue_ready;
    ManuvrRunnable event_spi_callback_ready;
    ManuvrRunnable event_spi_timeout;

    uint32_t  digit_interrupts   = 0; // CPLD register. Digit interrupts.
    uint32_t  pending_interrupts = 0; // These are interrupts that are in the SPI queue but haven't returned results yet.

    uint32_t  bus_timeout_millis = 5;

    uint8_t   cpld_version       = 0; // CPLD Register. If zero, than the CPLD has not been initialized.
    uint8_t   cpld_conf_value    = 0; // CPLD register. Configuration.
    uint8_t   spi_cb_per_event   = 3; // Used to limit the number of callbacks processed per event.


    /* SPI and work queue related members */
    int  cpld_max_bus_queue_depth     = 300;     // Debug
    int  spi_prescaler                = SPI_BAUDRATEPRESCALER_16;
    bool cpld_chain_cs_to_queue       = false;   // Debug
    bool cpld_prevent_bus_queue_flood = true;    // Debug
    bool spi_queue_idle               = true;    // TODO: Convert to bus profiler. Need to know bus use percentage/volume.
    bool cpld_service_irqs            = false;

    PriorityQueue<SPIBusOp*> work_queue;
    PriorityQueue<SPIBusOp*> callback_queue;
    PriorityQueue<SPIBusOp*> preallocated;
    static SPIBusOp preallocated_bus_jobs[PREALLOCATED_SPI_JOBS];// __attribute__ ((section(".ccm")));

    uint32_t preallocation_misses = 0;        // How many times have we starved the preallocation queue?
    uint32_t specificity_burden   = 0;        // How many queue items have new deleted?

    void purge_queued_work();     // Flush the work queue.
    void purge_queued_work_by_dev(SPIOpCallback *dev);   // Flush the work queue by callback match
    void purge_stalled_job();     // TODO: Misnomer. Really purges the active job.
    int8_t service_callback_queue();
    void reclaim_queue_item(SPIBusOp*);


    /* Setup and init fxns. */
    void gpioSetup(void);
    void init_spi(uint8_t cpol, uint8_t cpha);  // Pass 0 for CPHA 0.


    /* Low-level CPLD register stuff */
    bool      ext_oscillator_enabled;   // TODO: Collapse into a bitflag.
    bool      int_oscillator_enabled;   // TODO: Collapse into a bitflag.
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

    uint32_t  queued_interrupts;      // These are interrupts we haven't serviced yet.

    uint16_t readInternalStates(void);
};

#endif
