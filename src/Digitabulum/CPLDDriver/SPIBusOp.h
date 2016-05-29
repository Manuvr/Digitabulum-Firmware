/*
File:   SPIBusOp.h
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


This is the class that is used to keep bus operations on the SPI atomic.
*/


#ifndef __SPI_BUS_OP_H__
#define __SPI_BUS_OP_H__

  #include <Drivers/BusQueue/BusQueue.h>
  #include <DataStructures/StringBuilder.h>
  #include <Drivers/DeviceWithRegisters/DeviceRegister.h>
  #include <stm32f7xx_hal_dma.h>

  #define SPI_XFER_FLAG_NO_FLAGS        0x00   // By default, there are no flags set.
  #define SPI_XFER_FLAG_NO_FREE         0x01   // If set in a transaction's flags field, it will not be free()'d.
  #define SPI_XFER_FLAG_PREALLOCATE_Q   0x02   // If set, indicates this object should be returned to the prealloc queue.
  #define SPI_XFER_FLAG_DEVICE_REG_INC  0x40   // If set, indicates this operation advances addresses in the target device.
  #define SPI_XFER_FLAG_PROFILE         0x80   // If set, this bus operation shall be profiled.


  #define SPI_CALLBACK_ERROR    -1
  #define SPI_CALLBACK_NOMINAL   0
  #define SPI_CALLBACK_RECYCLE   1


class SPIOpCallback;

/*
* This class represents a single transaction on the bus.
*/
class SPIBusOp : public BusOp {
  public:
    //TODO: This is the new mechanism: uint8_t  xfer_params[4];                      // The address transfer lengths, preamble, etc...
    SPIOpCallback* callback = NULL;               // Which class gets pinged when we've finished?
    int16_t  reg_idx     = -1;                    // Optional register index. Makes callbacks faster.
    uint8_t  bus_addr    = 0x0000;                // The address that this operation is directed toward.

    //uint32_t time_began    = 0;   // This is the time when bus access begins.
    //uint32_t time_ended    = 0;   // This is the time when bus access stops (or is aborted).

    SPIBusOp();
    SPIBusOp(BusOpcode nu_op, uint16_t addr, uint8_t *buf, uint8_t len, SPIOpCallback* requester);
    ~SPIBusOp();

    /* Job control functions. */
    int8_t begin();
    int8_t markComplete();

    /**
    * This will mark the bus operation complete with a given error code.
    * Overriden for simplicity. Marks the operation with failure code NO_REASON.
    *
    * @return 0 on success. Non-zero on failure.
    */
    inline int8_t abort() {    return abort(XferFault::NO_REASON); }
    int8_t abort(XferFault);

    int8_t advance_operation(uint32_t status_reg, uint8_t data_reg);

    void wipe();

    /* Flag management fxns... */
    bool shouldReap(bool);    // Override to set the reap behavior.
    bool returnToPrealloc(bool);
    bool devRegisterAdvance(bool);

    /**
    * The bus manager calls this fxn to decide if it ought to return this object to the preallocation
    *   queue following completion.
    *
    * @return true if the bus manager class should return this object to its preallocation queue.
    */
    inline bool returnToPrealloc() {  return (flags & SPI_XFER_FLAG_PREALLOCATE_Q);  }

    /**
    * The bus manager calls this fxn to decide if it ought to return this object to the preallocation
    *   queue following completion.
    *
    * @return true if this bus operation is being profiled.
    */
    inline bool profile() {         return (flags & SPI_XFER_FLAG_PROFILE);  }
    inline void profile(bool en) {
      flags = (en) ? (flags | SPI_XFER_FLAG_PROFILE) : (flags & ~(SPI_XFER_FLAG_PROFILE));
    };

    /**
    * The bus manager calls this fxn to decide if it ought to return this object to the preallocation
    *   queue following completion.
    *
    * @return true if the bus manager class should return this object to its preallocation queue.
    */
    inline bool devRegisterAdvance() {  return (flags & SPI_XFER_FLAG_DEVICE_REG_INC);  }


    /**
    * The bus manager calls this fxn to decide if it ought to free this object after completion.
    *
    * @return true if the bus manager class should free() this object. False otherwise.
    */
    inline bool shouldReap() {        return ((flags & SPI_XFER_FLAG_NO_FREE) == 0);   }

    void printDebug(StringBuilder *);


    static uint32_t  total_transfers;
    static uint32_t  failed_transfers;
    static uint16_t  spi_wait_timeout;   // In microseconds. Per-byte.
    //static uint32_t  spi_cs_delay;       // In microseconds.

    static void buildDMAMembers();


  private:
    uint8_t  flags       = SPI_XFER_FLAG_NO_FLAGS;   // No flags set.

    int8_t init_dma();

    /**
    * How long is this transaction including the addressing overhead?
    *
    * @return the full length of the transaction, including addressing overhead.
    */
    uint8_t total_len() {     return buf_len + ((bus_addr > 255) ? 2 : 1);    }

    bool wait_with_timeout();


    static void enableSPI_DMA(bool enable);
};

#endif  // __SPI_BUS_OP_H__
