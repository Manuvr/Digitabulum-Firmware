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

  /*
  * These flags are hosted by the member in the BusOp class.
  * Be careful when scrubing the field between re-use.
  */
  #define SPI_XFER_FLAG_NO_FLAGS        0x00   // By default, there are no flags set.
  #define SPI_XFER_FLAG_NO_FREE         0x01   // If set in a transaction's flags field, it will not be free()'d.
  #define SPI_XFER_FLAG_PREALLOCATE_Q   0x02   // If set, indicates this object should be returned to the prealloc queue.
  #define SPI_XFER_FLAG_DEVICE_REG_INC  0x40   // If set, indicates this operation advances addresses in the target device.
  #define SPI_XFER_FLAG_PROFILE         0x80   // If set, this bus operation shall be profiled.


  #define SPI_CALLBACK_ERROR    -1
  #define SPI_CALLBACK_NOMINAL   0
  #define SPI_CALLBACK_RECYCLE   1


/*
* This class represents a single transaction on the SPI bus.
*/
class SPIBusOp : public BusOp {
  public:
    BusOpCallback* callback = NULL;  // Which class gets pinged when we've finished?

    //uint32_t time_began    = 0;   // This is the time when bus access begins.
    //uint32_t time_ended    = 0;   // This is the time when bus access stops (or is aborted).

    SPIBusOp();
    SPIBusOp(BusOpcode nu_op, BusOpCallback* requester);
    ~SPIBusOp();

    /* Job control functions. */
    int8_t begin();
    int8_t markComplete();

    void setBuffer(uint8_t *buf, uint8_t len);

    void setParams(uint8_t _dev_addr, uint8_t _xfer_len, uint8_t _dev_count, uint8_t _reg_addr);
    void setParams(uint8_t _reg_addr, uint8_t _val);
    void setParams(uint8_t _reg_addr);

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
    * @return The address of the internal register this operation addresses.
    */
    inline uint8_t getRegAddr() {
      return ((4 == _param_len) ? xfer_params[3] : xfer_params[0] );
    };

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
    uint8_t  xfer_params[4];   // The address transfer lengths, preamble, etc...
    uint8_t  _param_len  = 0;  // The length of transfer parameters to send.
    uint8_t  flags       = 0;  // No flags set.

    int8_t init_dma();

    bool wait_with_timeout();


    static void enableSPI_DMA(bool enable);
};

#endif  // __SPI_BUS_OP_H__
