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

  // Bit[2] is the bit that indicates bus control.
  #define SPI_XFER_STATE_IDLE      0b00000000   // Bus op is waiting somewhere outside of the queue.
  #define SPI_XFER_STATE_INITIATE  0b00000001   // Waiting for initiation
  #define SPI_XFER_STATE_ADDR      0b00000100   // Sending the register address.
  #define SPI_XFER_STATE_DMA_WAIT  0b00000101   // Waiting for DMA operation to complete.
  #define SPI_XFER_STATE_STOP      0b00000110   // Release the CS line
  #define SPI_XFER_STATE_COMPLETE  0b00001000   // We have cleaned up everything and are awaiting reap.

  #define SPI_OPCODE_UNDEFINED     0x00
  #define SPI_OPCODE_READ          0x01
  #define SPI_OPCODE_WRITE         0x02

  #define SPI_XFER_ERROR_NONE           0x00   // No error on this transfer.
  #define SPI_XFER_ERROR_DMA_TIMEOUT    0x01   // DMA timeout.
  #define SPI_XFER_ERROR_NO_REASON      0x02   // No reason defined, but still errored.
  #define SPI_XFER_ERROR_QUEUE_FLUSH    0x03   // The work queue was flushed and this was a casualty.
  #define SPI_XFER_ERROR_BUS_FAULT      0x04   // The bus had a meltdown.
  #define SPI_XFER_ERROR_ILLEGAL_STATE  0x05   // The bus operation is in an illegal state.
  #define SPI_XFER_ERROR_HANGING_IRQ    0x06   // One too many IRQs happened for this operation.
  #define SPI_XFER_ERROR_BAD_XFER_PARM  0x07   // Did you try to send a 0-length message?
  #define SPI_XFER_ERROR_DMA_FAILURE    0x08   // Something went sideways with DMA that wasn't a timeout.

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
class SPIBusOp {
  public:
    SPIOpCallback* callback = NULL;               // Which class gets pinged when we've finished?
    XferState xfer_state = XferState::UNDEF;      // What state is this transfer in?
    BusOpcode  opcode    = BusOpcode::UNDEF;      // What is the particular operation being done?
    uint16_t bus_addr    = 0x0000;                // The address that this operation is directed toward.
    int16_t  reg_idx     = -1;                    // Optional register index. Makes callbacks faster.
    uint8_t* buf            = NULL;               // Pointer to the data buffer for the transaction.
    uint8_t  buf_len        = 0;                  // How large is the above buffer?

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
    inline int8_t abort() {    return abort(SPI_XFER_ERROR_NO_REASON); }
    int8_t abort(uint8_t cause);

    int8_t advance_operation(uint32_t status_reg, uint8_t data_reg);

    void wipe();

    /* Flag management fxns... */
    bool shouldReap(bool);    // Override to set the reap behavior.
    bool returnToPrealloc(bool);
    bool devRegisterAdvance(bool);

    /**
    * @return true if this operation experienced any abnormal condition.
    */
    inline bool isIdle() {       return (XferState::IDLE     == xfer_state);  }
    inline bool complete() {     return (XferState::COMPLETE == xfer_state);  }
    bool set_state(XferState);  // Set the state of this operation.

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
    * @return true if this operation experienced any abnormal condition.
    */
    inline bool hasError() {          return (SPI_XFER_ERROR_NONE != err_code);      }

    /**
    * The bus manager calls this fxn to decide if it ought to free this object after completion.
    *
    * @return true if the bus manager class should free() this object. False otherwise.
    */
    inline bool shouldReap() {        return ((flags & SPI_XFER_FLAG_NO_FREE) == 0);   }

    void printDebug(StringBuilder *);

    /* Logging support */
    const char* getErrorString();
    inline const char* getOpcodeString() {  return BusOp::getOpcodeString(opcode);     };
    inline const char* getStateString() {   return BusOp::getStateString(xfer_state);  };


    static uint32_t  total_transfers;
    static uint32_t  failed_transfers;
    static uint16_t  spi_wait_timeout;   // In microseconds. Per-byte.
    //static uint32_t  spi_cs_delay;       // In microseconds.

    static void buildDMAMembers();


  private:
    uint8_t  flags       = SPI_XFER_FLAG_NO_FLAGS;   // No flags set.
    int8_t   err_code    = SPI_XFER_ERROR_NONE;      // What is the error code when we've finished?

    int8_t init_dma();

    /**
    * How long is this transaction including the addressing overhead?
    *
    * @return the full length of the transaction, including addressing overhead.
    */
    uint8_t total_len() {     return buf_len + ((bus_addr > 255) ? 2 : 1);    }

    bool wait_with_timeout();

    /* This only works because of careful defines. Tread lightly. */
    inline bool has_bus_control() {
      return (
        (xfer_state == XferState::STOP) | (xfer_state == XferState::IO_WAIT) | \
        (xfer_state == XferState::INITIATE) | (xfer_state == XferState::ADDR)
      );
    }


    static void enableSPI_DMA(bool enable);
};

#endif  // __SPI_BUS_OP_H__
