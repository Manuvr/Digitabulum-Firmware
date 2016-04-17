#ifndef __SPI_BUS_OP_H__
#define __SPI_BUS_OP_H__
  /*
  * This is the struct that is used to keep bus operations on the SPI atomic. We provide
  *   the callback in the LSM9DS0_Common class because it is the greatest common denominator
  *   where the logical sensors and their registers are still differentiable.
  * This is in preparation for the conversion to interrupt mode, and then from there to DMA.
  */

  #include <inttypes.h>
  #include "stm32f7xx_hal_dma.h"
  #include "DataStructures/StringBuilder.h"
  #include "Drivers/DeviceWithRegisters/DeviceRegister.h"

  class SPIOpCallback;

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


  #define SPI_CALLBACK_ERROR     -1
  #define SPI_CALLBACK_NOMINAL   0
  #define SPI_CALLBACK_RECYCLE   1


/*
* This class represents a single transaction on the bus.
*/
class SPIBusOp {
  public:
    SPIOpCallback* callback = NULL;                  // Which class gets pinged when we've finished?
    uint8_t* buf            = NULL;                  // Pointer to the data buffer for the transaction.

    uint16_t bus_addr    = 0x0000;                   // The address that this operation is directed toward.
    int16_t  reg_idx     = -1;                       // Optional register index. Makes callbacks faster.

    uint8_t  opcode      = SPI_OPCODE_UNDEFINED;     // What is the particular operation being done?
    uint8_t  xfer_state  = SPI_XFER_STATE_IDLE;      // What state is this transfer in?

    //uint32_t time_began    = 0;   // This is the time when bus access begins.
    //uint32_t time_ended    = 0;   // This is the time when bus access stops (or is aborted).

    uint8_t  buf_len     = 0;                        // How large is the above buffer?
    bool           profile  = false;     // Set to true to profile this transaction.

    SPIBusOp();
    SPIBusOp(uint8_t nu_op, uint16_t addr, uint8_t *buf, uint8_t len, SPIOpCallback* requester);
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
    inline bool isIdle() {       return (SPI_XFER_STATE_IDLE == xfer_state);      }
    inline bool complete() {     return (SPI_XFER_STATE_COMPLETE == xfer_state);  }
    bool set_state(uint8_t);  // Set the state of this operation.

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
    const char* getOpcodeString();
    const char* getStateString();


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
    bool wait_txe_with_timeout();

    /* This only works because of careful defines. Tread lightly. */
    inline bool has_bus_control() {     return (xfer_state & 0x04);   }


    static void enableSPI_IRQ(bool enable);
    static void enableSPI_DMA(bool enable);

    static void assertCS(bool);   // TODO: Shove this functionality into NSS. Possible???

};

#endif
