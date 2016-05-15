/*
File:   SPIBusOp.cpp
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


*/


#include "SPIBusOp.h"
#include "CPLDDriver.h"
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_spi.h>
#include <stm32f7xx_hal.h>

extern volatile CPLDDriver* cpld;
extern SPI_HandleTypeDef hspi1;


/****************************************************************************************************
* Out-of-class                                                                                      *
****************************************************************************************************/
StringBuilder debug_log;   // TODO: Relocate this to a static member.

/* These are the basic DMA init parameters that will be needed by the SPI classes. */
DMA_HandleTypeDef _dma_r_handle;
DMA_HandleTypeDef _dma_w_handle;

/*
* Notated like a const, but should NOT be a const, because we use this as a DMA read sink as well
*   as a sort of /dev/zero. This never contains any meaningful data.
*/
uint32_t STATIC_ZERO = 0;


/****************************************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here. Initializers first, functions second.
****************************************************************************************************/

/* Static Initializers */
uint32_t SPIBusOp::total_transfers  = 0;  // How many total SPI transfers have we seen?
uint32_t SPIBusOp::failed_transfers = 0;  // How many failed SPI transfers have we seen?
uint16_t SPIBusOp::spi_wait_timeout = 20; // In microseconds. Per-byte.
//uint32_t SPIBusOp::spi_cs_delay     = 0;  // How many microseconds to delay before CS disassertion?


/**
* This is called upon CPLD instantiation to build the DMA init structures for the bus operation.
* This data is retained following a wipe(), so we eat the minor memory penalty so that the
*   same init doesn't need to be done a million times for repeat operations.
*/
void SPIBusOp::buildDMAMembers() {
  // We setup the interrupt-driven stuff so we aren't surprised on the first use of the bus.
  enableSPI_DMA(false);

  _dma_r_handle.Instance                  = DMA2_Stream2;
  _dma_r_handle.Init.Direction            = DMA_PERIPH_TO_MEMORY;   // Receive
  _dma_r_handle.Init.Channel              = DMA_CHANNEL_3;
  _dma_r_handle.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
  _dma_r_handle.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
  _dma_r_handle.Init.Priority             = DMA_PRIORITY_HIGH;
  _dma_r_handle.Init.FIFOMode             = DMA_FIFOMODE_DISABLE;  // Required for differnt access-widths.
  _dma_r_handle.Init.FIFOThreshold        = DMA_FIFO_THRESHOLD_FULL;
  _dma_r_handle.Init.MemBurst             = DMA_MBURST_SINGLE;
  _dma_r_handle.Init.PeriphInc            = DMA_PINC_DISABLE;
  _dma_r_handle.Init.PeriphBurst          = DMA_PBURST_SINGLE;
  _dma_r_handle.Init.Mode                 = DMA_NORMAL;
  _dma_r_handle.Init.MemInc               = DMA_MINC_ENABLE;
  //DMA_InitStructure_Read.DMA_PeripheralBaseAddr  = (uint32_t) &SPI1->DR;

  _dma_w_handle.Instance                 = DMA2_Stream3;
  _dma_w_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;   // Transmit
  _dma_w_handle.Init.Channel             = DMA_CHANNEL_3;
  _dma_w_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  _dma_w_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  _dma_w_handle.Init.Priority            = DMA_PRIORITY_HIGH;
  _dma_w_handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;  // Required for differnt access-widths.
  _dma_w_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  _dma_w_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
  _dma_w_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  _dma_w_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  _dma_w_handle.Init.Mode                = DMA_NORMAL;
  _dma_w_handle.Init.MemInc              = DMA_MINC_ENABLE;
  //DMA_InitStructure_Write.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DR;

  __HAL_DMA_ENABLE_IT(&_dma_r_handle, DMA_IT_TC);
  __HAL_DMA_ENABLE_IT(&_dma_w_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
}


/**
* Used to disable the DMA IRQs at the NVIC.
*
* @param bool enable the interrupts?
*/
void SPIBusOp::enableSPI_DMA(bool enable) {
  if (!enable) {
    NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  }
  else {
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  }
}






/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/

/**
* Vanilla constructor that calls wipe().
*/
SPIBusOp::SPIBusOp() {
  wipe();
}


/**
* Constructor that does setup by parameters.
*
* @param  nu_op The opcode that dictates the bus operation we use
* @param  addr       The address of the target on the SPI bus.
* @param  buf        The place where the transaction data is to be stored.
* @param  len        The length of the transaction.
* @param  requester  The object to be notified when the bus operation completes with success.
*/
SPIBusOp::SPIBusOp(uint8_t nu_op, uint16_t addr, uint8_t *buf, uint8_t len, SPIOpCallback* requester) {
  this->opcode          = nu_op;
  this->buf             = buf;
  this->buf_len         = len;
  this->bus_addr        = addr;
  callback = requester;
  profile = false;
}


/**
* Destructor
* Should be nothing to do here. If this is DMA'd, we expect the referenced buffer
*   to be managed by the class that creates these objects.
*
* Moreover, sometimes instances of this class will be preallocated, and never torn down.
*/
SPIBusOp::~SPIBusOp() {
  if (profile) {
    debug_log.concat("Destroying an SPI job that was marked for profiling:\n");
    printDebug(&debug_log);
  }
  if (debug_log.length() > 0) Kernel::log(&debug_log);
}



/**
* Mutate the predefined DMA init structs to be specific to this particular operation.
*
* @return 0 on success, or non-zero on failure.
*/
int8_t SPIBusOp::init_dma() {
  if (HAL_DMA_GetState(&_dma_w_handle) != HAL_DMA_STATE_RESET) __HAL_DMA_DISABLE(&_dma_w_handle);
  if (HAL_DMA_GetState(&_dma_r_handle) != HAL_DMA_STATE_RESET) __HAL_DMA_DISABLE(&_dma_r_handle);

  uint32_t _origin_buf = 0;
  uint32_t _target_buf = 0;

  if (opcode == SPI_OPCODE_READ) {
    _dma_r_handle.Init.MemInc = DMA_MINC_DISABLE;
    _dma_w_handle.Init.MemInc = DMA_MINC_DISABLE;

    //DMA_InitStructure_Read.DMA_Memory0BaseAddr    = (uint32_t) buf;

    // We still need a transmit DMA operation to send the transfer parameters.
    //DMA_InitStructure_Write.DMA_Memory0BaseAddr   = (uint32_t) &STATIC_ZERO;
  }
  else if (opcode == SPI_OPCODE_WRITE) {
    _dma_r_handle.Init.MemInc = DMA_MINC_DISABLE;
    _dma_w_handle.Init.MemInc = DMA_MINC_ENABLE;

    //DMA_InitStructure_Write.DMA_Memory0BaseAddr   = (uint32_t) buf;

    // For now, we are reliant on the Rx DMA IRQ. Tx IRQ is never used. So when
    // transmitting, we need to sink the read bytes until we do something smarter.
    //DMA_InitStructure_Read.DMA_Memory0BaseAddr    = (uint32_t) &STATIC_ZERO;
  }
  else {
    return -1;
  }

  while (HAL_DMA_GetState(&_dma_w_handle) != HAL_DMA_STATE_RESET) {}  // TODO: Might-could cut this.
  HAL_DMA_Init(&_dma_w_handle);

  while (HAL_DMA_GetState(&_dma_r_handle) != HAL_DMA_STATE_RESET) {}  // TODO: Might-could cut this.
  HAL_DMA_Init(&_dma_r_handle);

  if (opcode == SPI_OPCODE_READ) {
    HAL_DMA_Start_IT(&_dma_r_handle, (uint32_t) hspi1.pRxBuffPtr, (uint32_t) buf, (uint32_t) buf_len);
    HAL_DMA_Start_IT(&_dma_w_handle, (uint32_t) buf, (uint32_t) hspi1.pTxBuffPtr, (uint32_t) buf_len);
  }
  else if (opcode == SPI_OPCODE_WRITE) {
    HAL_DMA_Start_IT(&_dma_w_handle, (uint32_t) buf, (uint32_t) hspi1.pTxBuffPtr, (uint32_t) buf_len);
  }

  return 0;
}



/**
* Wipes this bus operation so it can be reused.
* Be careful not to blow away the flags that prevent us from being reaped.
*/
void SPIBusOp::wipe() {
  set_state(SPI_XFER_STATE_IDLE);
  // We need to preserve flags that deal with memory management.
  flags       = flags & (SPI_XFER_FLAG_NO_FREE | SPI_XFER_FLAG_PREALLOCATE_Q);
  err_code    = SPI_XFER_ERROR_NONE;
  opcode      = SPI_OPCODE_UNDEFINED;
  bus_addr    = 0x0000;
  buf_len     = 0;
  buf         = NULL;
  reg_idx     = -1;
  callback    = NULL;
  profile     = false;
}


bool SPIBusOp::wait_with_timeout() {
  uint32_t to_mark = micros();
  uint32_t timeout_val = (2 * spi_wait_timeout) + (buf_len * spi_wait_timeout);
  uint32_t m_mark = micros();
  while(__HAL_SPI_GET_FLAG(&hspi1, HAL_SPI_STATE_BUSY) && ((max(to_mark, m_mark) - min(to_mark, m_mark)) <= timeout_val) ) {
    m_mark = micros();
  } // wait until bus is not busy, JIC.
  if (__HAL_SPI_GET_FLAG(&hspi1, HAL_SPI_STATE_BUSY)) {
    debug_log.concatf("SPI Bus timeout after %uuS.\n", timeout_val);
    return false;
  }
  else {
    return true;
  }
}


/****************************************************************************************************
*     8                  eeeeee
*     8  eeeee eeeee     8    e eeeee eeeee eeeee eeeee  eeeee e
*     8e 8  88 8   8     8e     8  88 8   8   8   8   8  8  88 8
*     88 8   8 8eee8e    88     8   8 8e  8   8e  8eee8e 8   8 8e
* e   88 8   8 88   8    88   e 8   8 88  8   88  88   8 8   8 88
* 8eee88 8eee8 88eee8    88eee8 8eee8 88  8   88  88   8 8eee8 88eee
****************************************************************************************************/

// TODO: This should eventually be the only means of moving state within this class.
bool SPIBusOp::set_state(uint8_t nu) {
  if (nu != xfer_state) {
    // Depending on our new state, we might clean up, or take other special action.
    switch (nu) {
      case SPI_XFER_STATE_IDLE:
        // Reset the job for next use. Assume we will be re-used.
        err_code    = SPI_XFER_ERROR_NONE;
        if (debug_log.length() > 0) {
          printDebug(&debug_log);
          Kernel::log(&debug_log);
        }
        break;

      case SPI_XFER_STATE_INITIATE:
      case SPI_XFER_STATE_ADDR:
      case SPI_XFER_STATE_DMA_WAIT:
      case SPI_XFER_STATE_STOP:
      case SPI_XFER_STATE_COMPLETE:
        break;

      default:    // Nope.
        return false;
        break;
    }
    // Assign the new state.
    xfer_state = nu;
  }
  return true;
}


// Useful trick to mask warnings that the compiler raises, but which we know are
//   intentional.
//   http://stackoverflow.com/questions/3378560/how-to-disable-gcc-warnings-for-a-few-lines-of-code
//   https://gcc.gnu.org/onlinedocs/gcc/Diagnostic-Pragmas.html
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

/**
* Calling this member will cause the bus operation to be started.
*
* @return 0 on success, or non-zero on failure.
*/
int8_t SPIBusOp::begin() {
  //time_began    = micros();

  if (0 == buf_len) {
    abort(SPI_XFER_ERROR_BAD_XFER_PARM);
    return -1;
  }

  if (!wait_with_timeout()) {
    debug_log.concat("SPI op aborted before taking bus control.\n");
    return -1;
  }

  //assertCS(true);

  /* In this case, we need to clear any pending interrupts for the SPI, and to do that, we must
     read this register, even though we don't care about the result.  */
  volatile uint8_t throw_away;
  if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)) {} // throw_away = hspi1->DR;   // Clear the Rx flag (if set).

  /* The peripheral should be totally clear at this point. Since the TX FIFO is two slots deep,
     we're going to shovel in both bytes if we have a 16-bit address. Since the ISR only calls
     us back when the bus goes idle, we don't need to worry about tracking the extra IRQ. */
  xfer_state = SPI_XFER_STATE_ADDR;
  if (bus_addr > 255) {
    //hspi1->DR = (uint8_t) (bus_addr >> 8);
  }

  if (!wait_with_timeout()) {
    debug_log.concatf("SPI op aborted halfway into ADDR phase?!\n");
    abort();
    return -1;
  }
  /* Shovel in the last (or only) address byte... */
  //hspi1->DR = (uint8_t) bus_addr;

  return 0;
}

#pragma GCC diagnostic pop




/**
* Marks this bus operation complete.
*
* Need to remember: this gets called in the event of ANY condition that ends this job. And
*   that includes abort() where the bus operation was never begun, and SOME OTHER job has
*   control of the bus.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t SPIBusOp::markComplete() {
  if (has_bus_control() || (((CPLDDriver*) cpld)->current_queue_item == this) ) {
    // If this job has bus control, we need to release the bus and tidy up IRQs.
    if (buf_len > 1) {
      // We have DMA cruft to clean.
      enableSPI_DMA(false);
      __HAL_DMA_DISABLE(&_dma_r_handle);
      __HAL_DMA_CLEAR_FLAG(&_dma_r_handle, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6);

      __HAL_DMA_DISABLE(&_dma_w_handle);
      __HAL_DMA_CLEAR_FLAG(&_dma_w_handle, DMA_FLAG_TCIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TEIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_FEIF3_7);
    }

    //assertCS(false);
  }

  //time_ended = micros();
  total_transfers++;
  xfer_state = SPI_XFER_STATE_COMPLETE;
  ((CPLDDriver*) cpld)->step_queues(false);
  return 0;
}


/**
* This will mark the bus operation complete with a given error code.
*
* @param  cause A failure code to mark the operation with.
* @return 0 on success. Non-zero on failure.
*/
int8_t SPIBusOp::abort(uint8_t cause) {
  SPIBusOp::failed_transfers++;
  err_code = cause;
  debug_log.concatf("SPI job aborted at state %s. Cause: %s.\n", getStateString(), getErrorString());
  printDebug(&debug_log);
  return markComplete();
}





/**
* Called from the ISR to advance this operation on the bus.
* Stay brief. We are in an ISR.
*
* @return 0 on success. Non-zero on failure.
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
int8_t SPIBusOp::advance_operation(uint32_t status_reg, uint8_t data_reg) {
  int8_t return_value = 0;

  /* These are our transfer-size-invariant cases. */
  switch (xfer_state) {
    case SPI_XFER_STATE_COMPLETE:
      //if (profile) transition_time_COMPLETE     = micros();
      abort(SPI_XFER_ERROR_HANGING_IRQ);
      return 0;

    /* Below are the states that we shouldn't be in at this point... */
    case SPI_XFER_STATE_INITIATE:
    case SPI_XFER_STATE_IDLE:
      abort(SPI_XFER_ERROR_ILLEGAL_STATE);
      return 0;
  }

  if (buf_len == 1) {
    /*
    * This is the IRQ-only block.
    */
    if (profile) debug_log.concatf("IRQ  %s\t status: 0x%08x\n", getStateString(), (unsigned long) status_reg);
    switch (xfer_state) {
      case SPI_XFER_STATE_ADDR:
        break;

      case SPI_XFER_STATE_STOP:
        markComplete();
        //if (profile) transition_time_STOP = micros();
        break;

      /* Below are the states that we shouldn't be in at this point... */
      case SPI_XFER_STATE_DMA_WAIT:
      default:
        abort(SPI_XFER_ERROR_ILLEGAL_STATE);
        break;
    }
  }

  else {
    /*
    * This is the DMA block.
    */
    if (profile) {
      uint16_t count_0 = __HAL_DMA_GET_COUNTER(&_dma_r_handle);
      debug_log.concatf("DMA  %s\t DMA0: %d \t buf_len: %d \t status: 0x%08x\n", getStateString(), (uint16_t) count_0, buf_len, (unsigned long) hspi1.State);
    }

    switch (xfer_state) {
      case SPI_XFER_STATE_ADDR:
        xfer_state = SPI_XFER_STATE_DMA_WAIT;   // We will only ever end up here ONCE per job.

        wait_with_timeout();    // Just in case the bus is still running (it ought not be).

        if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)) {
          //data_reg = hspi1->DR;   // Clear the Rx flag (if set).
          //debug_log.concatf("\t DMA had to wait on a byte before address setup: 0x%02x. SR is now 0x%04x \n", data_reg, hspi1->SR);
        }

        init_dma();
        //SPI_I2S_DMACmd(hspi1, SPI_I2S_DMAReq_Rx, ENABLE);
        //SPI_I2S_DMACmd(hspi1, SPI_I2S_DMAReq_Tx, ENABLE);
        enableSPI_DMA(true);
        break;

      case SPI_XFER_STATE_DMA_WAIT:
        //if (profile) transition_time_DMA_WAIT = micros();
        if (0 == __HAL_DMA_GET_COUNTER(&_dma_r_handle)) {
          xfer_state = SPI_XFER_STATE_STOP;
          if (HAL_DMA_GetState(&_dma_r_handle) == HAL_DMA_STATE_RESET) {
            markComplete();
          }
          else {
            //if (profile) debug_log.concat("\t DMA looks sick...\n");
            abort(SPI_XFER_ERROR_DMA_TIMEOUT);
          }
        }
        else {
          //if (profile) debug_log.concatf("\tJob 0x%04x looks incomplete, but DMA is IRQ. Advancing state to STOP for next IRQ and hope.\n", txn_id);
          debug_log.concat("\tJob looks incomplete, but DMA is IRQ. Advancing state to STOP for next IRQ and hope.\n");
          abort(SPI_XFER_ERROR_DMA_TIMEOUT); // TODO: WRONG
        }
        break;

      case SPI_XFER_STATE_STOP:
        //if (profile) transition_time_STOP = micros();
        markComplete();
        break;

      default:
        abort(SPI_XFER_ERROR_ILLEGAL_STATE);
        break;
    }
  }

  return return_value;
}
#pragma GCC diagnostic pop



/****************************************************************************************************
* Memory-management and cleanup support.                                                            *
****************************************************************************************************/



/**
* The client class calls this fxn to set this object's post-completion behavior.
* If this fxn is never called, the default behavior of the class is to allow itself to be free()'d.
*
* This flag is preserved by wipe().
*
* @param  nu_reap_state Pass false to cause the bus manager to leave this object alone.
* @return true if the bus manager class should free() this object. False otherwise.
*/
bool SPIBusOp::shouldReap(bool nu_reap_state) {
  flags = (nu_reap_state) ? (flags & (uint8_t) ~SPI_XFER_FLAG_NO_FREE) : (flags | SPI_XFER_FLAG_NO_FREE);
  return ((flags & SPI_XFER_FLAG_NO_FREE) == 0);
}


/**
* Call this on instantiation with a value of 'true' to disable reap, and indicate to the bus manager that
*   it ought to return this object to the preallocation queue following completion.
* If this fxn is never called, the default behavior of the class is to decline to be recycled
*   into the prealloc queue.
* Calling this fxn implies an opposite return value for calls to shouldReap().
*
* This flag is preserved by wipe().
*
* @param  nu_prealloc_state Pass true to inform the bus manager that this object was preallocated.
* @return true if the bus manager class should return this object to its preallocation queue.
*/
bool SPIBusOp::returnToPrealloc(bool nu_prealloc_state) {
  flags = (!nu_prealloc_state) ? (flags & (uint8_t) ~SPI_XFER_FLAG_PREALLOCATE_Q) : (flags | SPI_XFER_FLAG_PREALLOCATE_Q);
  shouldReap(!nu_prealloc_state);
  return (flags & SPI_XFER_FLAG_PREALLOCATE_Q);
}


/**
* This is a means for a client class to remind itself if the write operation advanced the
*   device's registers or not.
*
* This flag is cleared if the operation is wipe()'d.
*
* @param  nu_reap_state Pass false to cause the bus manager to leave this object alone.
* @return true if the bus manager class should free() this object. False otherwise.
*/
bool SPIBusOp::devRegisterAdvance(bool _reg_advance) {
  flags = (_reg_advance) ? (flags | SPI_XFER_FLAG_DEVICE_REG_INC) : (flags & (uint8_t) ~SPI_XFER_FLAG_DEVICE_REG_INC);
  return ((flags & SPI_XFER_FLAG_DEVICE_REG_INC) == 0);
}




/****************************************************************************************************
* These functions are for logging support.                                                          *
****************************************************************************************************/

/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void SPIBusOp::printDebug(StringBuilder *output) {
  if (NULL == output) return;
  output->concatf("-----SPIBusOp 0x%08x (%s)------------\n", (uint32_t) this, getOpcodeString());
  output->concatf("\t xfer_state        %s\n\t err               %s\n", getStateString(), getErrorString());
  //if (SPI_XFER_STATE_COMPLETE == xfer_state) {
  //  output->concatf("\t completed (uS)   %u\n",   (unsigned long) time_ended - time_began);
  //}
  output->concatf("\t callback set      %s\n", (callback ? "yes":"no"));
  output->concatf("\t will reap?        %s\n", shouldReap()?"yes":"no");
  output->concatf("\t ret to prealloc?  %s\n", returnToPrealloc()?"yes":"no");
  output->concatf("\t reg_idx           %d\n", reg_idx);
  output->concatf("\t bus_addr          0x%04x\n", bus_addr);
  output->concatf("\t buf_len           %d\n", buf_len);
  output->concatf("\t buf *(0x%08x) ", (uint32_t) buf);

  if (buf_len > 0) {
    for (uint8_t i = 0; i < buf_len; i++) {
      output->concatf("0x%02x ", (uint8_t) *(buf + i));
    }
  }
  output->concat("\n\n");
}


/**
* Logging support.
*
* @return a const char* containing a human-readable representation of the data.
*/
const char* SPIBusOp::getStateString() {
  switch (xfer_state) {
    case SPI_XFER_STATE_IDLE:        return "IDLE";
    case SPI_XFER_STATE_INITIATE:    return "INITIATE";
    case SPI_XFER_STATE_ADDR:        return "ADDR";
    case SPI_XFER_STATE_DMA_WAIT:    return "DMA_WAIT";
    case SPI_XFER_STATE_STOP:        return "STOP";
    case SPI_XFER_STATE_COMPLETE:    return "COMPLETE";
    default:                         return "<UNDEFINED STATE>";
  }
}

/**
* Logging support.
*
* @return a const char* containing a human-readable representation of the data.
*/
const char* SPIBusOp::getOpcodeString() {
  switch (opcode) {
    case SPI_OPCODE_UNDEFINED:  return "UNINITIALIZED";
    case SPI_OPCODE_READ:       return "READ";
    case SPI_OPCODE_WRITE:      return "WRITE";
    default:                    return "<UNDEFINED OPCODE>";
  }
}

/**
* Logging support.
*
* @return a const char* containing a human-readable representation of the data.
*/
const char* SPIBusOp::getErrorString() {
  switch (err_code) {
    case SPI_XFER_ERROR_QUEUE_FLUSH:   return "QUEUE_FLUSH";
    case SPI_XFER_ERROR_NONE:          return "NONE";
    case SPI_XFER_ERROR_DMA_TIMEOUT:   return "DMA_TIMEOUT";
    case SPI_XFER_ERROR_ILLEGAL_STATE: return "ILLEGAL_STATE";
    case SPI_XFER_ERROR_HANGING_IRQ:   return "HANGING_IRQ";
    case SPI_XFER_ERROR_BAD_XFER_PARM: return "BAD_XFER_PARM";
    case SPI_XFER_ERROR_BUS_FAULT:     return "BUS_FAULT";
    case SPI_XFER_ERROR_DMA_FAILURE:   return "DMA_FAILURE";
    case SPI_XFER_ERROR_NO_REASON:     return "NO_REASON";
    default:                           return "<UNDEFINED ERROR>";
  }
}
