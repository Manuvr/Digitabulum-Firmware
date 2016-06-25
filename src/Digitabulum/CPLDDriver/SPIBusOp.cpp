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
uint32_t STATIC_SINK = 0;


/*
*
*/
void DMA2_Stream2_IRQHandler(void) {
  Kernel::log("DMA2_Stream2_IRQHandler()\n");
  __HAL_DMA_DISABLE_IT(&_dma_r_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
  __HAL_DMA_DISABLE_IT(&_dma_w_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
}


/*
*
*/
void DMA2_Stream3_IRQHandler(void) {
  Kernel::log("DMA2_Stream3_IRQHandler()\n");
  __HAL_DMA_DISABLE_IT(&_dma_r_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
  __HAL_DMA_DISABLE_IT(&_dma_w_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
}




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

  //__HAL_DMA_ENABLE_IT(&_dma_r_handle, DMA_IT_TC);
  //__HAL_DMA_ENABLE_IT(&_dma_w_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
}


/**
* Used to disable the DMA IRQs at the NVIC.
*
* @param bool enable the interrupts?
*/
void SPIBusOp::enableSPI_DMA(bool enable) {
  if (enable) {
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  }
  else {
    NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    NVIC_DisableIRQ(DMA2_Stream3_IRQn);
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
SPIBusOp::SPIBusOp(BusOpcode nu_op, BusOpCallback* requester) {
  wipe();
  this->opcode          = nu_op;
  callback = requester;
}


/**
* Destructor
* Should be nothing to do here. If this is DMA'd, we expect the referenced buffer
*   to be managed by the class that creates these objects.
*
* Moreover, sometimes instances of this class will be preallocated, and never torn down.
*/
SPIBusOp::~SPIBusOp() {
  if (profile()) {
    debug_log.concat("Destroying an SPI job that was marked for profiling:\n");
    printDebug(&debug_log);
  }
  if (debug_log.length() > 0) Kernel::log(&debug_log);
}



/**
* Set the buffer parameters. Note the there is only ONE buffer, despite this
*   bus being full-duplex.
*
* @param  buf The transfer buffer.
* @param  len The length of the buffer.
*/
void SPIBusOp::setBuffer(uint8_t *buf, uint8_t len) {
  this->buf     = buf;
  this->buf_len = len;
}


/**
* This set of parameters is used for IMU access.
*
* @param  _dev_addr  The first CPLD transfer parameter.
* @param  _xfer_len  The second CPLD transfer parameter.
* @param  _dev_count The third CPLD transfer parameter.
* @param  _reg_addr  The fourth CPLD transfer parameter.
*/
void SPIBusOp::setParams(uint8_t _dev_addr, uint8_t _xfer_len, uint8_t _dev_count, uint8_t _reg_addr) {
  _param_len     = 4;
  xfer_params[0] = _dev_addr;
  xfer_params[1] = _xfer_len;
  xfer_params[2] = _dev_count;
  xfer_params[3] = _reg_addr;
}


/**
* This set of parameters is used for internal CPLD register access.
*
* @param  _reg_addr The first CPLD transfer parameter.
* @param  _val      The second CPLD transfer parameter.
*/
void SPIBusOp::setParams(uint8_t _reg_addr, uint8_t _val) {
  _param_len     = 2;
  xfer_params[0] = _reg_addr;
  xfer_params[1] = _val;
  xfer_params[2] = 0;
  xfer_params[3] = 0;
  this->buf      = NULL;
  this->buf_len  = 0;
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

  if (opcode == BusOpcode::RX) {
    _dma_r_handle.Init.MemInc = DMA_MINC_DISABLE;
    _dma_w_handle.Init.MemInc = DMA_MINC_DISABLE;

    //DMA_InitStructure_Read.DMA_Memory0BaseAddr    = (uint32_t) buf;

    // We still need a transmit DMA operation to send the transfer parameters.
    //DMA_InitStructure_Write.DMA_Memory0BaseAddr   = (uint32_t) &STATIC_ZERO;
  }
  else if (opcode == BusOpcode::TX) {
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

  if (opcode == BusOpcode::RX) {
    HAL_DMA_Start_IT(&_dma_r_handle, (uint32_t) hspi1.pRxBuffPtr, (uint32_t) buf, (uint32_t) buf_len);
    HAL_DMA_Start_IT(&_dma_w_handle, (uint32_t) buf, (uint32_t) hspi1.pTxBuffPtr, (uint32_t) buf_len);
  }
  else if (opcode == BusOpcode::TX) {
    HAL_DMA_Start_IT(&_dma_w_handle, (uint32_t) buf, (uint32_t) hspi1.pTxBuffPtr, (uint32_t) buf_len);
  }

  return 0;
}



/**
* Wipes this bus operation so it can be reused.
* Be careful not to blow away the flags that prevent us from being reaped.
*/
void SPIBusOp::wipe() {
  set_state(XferState::IDLE);
  // We need to preserve flags that deal with memory management.
  flags       = flags & (SPI_XFER_FLAG_NO_FREE | SPI_XFER_FLAG_PREALLOCATE_Q);
  xfer_fault  = XferFault::NONE;
  opcode      = BusOpcode::UNDEF;
  buf_len     = 0;
  buf         = NULL;
  callback    = NULL;

  _param_len     = 0;
  xfer_params[0] = 0;
  xfer_params[1] = 0;
  xfer_params[2] = 0;
  xfer_params[3] = 0;

  profile(false);
}


/**
* Ensure that the bus is free prior to beginning a new operation. This function
*   will return after spi_wait_timeout milliseconds.
*
* @return true if the bus is available. False otherwise.
*/
bool SPIBusOp::wait_with_timeout() {
  uint32_t to_mark = micros();
  uint32_t timeout_val = (2 * spi_wait_timeout) + (buf_len * spi_wait_timeout);
  uint32_t m_mark = micros();
  while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) && ((max(to_mark, m_mark) - min(to_mark, m_mark)) <= timeout_val) ) {
    m_mark = micros();
  } // wait until bus is not busy, JIC.
  if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)) {
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
  if (0 == _param_len) {
    // Obvious invalidity. We must have at least one transfer parameter.
    abort(XferFault::BAD_PARAM);
    return -1;
  }

  if (!wait_with_timeout()) {
    Kernel::log("SPI op aborted before taking bus control.\n");
    abort(XferFault::BUS_BUSY);
    return -1;
  }

  set_state(XferState::INITIATE);  // Indicate that we now have bus control.

  if ((opcode == BusOpcode::TX) || (_param_len > 2)) {
    if (0 == buf_len) {
      // If this transfer is all that we are going to do...
      set_state(XferState::IO_WAIT);
    }
    else {
      // Otherwise, let the ISR feed the next DMA operation.
      set_state(XferState::ADDR);
    }
    // If we don't care about the values returning from the bus, our task is easy.
    if (_param_len <= 2) {
      HAL_SPI_Transmit_IT(&hspi1, (uint8_t*) xfer_params, _param_len);
    }
    else {
    // If we DO care about the return values, and the buffer will be
    //   required to capture it all.
      buf[0] = xfer_params[0];
      buf[1] = xfer_params[1];
      buf[2] = xfer_params[2];
      buf[3] = xfer_params[3];
      HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) buf, ((uint8_t*) buf+8), _param_len + (xfer_params[2] * xfer_params[1]));
    }
  }
  else {
    set_state(XferState::IO_WAIT);
    // If we do care, and our transfer length is half the array size, we won't
    //   bother with DMA, as we can accomplish the task on a single ISR.
    if (_param_len <= 2) {
      HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) xfer_params, ((uint8_t*) xfer_params+2), 2);
    }
    else {
      buf[0] = xfer_params[0];
      buf[1] = xfer_params[1];
      buf[2] = xfer_params[2];
      buf[3] = xfer_params[3];
      HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) buf, ((uint8_t*) buf+8), _param_len + (xfer_params[2] * xfer_params[1]));
    }
  }

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
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    if (buf_len > 1) {
      // We have DMA cruft to clean.
      enableSPI_DMA(false);
      __HAL_DMA_DISABLE(&_dma_r_handle);
      __HAL_DMA_CLEAR_FLAG(&_dma_r_handle, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6);

      __HAL_DMA_DISABLE(&_dma_w_handle);
      __HAL_DMA_CLEAR_FLAG(&_dma_w_handle, DMA_FLAG_TCIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TEIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_FEIF3_7);
    }
  }

  //time_ended = micros();
  total_transfers++;
  xfer_state = XferState::COMPLETE;
  ((CPLDDriver*) cpld)->step_queues();
  return 0;
}


/**
* This will mark the bus operation complete with a given error code.
*
* @param  cause A failure code to mark the operation with.
* @return 0 on success. Non-zero on failure.
*/
int8_t SPIBusOp::abort(XferFault cause) {
  SPIBusOp::failed_transfers++;
  xfer_fault = cause;
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

  debug_log.concatf("advance_operation(0x%08x, 0x%02x)\n\t %s\n\t status: 0x%08x\n", status_reg, data_reg, getStateString(), (unsigned long) hspi1.State);
  Kernel::log(&debug_log);

  /* These are our transfer-size-invariant cases. */
  switch (xfer_state) {
    case XferState::COMPLETE:
      //if (profile()) transition_time_COMPLETE     = micros();
      abort(XferFault::HUNG_IRQ);
      return 0;

    case XferState::IO_WAIT:
      markComplete();
      return 0;

    case XferState::FAULT:
      return 0;

    case XferState::QUEUED:
    case XferState::ADDR:
    case XferState::STOP:
    case XferState::UNDEF:

    /* Below are the states that we shouldn't be in at this point... */
    case XferState::INITIATE:
    case XferState::IDLE:
      abort(XferFault::ILLEGAL_STATE);
      return 0;
  }

//  if (buf_len == 1) {
//    /*
//    * This is the IRQ-only block.
//    */
//    if (profile()) debug_log.concatf("IRQ  %s\t status: 0x%08x\n", getStateString(), (unsigned long) status_reg);
//    switch (xfer_state) {
//      case XferState::ADDR:
//        break;
//
//      case XferState::STOP:
//        markComplete();
//        //if (profile()) transition_time_STOP = micros();
//        break;
//
//      /* Below are the states that we shouldn't be in at this point... */
//      case XferState::IO_WAIT:
//      default:
//        abort(XferFault::ILLEGAL_STATE);
//        break;
//    }
//  }
//
//  else {
//    /*
//    * This is the DMA block.
//    */
//    if (profile()) {
//      uint16_t count_0 = __HAL_DMA_GET_COUNTER(&_dma_r_handle);
//      debug_log.concatf("DMA  %s\t DMA0: %d \t buf_len: %d \t status: 0x%08x\n", getStateString(), (uint16_t) count_0, buf_len, (unsigned long) hspi1.State);
//    }
//
//    switch (xfer_state) {
//      case XferState::ADDR:
//        xfer_state = XferState::IO_WAIT;   // We will only ever end up here ONCE per job.
//
//        wait_with_timeout();    // Just in case the bus is still running (it ought not be).
//
//        if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)) {
//          //data_reg = hspi1->DR;   // Clear the Rx flag (if set).
//          //debug_log.concatf("\t DMA had to wait on a byte before address setup: 0x%02x. SR is now 0x%04x \n", data_reg, hspi1->SR);
//        }
//
//        init_dma();
//        //SPI_I2S_DMACmd(hspi1, SPI_I2S_DMAReq_Rx, ENABLE);
//        //SPI_I2S_DMACmd(hspi1, SPI_I2S_DMAReq_Tx, ENABLE);
//        enableSPI_DMA(true);
//        break;
//
//      case XferState::IO_WAIT:
//        //if (profile()) transition_time_DMA_WAIT = micros();
//        if (0 == __HAL_DMA_GET_COUNTER(&_dma_r_handle)) {
//          xfer_state = XferState::STOP;
//          if (HAL_DMA_GetState(&_dma_r_handle) == HAL_DMA_STATE_RESET) {
//            markComplete();
//          }
//          else {
//            //if (profile()) debug_log.concat("\t DMA looks sick...\n");
//            abort(XferFault::DMA_FAULT);
//          }
//        }
//        else {
//          //if (profile()) debug_log.concatf("\tJob 0x%04x looks incomplete, but DMA is IRQ. Advancing state to STOP for next IRQ and hope.\n", txn_id);
//          debug_log.concat("\tJob looks incomplete, but DMA is IRQ. Advancing state to STOP for next IRQ and hope.\n");
//          abort(XferFault::DMA_FAULT); // TODO: WRONG
//        }
//        break;
//
//      case XferState::STOP:
//        //if (profile()) transition_time_STOP = micros();
//        markComplete();
//        break;
//
//      default:
//        abort(XferFault::ILLEGAL_STATE);
//        break;
//    }
//  }
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
  //if (XferState::COMPLETE == xfer_state) {
  //  output->concatf("\t completed (uS)   %u\n",   (unsigned long) time_ended - time_began);
  //}
  output->concatf("\t callback set      %s\n", (callback ? "yes":"no"));
  output->concatf("\t will reap?        %s\n", shouldReap()?"yes":"no");
  output->concatf("\t ret to prealloc?  %s\n", returnToPrealloc()?"yes":"no");
  output->concatf("\t param_len         %d\n", _param_len);
  output->concat("\t params            ");

  if (_param_len > 0) {
    for (uint8_t i = 0; i < _param_len; i++) {
      output->concatf("0x%02x ", xfer_params[i]);
    }
  }

  output->concatf("\n\t buf_len           %d\n", buf_len);

  if (buf_len > 0) {
    output->concatf("\t buf *(0x%08x) ", (uint32_t) buf);
    for (uint8_t i = 0; i < buf_len; i++) {
      output->concatf("0x%02x ", (uint8_t) *(buf + i));
    }
  }
  output->concat("\n\n");
}
