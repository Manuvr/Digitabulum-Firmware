/*
File:   CPLDBusOp.cpp
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


#include "CPLDBusOp.h"
#include "CPLDDriver.h"


/*******************************************************************************
* Out-of-class                                                                 +
*******************************************************************************/
StringBuilder debug_log;   // TODO: Relocate this to a static member.


/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/
uint32_t CPLDBusOp::total_transfers  = 0;  // How many total SPI transfers have we seen?
uint32_t CPLDBusOp::failed_transfers = 0;  // How many failed SPI transfers have we seen?
uint16_t CPLDBusOp::spi_wait_timeout = 20; // In microseconds. Per-byte.
ManuvrMsg CPLDBusOp::event_spi_queue_ready;


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Vanilla constructor that calls wipe().
*/
CPLDBusOp::CPLDBusOp() {
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
CPLDBusOp::CPLDBusOp(BusOpcode nu_op, BusOpCallback* requester) {
  wipe();
  this->opcode = nu_op;
  callback     = requester;
}


/**
* Destructor
* Should be nothing to do here. If this is DMA'd, we expect the referenced buffer
*   to be managed by the class that creates these objects.
*
* Moreover, sometimes instances of this class will be preallocated, and never torn down.
*/
CPLDBusOp::~CPLDBusOp() {
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
void CPLDBusOp::setBuffer(uint8_t *buf, uint8_t len) {
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
void CPLDBusOp::setParams(uint8_t _dev_addr, uint8_t _xfer_len, uint8_t _dev_count, uint8_t _reg_addr) {
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
void CPLDBusOp::setParams(uint8_t _reg_addr, uint8_t _val) {
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
int8_t CPLDBusOp::init_dma() {
//  if (HAL_DMA_GetState(&_dma_w) != HAL_DMA_STATE_RESET) __HAL_DMA_DISABLE(&_dma_w);
//  if (HAL_DMA_GetState(&_dma_r) != HAL_DMA_STATE_RESET) __HAL_DMA_DISABLE(&_dma_r);
//
//  uint32_t _origin_buf = 0;
//  uint32_t _target_buf = 0;
//
//  if (opcode == BusOpcode::RX) {
//    _dma_r.Init.MemInc = DMA_MINC_DISABLE;
//    _dma_w.Init.MemInc = DMA_MINC_DISABLE;
//
//    //DMA_InitStructure_Read.DMA_Memory0BaseAddr    = (uint32_t) buf;
//
//    // We still need a transmit DMA operation to send the transfer parameters.
//    //DMA_InitStructure_Write.DMA_Memory0BaseAddr   = (uint32_t) &STATIC_ZERO;
//  }
//  else if (opcode == BusOpcode::TX) {
//    _dma_r.Init.MemInc = DMA_MINC_DISABLE;
//    _dma_w.Init.MemInc = DMA_MINC_ENABLE;
//
//    //DMA_InitStructure_Write.DMA_Memory0BaseAddr   = (uint32_t) buf;
//
//    // For now, we are reliant on the Rx DMA IRQ. Tx IRQ is never used. So when
//    // transmitting, we need to sink the read bytes until we do something smarter.
//    //DMA_InitStructure_Read.DMA_Memory0BaseAddr    = (uint32_t) &STATIC_ZERO;
//  }
//  else {
//    return -1;
//  }
//
//  while (HAL_DMA_GetState(&_dma_w) != HAL_DMA_STATE_RESET) {}  // TODO: Might-could cut this.
//  HAL_DMA_Init(&_dma_w);
//
//  while (HAL_DMA_GetState(&_dma_r) != HAL_DMA_STATE_RESET) {}  // TODO: Might-could cut this.
//  HAL_DMA_Init(&_dma_r);
//
//  if (opcode == BusOpcode::RX) {
//    HAL_DMA_Start_IT(&_dma_r, (uint32_t) hspi1.pRxBuffPtr, (uint32_t) buf, (uint32_t) buf_len);
//    HAL_DMA_Start_IT(&_dma_w, (uint32_t) buf, (uint32_t) hspi1.pTxBuffPtr, (uint32_t) buf_len);
//  }
//  else if (opcode == BusOpcode::TX) {
//    HAL_DMA_Start_IT(&_dma_w, (uint32_t) buf, (uint32_t) hspi1.pTxBuffPtr, (uint32_t) buf_len);
//  }
//
  return 0;
}


/**
* Wipes this bus operation so it can be reused.
* Be careful not to blow away the flags that prevent us from being reaped.
*/
void CPLDBusOp::wipe() {
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


/*******************************************************************************
*     8                  eeeeee
*     8  eeeee eeeee     8    e eeeee eeeee eeeee eeeee  eeeee e
*     8e 8  88 8   8     8e     8  88 8   8   8   8   8  8  88 8
*     88 8   8 8eee8e    88     8   8 8e  8   8e  8eee8e 8   8 8e
* e   88 8   8 88   8    88   e 8   8 88  8   88  88   8 8   8 88
* 8eee88 8eee8 88eee8    88eee8 8eee8 88  8   88  88   8 8eee8 88eee
*******************************************************************************/


/**
* Marks this bus operation complete.
*
* Need to remember: this gets called in the event of ANY condition that ends this job. And
*   that includes abort() where the bus operation was never begun, and SOME OTHER job has
*   control of the bus.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t CPLDBusOp::markComplete() {
  if (has_bus_control() || (CPLDDriver::current_queue_item == this) ) {
    // If this job has bus control, we need to release the bus and tidy up IRQs.
    setPin(30, false);
    if (buf_len > 1) {
      // We have DMA cruft to clean.
      enableSPI_DMA(false);
      //__HAL_DMA_DISABLE(&_dma_r);
      //__HAL_DMA_CLEAR_FLAG(&_dma_r, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6);

      //__HAL_DMA_DISABLE(&_dma_w);
      //__HAL_DMA_CLEAR_FLAG(&_dma_w, DMA_FLAG_TCIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TEIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_FEIF3_7);
    }
  }

  //time_ended = micros();
  total_transfers++;
  xfer_state = XferState::COMPLETE;
  step_queues();
  return 0;
}


/**
* This will mark the bus operation complete with a given error code.
*
* @param  cause A failure code to mark the operation with.
* @return 0 on success. Non-zero on failure.
*/
int8_t CPLDBusOp::abort(XferFault cause) {
  CPLDBusOp::failed_transfers++;
  xfer_fault = cause;
  debug_log.concatf("SPI job aborted at state %s. Cause: %s.\n", getStateString(), getErrorString());
  printDebug(&debug_log);
  return markComplete();
}


/*******************************************************************************
* Memory-management and cleanup support.                                       *
*******************************************************************************/

/**
* The client class calls this fxn to set this object's post-completion behavior.
* If this fxn is never called, the default behavior of the class is to allow itself to be free()'d.
*
* This flag is preserved by wipe().
*
* @param  nu_reap_state Pass false to cause the bus manager to leave this object alone.
* @return true if the bus manager class should free() this object. False otherwise.
*/
bool CPLDBusOp::shouldReap(bool nu_reap_state) {
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
bool CPLDBusOp::returnToPrealloc(bool nu_prealloc_state) {
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
bool CPLDBusOp::devRegisterAdvance(bool _reg_advance) {
  flags = (_reg_advance) ? (flags | SPI_XFER_FLAG_DEVICE_REG_INC) : (flags & (uint8_t) ~SPI_XFER_FLAG_DEVICE_REG_INC);
  return ((flags & SPI_XFER_FLAG_DEVICE_REG_INC) == 0);
}



/*******************************************************************************
* These functions are for logging support.                                     *
*******************************************************************************/

/**
* Debug support method. This fxn is only present in debug builds.
*
* @param  StringBuilder* The buffer into which this fxn should write its output.
*/
void CPLDBusOp::printDebug(StringBuilder *output) {
  if (NULL == output) return;
  output->concatf("-----CPLDBusOp %p (%s)------------\n", (uintptr_t) this, getOpcodeString());
  if (shouldReap())       output->concat("\t Will reap\n");
  if (returnToPrealloc()) output->concat("\t Returns to prealloc\n");
  output->concatf("\t xfer_state        %s\n\t err               %s\n", getStateString(), getErrorString());
  //if (XferState::COMPLETE == xfer_state) {
  //  output->concatf("\t completed (uS)   %u\n",   (unsigned long) time_ended - time_began);
  //}
  output->concatf("\t param_len         %d\n", _param_len);
  output->concat("\t params            ");

  if (_param_len > 0) {
    for (uint8_t i = 0; i < _param_len; i++) {
      output->concatf("0x%02x ", xfer_params[i]);
    }
  }

  output->concatf("\n\t buf_len           %d\n", buf_len);

  if (buf_len > 0) {
    output->concatf("\t buf *(%p) ", (uintptr_t) buf);
    //for (uint8_t i = 0; i < buf_len; i++) {
    for (uint8_t i = 0; i < buf_len; i++) {
      output->concatf("0x%02x ", (uint8_t) *(buf + i));
    }
  }

  output->concat("\n\n");
}
