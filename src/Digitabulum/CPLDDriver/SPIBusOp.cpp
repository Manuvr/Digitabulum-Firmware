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
extern DMA_HandleTypeDef _dma_r;
extern DMA_HandleTypeDef _dma_w;

/*
* The HAL library does not break this out, and it doesn't support double-buffer.
* Replicated definition from stm32f7xx_hal_dma.c
*/
typedef struct {
  __IO uint32_t ISR;       /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;      /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


/*******************************************************************************
* Out-of-class                                                                 +
*******************************************************************************/
StringBuilder debug_log;   // TODO: Relocate this to a static member.

/*
* Notated like a const, but should NOT be a const, because we use this as a DMA read sink as well
*   as a sort of /dev/zero. This never contains any meaningful data.
*/
static uint32_t STATIC_ZERO = 0;
static uint32_t STATIC_SINK = 0;


/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/

extern "C" {
/**
* DMA ISR. Rx
*/
void DMA2_Stream2_IRQHandler() {
  Kernel::log("DMA2_Stream2_IRQHandler()\n");
  DMA_Base_Registers* regs = (DMA_Base_Registers*)_dma_r.StreamBaseAddress;
  int streamIndex = _dma_r.StreamIndex;
  /* Transfer Error Interrupt management */
  if ((regs->ISR & (DMA_FLAG_TEIF0_4 << streamIndex)) != RESET) {
    if(__HAL_DMA_GET_IT_SOURCE(&_dma_r, DMA_IT_TE) != RESET) {
      /* Disable the transfer error interrupt */
      __HAL_DMA_DISABLE_IT(&_dma_r, DMA_IT_TE);
      /* Clear the transfer error flag */
      regs->IFCR = DMA_FLAG_TEIF0_4 << streamIndex;
      /* Update error code */
      _dma_r.ErrorCode |= HAL_DMA_ERROR_TE;
      /* Change the DMA state */
      _dma_r.State = HAL_DMA_STATE_ERROR;
      Kernel::log("DMA2_Stream2 Error (Transfer)\n");
    }
  }
  /* FIFO Error Interrupt management */
  if ((regs->ISR & (DMA_FLAG_FEIF0_4 << streamIndex)) != RESET) {
    if(__HAL_DMA_GET_IT_SOURCE(&_dma_r, DMA_IT_FE) != RESET) {
      /* Disable the FIFO Error interrupt */
      __HAL_DMA_DISABLE_IT(&_dma_r, DMA_IT_FE);
      /* Clear the FIFO error flag */
      regs->IFCR = DMA_FLAG_FEIF0_4 << streamIndex;
      /* Update error code */
      _dma_r.ErrorCode |= HAL_DMA_ERROR_FE;
      /* Change the DMA state */
      _dma_r.State = HAL_DMA_STATE_ERROR;
      Kernel::log("DMA2_Stream2 Error (FIFO)\n");
    }
  }
  /* Transfer Complete Interrupt management */
  if ((regs->ISR & (DMA_FLAG_TCIF0_4 << streamIndex)) != RESET) {
    Kernel::log("DMA2_Stream2 TC\n");
    if(__HAL_DMA_GET_IT_SOURCE(&_dma_r, DMA_IT_TC) != RESET) {
      /* Clear the transfer complete flag */
      regs->IFCR = DMA_FLAG_TCIF0_4 << streamIndex;
    }
    /* Update error code */
    _dma_r.ErrorCode |= HAL_DMA_ERROR_NONE;
    /* Change the DMA state */
    _dma_r.State = HAL_DMA_STATE_READY_MEM0;
    __HAL_DMA_DISABLE(&_dma_r);
    if (NULL != CPLDDriver::current_queue_item) {
      CPLDDriver::current_queue_item->advance_operation(0, 0);
    }
  }
}


/*
* DMA ISR. Tx
*/
void DMA2_Stream3_IRQHandler(void) {
  Kernel::log("DMA2_Stream3_IRQHandler()\n");
  __HAL_DMA_DISABLE_IT(&_dma_r, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
  __HAL_DMA_DISABLE_IT(&_dma_w, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
}


}

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
uint32_t SPIBusOp::total_transfers  = 0;  // How many total SPI transfers have we seen?
uint32_t SPIBusOp::failed_transfers = 0;  // How many failed SPI transfers have we seen?
uint16_t SPIBusOp::spi_wait_timeout = 20; // In microseconds. Per-byte.

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


/*******************************************************************************
*     8                  eeeeee
*     8  eeeee eeeee     8    e eeeee eeeee eeeee eeeee  eeeee e
*     8e 8  88 8   8     8e     8  88 8   8   8   8   8  8  88 8
*     88 8   8 8eee8e    88     8   8 8e  8   8e  8eee8e 8   8 8e
* e   88 8   8 88   8    88   e 8   8 88  8   88  88   8 8   8 88
* 8eee88 8eee8 88eee8    88eee8 8eee8 88  8   88  88   8 8eee8 88eee
*******************************************************************************/
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

  if (SPI1->SR & SPI_FLAG_BSY) {
    Kernel::log("SPI op aborted before taking bus control.\n");
    abort(XferFault::BUS_BUSY);
    return -1;
  }

  set_state(XferState::INITIATE);  // Indicate that we now have bus control.

  if ((opcode == BusOpcode::TX) || (2 < _param_len)) {
    set_state((0 == buf_len) ? XferState::TX_WAIT : XferState::ADDR);
    //__HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_TXE));
    HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) xfer_params, (uint8_t*) &STATIC_SINK, _param_len);
  }
  else {
    set_state((0 == buf_len) ? XferState::RX_WAIT : XferState::ADDR);
    // We can afford to read two bytes into the same space as our xfer_params...
    HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) xfer_params, (uint8_t*)(xfer_params + 2), 2);
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
  if (has_bus_control() || (CPLDDriver::current_queue_item == this) ) {
    // If this job has bus control, we need to release the bus and tidy up IRQs.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
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
  //debug_log.concatf("advance_op(0x%08x, 0x%02x)\n\t %s\n\t status: 0x%08x\n", status_reg, data_reg, getStateString(), (unsigned long) hspi1.State);
  //Kernel::log(&debug_log);

  /* These are our transfer-size-invariant cases. */
  switch (xfer_state) {
    case XferState::COMPLETE:
      abort(XferFault::HUNG_IRQ);
      return 0;

    case XferState::TX_WAIT:
    case XferState::RX_WAIT:
      markComplete();
      return 0;

    case XferState::FAULT:
      return 0;

    case XferState::QUEUED:
    case XferState::ADDR:
      if (buf_len > 0) {
        // We have 4 bytes to throw away from the params transfer.
        uint16_t tmpreg = hspi1.Instance->DR;
        tmpreg = hspi1.Instance->DR;
        if (opcode == BusOpcode::TX) {
          set_state(XferState::TX_WAIT);
          HAL_SPI_Transmit_IT(&hspi1, (uint8_t*) buf, buf_len);
        }
        else {
          set_state(XferState::RX_WAIT);
          HAL_SPI_Receive_DMA(&hspi1, buf, buf_len);
          //HAL_NVIC_DisableIRQ(SPI1_IRQn);
          //HAL_DMA_Init(&_dma_r);
          //_dma_r.State = HAL_DMA_STATE_BUSY;
          //__HAL_DMA_DISABLE(&_dma_r);
          //DMA2_Stream2->PAR   = (uint32_t) &hspi1.Instance->DR;
          //DMA2_Stream2->M0AR  = (uint32_t) buf;
          //DMA2_Stream2->NDTR  = buf_len;
          //DMA2_Stream2->CR   &= ~((uint32_t) (DMA_SxCR_DBM | DMA_SxCR_CT));
          //DMA2_Stream2->CR   |= (uint32_t) (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
          //DMA2_Stream2->FCR  |= (uint32_t) DMA_IT_FE;
          //__HAL_DMA_ENABLE(&_dma_r);
          //__HAL_SPI_ENABLE(&hspi1);
        }
      }
      return 0;
    case XferState::STOP:
    case XferState::UNDEF:

    /* Below are the states that we shouldn't be in at this point... */
    case XferState::INITIATE:
    case XferState::IDLE:
      abort(XferFault::ILLEGAL_STATE);
      return 0;
  }

  return -1;
}
#pragma GCC diagnostic pop



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



/*******************************************************************************
* These functions are for logging support.                                     *
*******************************************************************************/

/**
* Debug support method. This fxn is only present in debug builds.
*
* @param  StringBuilder* The buffer into which this fxn should write its output.
*/
void SPIBusOp::printDebug(StringBuilder *output) {
  if (NULL == output) return;
  output->concatf("-----SPIBusOp 0x%08x (%s)------------\n", (uint32_t) this, getOpcodeString());
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
    output->concatf("\t buf *(0x%08x) ", (uint32_t) buf);
    //for (uint8_t i = 0; i < buf_len; i++) {
    for (uint8_t i = 0; i < buf_len; i++) {
      output->concatf("0x%02x ", (uint8_t) *(buf + i));
    }
  }

  output->concatf("\n\t _dma_r State      0x%04x\n", _dma_r.State);
  output->concatf("\t _dma_r->CR        0x%08x\n", DMA2_Stream2->CR);
  output->concatf("\t _dma_r->FCR       0x%08x\n", DMA2_Stream2->FCR);
  output->concatf("\t _dma_r->NDTR      0x%08x\n", DMA2_Stream2->NDTR);

  output->concat("\n\n");
}
