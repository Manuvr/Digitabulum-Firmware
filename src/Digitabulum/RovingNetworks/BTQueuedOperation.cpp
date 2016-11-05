/*
File:   BTQueuedOperation.cpp
Author: J. Ian Lindsay
Date:   2014.05.22

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


TODO: It would be really nice to unify this basic "work-queue" pattern and
        use it elsewhere. I'm pretty sick of writing and maintaining the same
        code in 8 places...
*/


#include "RNBase.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_dma.h>

DMA_HandleTypeDef _dma_handle;

extern UART_HandleTypeDef huart2;


/**
* Used to disable the DMA IRQs at the NVIC.
*
* @param bool enable the interrupts?
*/
void BTQueuedOperation::enable_DMA_IRQ(bool enable) {
  if (!enable) {
    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  }
  else {
    // Clear all DMA flags.
    __HAL_DMA_CLEAR_FLAG(&_dma_handle, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6);
    // Clear all possible pending interrupts.
    //DMA_ClearITPendingBit(DMA1_Stream6, ());

    // Allow the DMA module IRQ in the interrupt controller.
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  }
}

//{
//  HAL_OK       = 0x00,
//  HAL_ERROR    = 0x01,
//  HAL_BUSY     = 0x02,
//  HAL_TIMEOUT  = 0x03
//} HAL_StatusTypeDef;



/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/


BTQueuedOperation::BTQueuedOperation() {
  wipe();
}


BTQueuedOperation::BTQueuedOperation(BusOpcode nu_op) : BTQueuedOperation() {
  opcode    = nu_op;
}


BTQueuedOperation::BTQueuedOperation(BusOpcode nu_op, StringBuilder* nu_data) : BTQueuedOperation(nu_op) {
  data.concatHandoff(nu_data);
}


/*
* Specialized constructor for direct buffer spec.
*/
BTQueuedOperation::BTQueuedOperation(BusOpcode nu_op, unsigned char *nu_data, uint16_t nu_len) : BTQueuedOperation(nu_op) {
  data.concat(nu_data, nu_len);
}


BTQueuedOperation::~BTQueuedOperation() {
  //StringBuilder log_item("Destroying completed job++++++++++++++++++++++++++++++++++\n");
  //printDebug(&log_item);
  //Kernel::log(&log_item);
}


void BTQueuedOperation::wipe() {
  //completed  = false;
  xfer_state = XferState::UNDEF;
  xfer_fault = XferFault::NONE;
  opcode     = BusOpcode::UNDEF;
  buf        = NULL;
  buf_len    = 0;
  txn_id     = BusOp::next_txn_id++;
  data.clear();
}


void BTQueuedOperation::set_data(BusOpcode nu_op, StringBuilder* nu_data) {
  opcode = nu_op;
  data.concatHandoff(nu_data);
  data.string();
}


/*
* This queue item can begin executing. This is where any bus access should be initiated.
*/
int8_t BTQueuedOperation::begin() {
  xfer_state = XferState::INITIATE;
  switch (opcode) {
    case BusOpcode::TX_CMD_WAIT_RX:  // Transmit and remember.
    case BusOpcode::TX_WAIT_RX:      // Transmit and remember.
    case BusOpcode::TX_CMD:          // Transmit and forget.
    case BusOpcode::TX:              // Transmit and forget.
      buf     = data.string();
      buf_len = data.length();
      if (buf_len == 0) {
        markComplete();
        StringBuilder log_item;
        xfer_state = XferState::FAULT;
        xfer_fault = XferFault::BAD_PARAM;
        printDebug(&log_item);
        Kernel::log(&log_item);
      }
      else {
        xfer_state = XferState::INITIATE;
        init_dma();
      }
      break;
    default:
      {
        StringBuilder log_item(__PRETTY_FUNCTION__);
        Kernel::log(&log_item);
      }
      break;
  }
  return 0;
}


/* Call to mark something completed that may not be. */
int8_t BTQueuedOperation::abort() {
  xfer_state = XferState::COMPLETE;
  buf       = NULL;
  buf_len   = 0;
  RNBase::isr_bt_queue_ready();
  return 0;
}


/* Call to mark TX complete. */
int8_t BTQueuedOperation::markComplete() {
  buf       = NULL;
  buf_len   = 0;
  enable_DMA_IRQ(false);
  HAL_DMA_Abort(&_dma_handle);

  switch (opcode) {
    case BusOpcode::TX_CMD_WAIT_RX:
      {
      // We need to be able to time out...
      //Kernel::raiseEvent(MANUVR_MSG_BT_RX_BUF_NOT_EMPTY, NULL);
      }
      break;
    case BusOpcode::TX_CMD:
    case BusOpcode::TX:
      xfer_state = XferState::COMPLETE;
      RNBase::isr_bt_queue_ready();
      break;
    default:
      xfer_state = XferState::COMPLETE;
      break;
  }
  data.clear();   // Clear the data we just sent.
  return 0;
}



/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void BTQueuedOperation::printDebug(StringBuilder *output) {
  if (NULL == output) return;
  output->concatf("\n\t--- txn_id:  0x%08x -------------\n", txn_id);
  output->concatf("\t opcode:      %s\n", BusOp::getOpcodeString(opcode));
  output->concatf("\t comp/init:   %s\n", BusOp::getStateString(xfer_state));

  int tmp_len = data.length();
  output->concatf("\t length:      %d\n", tmp_len);
  if (tmp_len > 0) {
    output->concatf("\t data:        %s\n", data.string());
  }
}



void BTQueuedOperation::buildDMAMembers() {
  HAL_DMA_DeInit(&_dma_handle);

  _dma_handle.Instance                 = DMA1_Stream6;
  _dma_handle.Init.Channel             = DMA_CHANNEL_4;
  _dma_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;   // Transmit
  _dma_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  _dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
  _dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  _dma_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  _dma_handle.Init.Mode                = DMA_NORMAL;
  _dma_handle.Init.Priority            = DMA_PRIORITY_LOW;
  _dma_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;  // Required for differnt access-widths.
  _dma_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  _dma_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
  _dma_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;

  /* Enable DMA Stream Transfer Complete interrupt */
  enable_DMA_IRQ(false);
  //__HAL_DMA_ENABLE_IT(&_dma_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
}



/*
* This is actually the function that does the work of sending things to
*   the host. It is to be the last stop for a buffer prior to being fed
*   to USART2's DMA channel.
*
* Thank you again, clive1
* https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2Fcortex_mx_stm32%2FSTM32F4%20Discovery%20UART%20DMA%20TX%20Problem&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=380
*
* DMA_InitStructure.DMA_BufferSize         = (uint16_t) buf_len;   // Why did clive1 have (len-1)??   // I know why. clive1 made a mistake.
*/
int8_t BTQueuedOperation::init_dma() {
//  // Disable the DMA Tx Stream.
//  if (HAL_DMA_GetState(&_dma_handle) != HAL_DMA_STATE_RESET) __HAL_DMA_DISABLE(&_dma_handle);
//
//  HAL_DMA_Init(&_dma_handle);
//  HAL_DMA_Start_IT(&_dma_handle, (uint32_t) buf, (uint32_t) huart2.pTxBuffPtr, (uint32_t) buf_len);
//
//  enable_DMA_IRQ(true);
//
//  /* Enable the USART Tx DMA request */
//  //USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
  return 0;
}
