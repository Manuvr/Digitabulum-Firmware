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
#include <stm32f7xx_hal_dma.h>


/**
* Used to disable the DMA IRQs at the NVIC.
*
* @param bool enable the interrupts?
*/
void BTQueuedOperation::enable_DMA_IRQ(bool enable) {
  if (!enable) {
    NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  }
  else {
    DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_FEIF6|DMA_FLAG_DMEIF6|DMA_FLAG_TEIF6|DMA_FLAG_HTIF6|DMA_FLAG_TCIF6);
    DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6 | DMA_IT_TEIF6 | DMA_IT_HTIF6 | DMA_IT_DMEIF6 | DMA_IT_FEIF6);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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


BTQueuedOperation::BTQueuedOperation() {
  wipe();
}


BTQueuedOperation::BTQueuedOperation(uint8_t nu_op) {
  wipe();
  this->opcode    = nu_op;
}


BTQueuedOperation::BTQueuedOperation(uint8_t nu_op, StringBuilder* nu_data) {
  wipe();
  this->opcode    = nu_op;
  data.concatHandoff(nu_data);
}


/*
* Specialized constructor for direct buffer spec.
*/
BTQueuedOperation::BTQueuedOperation(uint8_t nu_op, unsigned char *nu_data, uint16_t nu_len) {
  wipe();
  this->opcode    = nu_op;
  data.concat(nu_data, nu_len);
}


BTQueuedOperation::~BTQueuedOperation(void) {
  //StringBuilder log_item("Destroying completed job++++++++++++++++++++++++++++++++++\n");
  //printDebug(&log_item);
  //Kernel::log(&log_item);
}



void BTQueuedOperation::set_data(uint8_t nu_op, StringBuilder* nu_data) {
  this->opcode    = nu_op;
  data.concatHandoff(nu_data);
  data.string();
}


void BTQueuedOperation::wipe() {
  this->completed  = false;
  this->initiated  = false;
  this->xenomsg_id = 0;
  this->opcode     = RNBASE_OP_CODE_UNDEFINED;
  this->tx_buf     = NULL;
  this->tx_len     = 0;
  this->txn_id     = randomInt();
  data.clear();
}



/*
* This queue item can begin executing. This is where any bus access should be initiated.
*/
int8_t BTQueuedOperation::begin(void) {
  initiated = true;
  switch (opcode) {
    case RNBASE_OP_CODE_CMD_TX_WAIT_RX:  // Transmit and remember.
    case RNBASE_OP_CODE_CMD_TX:          // Transmit and forget.
    case RNBASE_OP_CODE_TX:              // Transmit and forget.
      tx_buf = data.string();
      tx_len = data.length();
      if (tx_len == 0) {
        StringBuilder log_item;
        printDebug(&log_item);
        Kernel::log(&log_item);
        mark_complete();
      }
      else {
        initiated = true;
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
int8_t BTQueuedOperation::abort(void) {
  completed = true;
  initiated = true;
  tx_buf = NULL;
  tx_len = 0;
  RNBase::isr_bt_queue_ready();
  return 0;
}


/* Call to mark TX complete. */
int8_t BTQueuedOperation::mark_complete(void) {
  initiated = true;
  tx_buf = NULL;
  tx_len = 0;
  enable_DMA_IRQ(false);

  switch (opcode) {
    case RNBASE_OP_CODE_CMD_TX_WAIT_RX:
      {
      // We need to be able to time out...
      EventManager::raiseEvent(MANUVR_MSG_BT_RX_BUF_NOT_EMPTY, NULL);
      }
      break;
    case RNBASE_OP_CODE_CMD_TX:
    case RNBASE_OP_CODE_TX:
      completed = true;
      RNBase::isr_bt_queue_ready();
      break;
    default:
      completed = true;
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
  output->concatf("\t  opcode:      %s\n", getOpcodeString(opcode));
  output->concatf("\t  comp/init:   %d/%d\n", completed, initiated);

  int tmp_len = data.length();
  output->concatf("\t  length:      %d\n", tmp_len);
  if (tmp_len > 0) {
    output->concatf("\t  data:        %s\n", data.string());
  }
}



DMA_InitTypeDef DMA_InitStructure;


void BTQueuedOperation::buildDMAMembers() {
  DMA_DeInit(DMA1_Stream6);

  DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;   // Transmit
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Enable;  // Required for differnt access-widths.
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART2->DR;

  /* Enable DMA Stream Transfer Complete interrupt */
  enable_DMA_IRQ(false);
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME, ENABLE);
}



/*
* This is actually the function that does the work of sending things to
*   the host. It is to be the last stop for a buffer prior to being fed
*   to USART2's DMA channel.
*
* Thank you again, clive1
* https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2Fcortex_mx_stm32%2FSTM32F4%20Discovery%20UART%20DMA%20TX%20Problem&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=380
*
* DMA_InitStructure.DMA_BufferSize         = (uint16_t) tx_len;   // Why did clive1 have (len-1)??   // I know why. clive1 made a mistake.
*/
int8_t BTQueuedOperation::init_dma() {
  // Disable the DMA Tx Stream.
  if (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) DMA_Cmd(DMA1_Stream6, DISABLE);

  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) tx_buf;
  DMA_InitStructure.DMA_BufferSize         = (uint16_t) tx_len;

  while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream6, ENABLE);    // Enable the DMA Tx Stream

  enable_DMA_IRQ(true);

  /* Enable the USART Tx DMA request */
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
  return 0;
}



/**
* Debug support. Returns a human-readable representation of an opcode.
*/
const char* BTQueuedOperation::getOpcodeString(uint8_t code) {
  switch (code) {
    case RNBASE_OP_CODE_TX:               return "TX";
    case RNBASE_OP_CODE_CMD_TX:           return "CMD_TX";
    case RNBASE_OP_CODE_CMD_TX_WAIT_RX:   return "CMD_TX/RX";
    default:                              return "<UNKNOWN>";
  }
}
