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


/***************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
***************************************************************************************/


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
  xfer_state = XferState::UNDEF;
  xfer_fault = XferFault::NONE;
  opcode     = BusOpcode::UNDEF;
  buf        = nullptr;
  buf_len    = 0;
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
XferFault BTQueuedOperation::begin() {
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
        //init_dma();
      }
      break;
    default:
      {
        StringBuilder log_item(__PRETTY_FUNCTION__);
        Kernel::log(&log_item);
      }
      break;
  }
  return xfer_fault;
}


/* Call to mark something completed that may not be. */
int8_t BTQueuedOperation::abort(XferFault cause) {
  xfer_state = XferState::FAULT;
  xfer_fault = cause;
  buf       = nullptr;
  buf_len   = 0;
  return 0;
}


/* Call to mark TX complete. */
int8_t BTQueuedOperation::markComplete() {
  buf       = nullptr;
  buf_len   = 0;
  //enable_DMA_IRQ(false);
  //HAL_DMA_Abort(&_dma_handle);
  data.clear();   // Clear the data we just sent.
  xfer_state = XferState::COMPLETE;
  RNBase::isr_bt_queue_ready();
  return 0;
}



/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void BTQueuedOperation::printDebug(StringBuilder *output) {
  BusOp::printBusOp("BTOp", this, output);

  int tmp_len = data.length();
  output->concatf("\t length:      %d\n", tmp_len);
  if (tmp_len > 0) {
    output->concatf("\t data:        %s\n", data.string());
  }
}
