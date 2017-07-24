/*
File:   target_linux.cpp
Author: J. Ian Lindsay
Date:   2016.12.04

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


These are platform-specific functions for dealing with Digitabulum's sensor
  package.
This file contains functions for running firmware under linux. This should only
  be construed as being for firmware emulation's sake. Ports to linux-based SBCs
  should make a board-specific file that mirrors this one, rather than torturing
  the pre-processor in this file.

TODO: Until something smarter is done, it is assumed that this file will be
  #include'd by pre-processor choice in CPLDDriver.cpp.
*/


/**
* Should undo all the effects of the init functions.
*/
void CPLDDriver::_deinit() {
}

/**
* Init the timer to provide the CPLD with an external clock. This clock is the
*   most-flexible, and we use it by default.
*/
bool CPLDDriver::_set_timer_base(int hz) {
  return true;
}

/**
* Init the timer to provide the CPLD with an external clock. This clock is the
*   most-flexible, and we use it by default.
*/
void CPLDDriver::init_ext_clk() {
}

/**
* Init of SPI peripheral 1. This is broken out because we might be bringing it
*   up and down in a single runtime for debug reasons.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi(uint8_t cpol, uint8_t cpha) {
}


/**
* Init of SPI peripheral 1. This is broken out because we might be bringing it
*   up and down in a single runtime for debug reasons.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi2(uint8_t cpol, uint8_t cpha) {
}


/**
* Pass 'true' to enable the CPLD osciallator. False to disable it.
* This oscillator being enabled is a precondition for various other features
*   in the CPLD, so we keep track of the state in this class.
*
* @param  on  Should the osciallator be enabled?
*/
void CPLDDriver::externalOscillator(bool on) {
  _er_set_flag(CPLD_FLAG_EXT_OSC, on);
  if (on) {
  }
  else {
  }
}



/*******************************************************************************
* ___     _                                  This is a template class for
*  |   / / \ o    /\   _|  _. ._ _|_  _  ._  defining arbitrary I/O adapters.
* _|_ /  \_/ o   /--\ (_| (_| |_) |_ (/_ |   Adapters must be instanced with
*                             |              a BusOp as the template param.
*******************************************************************************/

int8_t CPLDDriver::bus_init() {
  return 0;
}

int8_t CPLDDriver::bus_deinit() {
  return 0;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void CPLDDriver::printHardwareState(StringBuilder *output) {
}



/*******************************************************************************
* ___     _                              These members are mandatory overrides
*  |   / / \ o     |  _  |_              from the BusOp class.
* _|_ /  \_/ o   \_| (_) |_)
*******************************************************************************/

/**
* Calling this member will cause the bus operation to be started.
*
* @return 0 on success, or non-zero on failure.
*/
XferFault SPIBusOp::begin() {
  //time_began    = micros();
  if (0 == _param_len) {
    // Obvious invalidity. We must have at least one transfer parameter.
    abort(XferFault::BAD_PARAM);
    return XferFault::BAD_PARAM;
  }

  set_state(XferState::INITIATE);  // Indicate that we now have bus control.

  if ((opcode == BusOpcode::TX) || (2 < _param_len)) {
    set_state((0 == buf_len) ? XferState::TX_WAIT : XferState::ADDR);
    //__HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_TXE));
    //HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) xfer_params, (uint8_t*) &STATIC_SINK, _param_len);
  }
  else {
    set_state((0 == buf_len) ? XferState::RX_WAIT : XferState::ADDR);
    // We can afford to read two bytes into the same space as our xfer_params...
    //HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) xfer_params, (uint8_t*)(xfer_params + 2), 2);
  }

  return XferFault::NONE;
}


/**
* Called from the ISR to advance this operation on the bus.
* Stay brief. We are in an ISR.
*
* @return 0 on success. Non-zero on failure.
*/
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
        //uint16_t tmpreg = 0;
        if (opcode == BusOpcode::TX) {
          set_state(XferState::TX_WAIT);
          //HAL_SPI_Transmit_IT(&hspi1, (uint8_t*) buf, buf_len);
        }
        else {
          set_state(XferState::RX_WAIT);
          //HAL_SPI_Receive_DMA(&hspi1, buf, buf_len);
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
