/*
File:   target_esp32.cpp
Author: J. Ian Lindsay
Date:   2017.02.04

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
This file contains functions for running firmware on the ESP32. We will be
  assuming the WROOM32 configuration.

SPI1 and SPI2 on the CPLD will be SPI2 and SPI3 on the ESP32, respectively.
For clarity...

CPLD   ESP32  ESP32 (Name)
--------------------------
SPI0    SPI2     HSPI
SPI1    SPI3     VSPI


TODO: Until something smarter is done, it is assumed that this file will be
  #include'd by pre-processor choice in CPLDDriver.cpp.
*/

#include "driver/ledc.h"



/**
* Should undo all the effects of the init functions.
*/
void CPLDDriver::_deinit() {
  ledc_stop(LEDC_HIGH_SPEED_MODE, (ledc_channel_t) LEDC_TIMER_0, 1);
}

/**
* Init the timer to provide the CPLD with an external clock. This clock is the
*   most-flexible, and we use it by default.
*/
bool CPLDDriver::_set_timer_base(uint16_t _freq) {
  return (ESP_OK == ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, _freq)) ;
}

/**
* On the ESP32, we use the LED peripheral to provide the CPLD with an external
*   clock. This clock is the most-flexible, and we use it by default.
*/
void CPLDDriver::init_ext_clk() {
  ledc_timer_config_t timer_conf = {
    speed_mode : LEDC_HIGH_SPEED_MODE, // TODO: Doc says this is the only mode supported.
    bit_num    : LEDC_TIMER_10_BIT,   // We only need a constant duty-cycle. Flip fewer bits.
    timer_num  : LEDC_TIMER_0,       // TODO: Understand implications of this choice.
    freq_hz    : 4000               // PWM frequency.
  };
  ledc_channel_config_t channel_conf = {
    gpio_num   : _pins.clk,            // The CLK output pin.
    speed_mode : LEDC_HIGH_SPEED_MODE, // TODO: Doc says this is the only mode supported.
    channel    : LEDC_CHANNEL_0,       // We use channel0 for this.
    intr_type  : LEDC_INTR_DISABLE,    // No IRQ required.
    timer_sel  : LEDC_TIMER_0,
    duty       : 511          // range is 0 ~ ((2**bit_num)-1)
  };

  if (ESP_OK == ledc_timer_config(&timer_conf)) {
    if (ESP_OK == ledc_channel_config(&channel_conf)) {
      // Success. Clock should be running.
    }
    else {
      Kernel::log("CPLDDriver::init_ext_clk(): Failed to configure channel.\n");
    }
  }
  else {
    Kernel::log("CPLDDriver::init_ext_clk(): Failed to configure timer.\n");
  }
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
    ledc_timer_resume(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
  }
  else {
    ledc_timer_pause(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
  }
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
* Init of the SPI peripheral tasked with interrupt data.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi2(uint8_t cpol, uint8_t cpha) {
}



/*******************************************************************************
* ___     _                                  This is a template class for
*  |   / / \ o    /\   _|  _. ._ _|_  _  ._  defining arbitrary I/O adapters.
* _|_ /  \_/ o   /--\ (_| (_| |_) |_ (/_ |   Adapters must be instanced with
*                             |              a BusOp as the template param.
*******************************************************************************/

int8_t CPLDDriver::bus_init() {
  init_spi(1, 0);   // CPOL=1, CPHA=0, HW-driven
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
  }
  else {
    set_state((0 == buf_len) ? XferState::RX_WAIT : XferState::ADDR);
    // We can afford to read two bytes into the same space as our xfer_params...
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
        if (opcode == BusOpcode::TX) {
          set_state(XferState::TX_WAIT);
        }
        else {
          set_state(XferState::RX_WAIT);
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
