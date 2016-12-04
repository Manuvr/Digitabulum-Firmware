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
This file contains functions for running firmware under linux.
*/


/**
* Should undo all the effects of the init functions.
*/
void CPLDDriver::_deinit() {
}


/**
* Setup GPIO pins and their bindings to on-chip peripherals, if required.
*/
void CPLDDriver::gpioSetup() {
}


/**
* Init the timer to provide the CPLD with an external clock. This clock is the
*   most-flexible, and we use it by default.
*/
bool CPLDDriver::_set_timer_base(uint16_t _period) {
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


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void CPLDDriver::printHardwareState(StringBuilder *output) {
}
