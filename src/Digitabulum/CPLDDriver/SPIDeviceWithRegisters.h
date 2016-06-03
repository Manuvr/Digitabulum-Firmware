/*
File:   SPIDeviceWithRegisters.h
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

#ifndef __OFFBOARD_REGISTER_SPI_DEV_H__
  #define __OFFBOARD_REGISTER_SPI_DEV_H__ 1

  #include "Drivers/DeviceWithRegisters/DeviceWithRegisters.h"
  #include "Drivers/DeviceWithRegisters/DeviceRegister.h"
  #include "SPIOpCallback.h"

  /*
  * This class is a representation of a generic offboard device that contains
  *   its own registers.
  */
  class SPIDeviceWithRegisters : public DeviceWithRegisters, public SPIOpCallback {
    public:
      uint8_t     bus_addr;            // What is our address on the bus?

      SPIDeviceWithRegisters(uint8_t bus_address, uint8_t r_count);
      ~SPIDeviceWithRegisters();

      /* Overrides from the SPICallback interface */
      virtual int8_t spi_op_callback(SPIBusOp*) = 0;
      int8_t queue_spi_job(SPIBusOp*);         // Implemented here.


    protected:
      /* We are only strictly-required to provide these... */
      int8_t writeRegister(DeviceRegister *reg);
      int8_t readRegister(DeviceRegister *reg);

      virtual int8_t writeRegister(uint8_t idx, uint8_t nu_val);
      virtual int8_t writeRegister(uint8_t index, unsigned int nu_value, bool defer);
      virtual int8_t readRegister(uint8_t index);

      /* Low-level bus-optimization functions. Use these to group register i/o into as few transactions
           as possible.*/
      // TODO: Need a variadic.
      int8_t readAll();

  };


#endif
