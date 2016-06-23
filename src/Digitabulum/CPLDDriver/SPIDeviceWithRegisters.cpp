/*
File:   SPIDeviceWithRegisters.cpp
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


#include "SPIDeviceWithRegisters.h"
#include "CPLDDriver.h"

extern volatile CPLDDriver* cpld;


SPIDeviceWithRegisters::SPIDeviceWithRegisters(uint8_t bus_address, uint8_t r_count) : DeviceWithRegisters(r_count) {
  reg_defs = (reg_count > 0) ? (DeviceRegister*) malloc(sizeof(DeviceRegister) * reg_count) : NULL;
  bus_addr = bus_address;
}

SPIDeviceWithRegisters::~SPIDeviceWithRegisters() {
  if (NULL != reg_defs) free(reg_defs);
  reg_defs = NULL;
  // TODO: We should purge the bus of any of our operations and mask our IRQs prior to doing this.
}



/*
* Ultimately, all bus access this class does passes to this function as its last-stop
*   before becoming folded into the SPI bus queue.
*/
int8_t SPIDeviceWithRegisters::queue_io_job(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  if (NULL == op) return -1;   // This should never happen.
  op->callback = this;         // Notify us of the results.
  return ((CPLDDriver*)cpld)->queue_io_job(op);     // Pass it to the CPLD for bus access.
}



int8_t SPIDeviceWithRegisters::writeRegister(DeviceRegister *reg) {
  if (reg == NULL) return -1;

  SPIBusOp* op = ((CPLDDriver*)cpld)->issue_spi_op_obj();
  op->set_opcode(BusOpcode::TX);
  op->buf        = reg->val;
  op->buf_len    = reg->len;
  op->setParams(bus_addr, reg->len, 1, reg->addr);

  reg->dirty = true;
  return SPIDeviceWithRegisters::queue_io_job(op);
}


int8_t SPIDeviceWithRegisters::writeRegister(uint8_t idx, unsigned int nu_val, bool defer) {
  if (idx >= reg_count) return -1;

  DeviceRegister *reg = &reg_defs[idx];
  switch (reg->len) {
    case 1:
      *((uint8_t *) reg->val) = (uint8_t) (0xFF & nu_val);
      reg->dirty = true;
      break;
    case 2:
      *((uint16_t *) reg->val) = (uint16_t) (0xFFFF & nu_val);
      reg->dirty = true;
      break;
    case 4:
      *((uint32_t *) reg->val) = (uint32_t) (0xFFFFFFFF & nu_val);
      reg->dirty = true;
      break;
    default:
      Kernel::log("SPIDeviecWithRegisters::writeRegisters(): Invalid register size.\n");
      return -1;
  }

  SPIBusOp* op;
  if (!defer) {
    op = ((CPLDDriver*)cpld)->issue_spi_op_obj();
    op->set_opcode(BusOpcode::TX);
    op->buf        = reg->val;
    op->buf_len    = reg->len;
    op->setParams(bus_addr, reg->len, 1, reg->addr);

    return SPIDeviceWithRegisters::queue_io_job(op);
  }

  return 0;
}


int8_t SPIDeviceWithRegisters::writeRegister(uint8_t idx, uint8_t nu_val) {
  return writeRegister(idx, nu_val, false);
}


int8_t SPIDeviceWithRegisters::readRegister(uint8_t idx) {
  if (idx >= reg_count) return -1;
  return readRegister(&reg_defs[idx]);
}


int8_t SPIDeviceWithRegisters::readRegister(DeviceRegister *reg) {
  if (reg == NULL) return -1;

  SPIBusOp* op = ((CPLDDriver*)cpld)->issue_spi_op_obj();
  op->set_opcode(BusOpcode::RX);
  op->buf        = reg->val;
  op->buf_len    = reg->len;
  op->setParams(bus_addr, reg->len, 1, reg->addr);

  reg->unread = true;
  reg->dirty = false;   // TODO: Reading a register will cancel a write operation.
  return SPIDeviceWithRegisters::queue_io_job(op);
}



int8_t SPIDeviceWithRegisters::readAll() {
  for (int i = 0; i < reg_count; i++) {
    readRegister(i);
  }
  return 0;
}
