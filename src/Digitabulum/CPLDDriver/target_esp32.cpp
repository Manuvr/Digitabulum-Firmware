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


#ifdef __cplusplus
extern "C" {
#endif

#include "../../Targets/ESP32/spi_common.h"


static void IRAM_ATTR spi3_isr(void *arg) {
  if (SPI3.slave.trans_done) {
    uint32_t words[3] = {SPI3.data_buf[0], SPI3.data_buf[1], SPI3.data_buf[2]};
    uint8_t* previous_buf;
    if (_irq_data_ptr == _irq_data_0) {
      _irq_data_ptr = (uint8_t*) _irq_data_1;
      previous_buf  = (uint8_t*) _irq_data_0;
    }
    else {
      _irq_data_ptr = (uint8_t*) _irq_data_0;
      previous_buf  = (uint8_t*) _irq_data_1;
    }
    memcpy((void*)_irq_data_ptr, &words[0], 10);
    for (int i = 0; i < 10; i++) {
      _irq_diff[i]   = previous_buf[i] ^ _irq_data_ptr[i];
      _irq_accum[i] |= _irq_diff[i];
    }

    Kernel::isrRaiseEvent(&_irq_data_arrival);   // TODO: Audit for mem layout.
    SPI3.slave.trans_done  = 0;  // Clear interrupt.
    return;
  }
  SPI3.slave.rd_sta_inten  = 0;  // Mask interrupt.
  SPI3.slave.wr_sta_inten  = 0;  // Mask interrupt.
  SPI3.slave.rd_buf_inten  = 0;  // Mask interrupt.
  SPI3.slave.wr_buf_inten  = 0;  // Mask interrupt.
}

#ifdef __cplusplus
}
#endif


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
* Init of SPI2. This is broken out because we might be bringing it
*   up and down in a single runtime for debug reasons.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi(uint8_t cpol, uint8_t cpha) {
  //SPI2
  for (int i = 0; i < 16; i++) SPI2.data_buf[i] = 0;
  _er_set_flag(CPLD_FLAG_SPI1_READY, false);
}


/**
* Init of SPI3. Tasked with interrupt data.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi2(uint8_t cpol, uint8_t cpha) {
  gpio_num_t p_cs    = (gpio_num_t) _pins.s2_cs;
  gpio_num_t p_clk   = (gpio_num_t) _pins.s2_clk;
  gpio_num_t p_mosi  = (gpio_num_t) _pins.s2_mosi;

  for (int i = 0; i < 16; i++) SPI3.data_buf[i] = 0;

  if (GPIO_IS_VALID_GPIO(p_cs) && GPIO_IS_VALID_GPIO(p_clk) && GPIO_IS_VALID_GPIO(p_mosi)) {
    SPI3.ctrl.fastrd_mode   = 0;  // No need of multi-lane SPI.

    SPI3.slave.slave_mode   = 1;
    SPI3.slave.wr_rd_buf_en = 1;

    SPI3.user.doutdin       = 1;  // Full-duplex. Needed to avoid command/status interpretation.
    SPI3.user.usr_miso_highpart = 1;  // The (non-existent) TX buffer should use W8-15.

    SPI3.user.usr_mosi      = 1;  // TODO: One of these can go.
    SPI3.user.usr_miso      = 1;  // TODO: One of these can go.
    SPI3.user.usr_command   = 0;  // Peripheral should not interpret content.

    SPI3.pin.ck_idle_edge       = cpol ? 1 : 0;  // CPOL
    SPI3.user.ck_i_edge         = (cpol ^ cpha) ? 1 : 0;
    SPI3.ctrl2.mosi_delay_mode  = (cpol ^ cpha) ? 1 : 2;

    SPI3.ctrl2.miso_delay_mode  = 0;  //
    SPI3.ctrl2.miso_delay_num   = 0;  //
    SPI3.ctrl2.mosi_delay_num   = 0;  //

    //SPI3.pin.ck_dis         = 1;  // We have no need of a clock output.

    SPI3.slave.trans_done   = 0;  // Interrupt conditions.
    SPI3.slave.trans_inten  = 1;  //

    SPI3.slv_wrbuf_dlen.bit_len   = 79;
    SPI3.slv_rdbuf_dlen.bit_len   = 79;
    SPI3.slv_rd_bit.slv_rdata_bit = 79;

    SPI3.slave.cs_i_mode   = 2;  // Double-buffered CS signal.
    SPI3.slave.sync_reset  = 1;  // Reset the pins(?)


    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_cs],   PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_clk],  PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_mosi], PIN_FUNC_GPIO);

    gpio_set_direction(p_cs,   GPIO_MODE_INPUT);
    gpio_set_direction(p_clk,  GPIO_MODE_INPUT);
    gpio_set_direction(p_mosi, GPIO_MODE_INPUT);

    gpio_matrix_in( p_cs,   VSPICS0_IN_IDX,  false);
    gpio_matrix_in( p_clk,  VSPICLK_IN_IDX,  false);
    //gpio_matrix_in( p_mosi, VSPIQ_IN_IDX,    false);
    gpio_matrix_in( p_mosi, VSPID_IN_IDX,    false);
  }
  periph_module_enable(PERIPH_VSPI_MODULE);
  esp_intr_alloc(ETS_SPI3_INTR_SOURCE, ESP_INTR_FLAG_IRAM, spi3_isr, nullptr, nullptr);
  _er_set_flag(CPLD_FLAG_SPI2_READY, true);
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
  output->concatf("-- SPI2 (%sline) --------------------\n", (_er_flag(CPLD_FLAG_SPI1_READY)?"on":"OFF"));
  for (int i = 0; i < 16; i+=4) {
    output->concatf(
      "--\t SPI2.data_buf[%2d-%2d]:   0x%08x  0x%08x  0x%08x  0x%08x\n",
      i, i+3,
      (uint32_t) SPI2.data_buf[i + 0],
      (uint32_t) SPI2.data_buf[i + 1],
      (uint32_t) SPI2.data_buf[i + 2],
      (uint32_t) SPI2.data_buf[i + 3]
    );
  }

  output->concatf("\n-- SPI3 (%sline) --------------------\n", (_er_flag(CPLD_FLAG_SPI2_READY)?"on":"OFF"));
  output->concatf("--\t Ops:         0x%08x\n", SPI3.slave.trans_cnt);
  output->concatf("--\t Last State:  0x%02x\n", (uint8_t) SPI3.slave.last_state);
  output->concatf("--\t Last CMD:    0x%02x\n", (uint8_t) SPI3.slave.last_command);
  output->concatf("--\t Ext2.State:  0x%02x\n", (uint8_t) SPI3.ext2.st);
  output->concatf("--\t mosi_dlen:   0x%08x\n", SPI3.mosi_dlen.val);
  output->concatf("--\t miso_dlen:   0x%08x\n", SPI3.miso_dlen.val);
  output->concatf("--\t user:        0x%08x\n", SPI3.user.val);
  output->concatf("--\t user1:       0x%08x\n", SPI3.user1.val);
  output->concatf("--\t user2:       0x%08x\n", SPI3.user2.val);
  output->concatf("--\t pin:         0x%08x\n", SPI3.pin.val);
  output->concatf("--\t slave:       0x%08x\n", SPI3.slave.val);
  output->concatf("--\t slave1:      0x%08x\n", SPI3.slave1.val);
  output->concatf("--\t slave2:      0x%08x\n", SPI3.slave2.val);
  output->concatf("--\t slave3:      0x%08x\n", SPI3.slave3.val);

  output->concatf("--\t slv_wrbuf_dlen.bit_len    0x%06x\n", (uint32_t) SPI3.slv_wrbuf_dlen.bit_len  );
  output->concatf("--\t slv_rdbuf_dlen.bit_len    0x%06x\n", (uint32_t) SPI3.slv_rdbuf_dlen.bit_len  );
  output->concatf("--\t slv_rd_bit.slv_rdata_bit  0x%06x\n", (uint32_t) SPI3.slv_rd_bit.slv_rdata_bit);
  for (int i = 0; i < 16; i+=4) {
    output->concatf(
      "--\t SPI3.data_buf[%2d-%2d]:   0x%08x  0x%08x  0x%08x  0x%08x\n",
      i, i+3,
      (uint32_t) SPI3.data_buf[i + 0],
      (uint32_t) SPI3.data_buf[i + 1],
      (uint32_t) SPI3.data_buf[i + 2],
      (uint32_t) SPI3.data_buf[i + 3]
    );
  }
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
