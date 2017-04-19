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

//#include "../../Targets/ESP32/spi_common.h"
#include "soc/spi_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "soc/spi_struct.h"
#include "soc/rtc_cntl_reg.h"

#include "rom/ets_sys.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "rom/lldesc.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_heap_alloc_caps.h"

// The default frequency for the external clock, if it isn't otherwise supplied.
#define DEFAULT_CPLD_FREQ 70000


/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/

static uint32_t spi2_byte_sink    = 0;
static uint16_t spi2_op_counter_0 = 0;
static uint16_t spi2_op_counter_1 = 0;

static lldesc_t _ll_tx_1;
static lldesc_t _ll_rx_1;

// This will always be the address phase of the CPLD operation.
static lldesc_t _ll_tx_0;

// This will always be the LL for the first four throw-away bytes.
static lldesc_t _ll_rx_0;



static SPIBusOp* _threaded_op = nullptr;

void reset_spi2_dma() {
  SPI2.dma_conf.val |= (SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST);
  SPI2.dma_out_link.start = 0;
  SPI2.dma_out_link.addr  = 0;
  SPI2.dma_in_link.start  = 0;
  SPI2.dma_in_link.addr   = 0;
  SPI2.dma_conf.val &= ~(SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST);
}


static void IRAM_ATTR dma_isr(void *arg) {
  spi2_op_counter_1++;
  if (SPI2.dma_int_st.inlink_dscr_empty) {   /* lack of enough inlink descriptors.*/
    //Kernel::log("dma_isr(): inlink_dscr_empty\n");
  }
  if (SPI2.dma_int_st.outlink_dscr_error) {  /* outlink descriptor error.*/
    //Kernel::log("dma_isr(): outlink_dscr_error\n");
  }
  if (SPI2.dma_int_st.inlink_dscr_error) {   /* inlink descriptor error.*/
    //Kernel::log("dma_isr(): inlink_dscr_error\n");
  }
  if (SPI2.dma_int_st.in_done) {             /* completing usage of a inlink descriptor.*/
    //Kernel::log("dma_isr(): in_done\n");
  }
  if (SPI2.dma_int_st.in_err_eof) {          /* receiving error.*/
    //Kernel::log("dma_isr(): in_err_eof\n");
  }
  if (SPI2.dma_int_st.in_suc_eof) {          /* completing receiving all the packets from host.*/
    //Kernel::log("dma_isr(): in_suc_eof\n");
  }
  if (SPI2.dma_int_st.out_done) {            /* completing usage of a outlink descriptor .*/
    //Kernel::log("dma_isr(): out_done\n");
  }
  if (SPI2.dma_int_st.out_eof) {             /* sending a packet to host done.*/
    //Kernel::log("dma_isr(): out_eof\n");
  }
  if (SPI2.dma_int_st.out_total_eof) {       /* sending all the packets to host done.*/
    //Kernel::log("dma_isr(): out_total_eof\n");
  }
  SPI2.dma_int_clr.val = 0x0000001F;  // Blindly clear all interrupts.
}


static void IRAM_ATTR spi2_isr(void *arg) {
  if (SPI2.slave.trans_done) {
    spi2_op_counter_0++;
    if (_threaded_op) {
      if (2 == _threaded_op->transferParamLength()) {
        // Internal CPLD register access doesn't use DMA, and expects us to
        // shuffle bytes around to avoid heaped buffers and DMA overhead.
        uint32_t word = SPI2.data_buf[0];
        // These should be xfer_param[2] and xfer_param[3].
        *(_threaded_op->buf+0) = (uint8_t) word & 0xFF;
        *(_threaded_op->buf+1) = (uint8_t) ((word & 0xFF00) >> 8);
      }
      else {
        // This was a DMA transaction. That ISR should deal with data movement.
      }
      SPIBusOp* tmp = _threaded_op;  // Concurrency "safety".
      _threaded_op = nullptr;
      tmp->markComplete();
    }
    SPI2.slave.trans_done  = 0;  // Clear interrupt.
    SPI2.dma_int_clr.val = 0x0000001F;  // Blindly clear all DMA interrupts.
    //SPI3.cmd.usr = 1;  // Start the transfer afresh.
  }

  if (SPI2.slave.rd_sta_inten) {
    SPI2.slave.rd_sta_inten  = 0;  // Clear interrupt.
  }
  if (SPI2.slave.wr_sta_inten) {
    SPI2.slave.wr_sta_inten  = 0;  // Clear interrupt.
  }
  if (SPI2.slave.rd_buf_inten) {
    SPI2.slave.rd_buf_inten  = 0;  // Clear interrupt.
  }
  if (SPI2.slave.wr_buf_inten) {
    SPI2.slave.wr_buf_inten  = 0;  // Clear interrupt.
  }
}


static void IRAM_ATTR spi3_isr(void *arg) {
  if (SPI3.slave.trans_done) {
    uint8_t* hw_buf;
    uint8_t* prior_buf;

    // TODO: Would be better to use the hardware's idea of buffer.
    if (_irq_data_ptr == _irq_data_0) {  // Double-buffer "Tock"
      prior_buf     = (uint8_t*) _irq_data_0;
      _irq_data_ptr = (uint8_t*) _irq_data_1;
    }
    else {  // Double-buffer "Tick"
      prior_buf     = (uint8_t*) _irq_data_1;
      _irq_data_ptr = (uint8_t*) _irq_data_0;
    }
    hw_buf        = (uint8_t*) &SPI3.data_buf[0];

    // TODO: Code below will be better.
    //if (SPI3.user.usr_mosi_highpart) {  // Double-buffer "Tock"
    //  hw_buf        = (uint8_t*) &SPI3.data_buf[8];
    //  prior_buf     = (uint8_t*) _irq_data_0;
    //  _irq_data_ptr = (uint8_t*) _irq_data_1;
    //  SPI3.user.usr_mosi_highpart = 0;
    //}
    //else {  // Double-buffer "Tick"
    //  hw_buf        = (uint8_t*) &SPI3.data_buf[0];
    //  prior_buf     = (uint8_t*) _irq_data_1;
    //  _irq_data_ptr = (uint8_t*) _irq_data_0;
    //  SPI3.user.usr_mosi_highpart = 1;
    //}

    memcpy((void*) _irq_data_ptr, (void*) hw_buf, 10);
    for (int i = 0; i < 10; i++) {
      _irq_diff[i]   = prior_buf[i] ^ _irq_data_ptr[i];
      _irq_accum[i] |= _irq_diff[i];
    }

    Kernel::isrRaiseEvent(&_irq_data_arrival);   // TODO: Audit for mem layout.
    SPI3.slave.trans_done  = 0;  // Clear interrupt.
    SPI3.cmd.usr = 1;  // Start the transfer afresh.
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
* TODO: Might not use the LED driver for this. Can't drive this clock to 40MHz.
*/
bool CPLDDriver::_set_timer_base(int hz) {
  if (ESP_OK == ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, hz)) {
    _ext_clk_freq = hz;
    return true;
  }
  return false;
}


/**
* On the ESP32, we use the LED peripheral to provide the CPLD with an external
*   clock. This clock is the most-flexible, and we use it by default.
*/
void CPLDDriver::init_ext_clk() {
  _er_set_flag(CPLD_FLAG_EXT_OSC, false);
  ledc_timer_config_t timer_conf = {
    speed_mode : LEDC_HIGH_SPEED_MODE, // TODO: Doc says this is the only mode supported.
    bit_num    : LEDC_TIMER_10_BIT,   // We only need a constant duty-cycle. Flip fewer bits.
    timer_num  : LEDC_TIMER_0,       // TODO: Understand implications of this choice.
    freq_hz    : DEFAULT_CPLD_FREQ  // PWM frequency.
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
      _ext_clk_freq = DEFAULT_CPLD_FREQ;
      _er_set_flag(CPLD_FLAG_EXT_OSC, true);
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
  esp_err_t ret = (on) ?
    ledc_timer_resume(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0) :
    ledc_timer_pause(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);

  if (ESP_OK == ret) _er_set_flag(CPLD_FLAG_EXT_OSC, on);
}



/**
* Init of SPI2. This is broken out because we might be bringing it
*   up and down in a single runtime for debug reasons.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi(uint8_t cpol, uint8_t cpha) {
  gpio_num_t p_cs    = (gpio_num_t) _pins.s1_cs;
  gpio_num_t p_clk   = (gpio_num_t) _pins.s1_clk;
  gpio_num_t p_mosi  = (gpio_num_t) _pins.s1_mosi;
  gpio_num_t p_miso  = (gpio_num_t) _pins.s1_miso;

  for (int i = 0; i < 16; i++) SPI2.data_buf[i] = 0;
  _er_set_flag(CPLD_FLAG_SPI1_READY, false);
  if (GPIO_IS_VALID_GPIO(p_cs) && GPIO_IS_VALID_GPIO(p_clk) && GPIO_IS_VALID_GPIO(p_mosi) && GPIO_IS_VALID_OUTPUT_GPIO(p_miso)) {
    SPI2.slave.slave_mode   = 1;  // CPLD drives the clock.
    SPI2.slave.wr_rd_buf_en = 1;  // We don't want any but buffer commands.
    SPI2.slave.cs_i_mode    = 2;  // Double-buffered CS signal.
    SPI2.slave.sync_reset   = 1;  // Reset the pins.
    //SPI2.slave.cmd_define   = 1;  // Use the custom slave command mode.
    SPI2.user.usr_miso_highpart = 1;  // The TX buffer should use W8-15.

    SPI2.user.doutdin       = 1;  // Full-duplex. Needed to avoid command/status interpretation.
    //SPI2.user.usr_miso_highpart = 1;  // The TX buffer should use W8-15.
    //SPI2.user.usr_mosi_highpart = 0;  // The RX buffer should use W0-7.
    SPI2.user.usr_miso_highpart = 0;  // The TX buffer should use W0-7.
    SPI2.user.usr_mosi      = 1;  // Enable RX shift register.
    SPI2.user.usr_miso      = 1;  // Enable TX shift register.
    // TODO: For some reason, these settings are required, or no clocks are recognized.
    SPI2.user.usr_command   = 1;
    SPI2.user1.val          = 0;  // No address phase.
    SPI2.cmd.usr = 1;
    SPI2.user2.usr_command_bitlen = 0;  // By doing this, we skip write to writing the buffer.
    SPI2.clock.clkcnt_l           = 0;  // Must be 0 in slave mode.
    SPI2.clock.clkcnt_h           = 0;  // Must be 0 in slave mode.
    SPI2.ctrl.fastrd_mode         = 0;  // No need of multi-lane SPI.
    SPI2.pin.ck_dis               = 1;  // We have no need of a clock output.
    SPI2.pin.ck_idle_edge         = cpol ? 1 : 0;  // CPOL
    SPI2.user.ck_i_edge           = (cpol ^ cpha) ? 1 : 0;
    SPI2.ctrl2.mosi_delay_mode    = (cpol ^ cpha) ? 1 : 2;
    SPI2.ctrl2.miso_delay_mode    = 0;  //
    SPI2.ctrl2.miso_delay_num     = 0;  //
    SPI2.ctrl2.mosi_delay_num     = 0;  //
    SPI2.slave.trans_done         = 0;  // Clear txfr-done bit.
    SPI2.slave.trans_inten        = 1;  // Enable txfr-done interrupt.
    SPI2.slv_rdbuf_dlen.bit_len   = 2048;  // TODO: Arbitrary as long as >length?
    SPI2.slv_wrbuf_dlen.bit_len   = 2048;  // TODO: Arbitrary as long as >length?

    //SPI2.dma_conf.out_eof_mode    = 1;  //
    SPI2.dma_conf.out_eof_mode    = 0;  //
    SPI2.dma_conf.dma_continue    = 1;  //
    SPI2.dma_conf.out_auto_wrback = 1;  //

    reset_spi2_dma();
    uint32_t intrpt_mask = (
      SPI_OUT_TOTAL_EOF_INT_ENA |
      SPI_OUT_EOF_INT_ENA |
      SPI_OUT_DONE_INT_ENA |
      SPI_IN_SUC_EOF_INT_ENA |
      SPI_IN_ERR_EOF_INT_ENA |
      SPI_IN_DONE_INT_ENA |
      SPI_INLINK_DSCR_ERROR_INT_ENA |
      SPI_OUTLINK_DSCR_ERROR_INT_ENA |
      SPI_INLINK_DSCR_EMPTY_INT_ENA);

    SPI2.dma_int_clr.val  = intrpt_mask;
    SPI2.dma_int_ena.val  = intrpt_mask;

    SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, 3, 1, 2);   // Point DMA channel to HSPI.

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_cs],   PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_clk],  PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_mosi], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_miso], PIN_FUNC_GPIO);

    gpio_set_direction(p_cs,   GPIO_MODE_INPUT);
    gpio_set_direction(p_clk,  GPIO_MODE_INPUT);
    gpio_set_direction(p_mosi, GPIO_MODE_INPUT);
    gpio_set_direction(p_miso, GPIO_MODE_OUTPUT);

    gpio_matrix_in( p_cs,    HSPICS0_IN_IDX,  false);
    gpio_matrix_in( p_clk,   HSPICLK_IN_IDX,  false);
    gpio_matrix_in( p_mosi,  HSPID_IN_IDX,    false);
    gpio_matrix_out( p_miso, HSPIQ_OUT_IDX,   false, false);

    periph_module_enable(PERIPH_HSPI_MODULE);
    esp_intr_alloc(ETS_SPI2_INTR_SOURCE, ESP_INTR_FLAG_IRAM, spi2_isr, nullptr, nullptr);
    //esp_intr_alloc(ETS_SPI2_DMA_INTR_SOURCE, ESP_INTR_FLAG_IRAM, dma_isr, nullptr, nullptr);
    _er_set_flag(CPLD_FLAG_SPI1_READY, true);
  }
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
    SPI3.slave.slave_mode   = 1;
    SPI3.slave.wr_rd_buf_en = 1;
    SPI3.slave.cs_i_mode    = 2;  // Double-buffered CS signal.
    SPI3.slave.sync_reset   = 1;  // Reset the pins.
    //SPI3.slave.cmd_define   = 1;  // Use the custom slave command mode.

    SPI3.user.doutdin       = 1;  // Full-duplex. Needed to avoid command/status interpretation.
    SPI3.user.usr_mosi_highpart = 0;
    SPI3.user.usr_mosi      = 1;  // Enable this shift register.

    // TODO: For some reason, these settings are required, or no clocks are recognized.
    SPI3.user.usr_command   = 1;
    SPI3.cmd.usr = 1;

    SPI3.user2.usr_command_bitlen = 0;

    SPI3.pin.ck_dis             = 1;  // We have no need of a clock output.
    SPI3.pin.ck_idle_edge       = cpol ? 1 : 0;  // CPOL
    SPI3.user.ck_i_edge         = (cpol ^ cpha) ? 1 : 0;
    SPI3.ctrl2.mosi_delay_mode  = (cpol ^ cpha) ? 1 : 2;
    SPI3.ctrl2.miso_delay_mode  = 0;  //

    SPI3.ctrl2.miso_delay_num   = 0;  //
    SPI3.ctrl2.mosi_delay_num   = 0;  //

    SPI3.ctrl.fastrd_mode   = 0;  // No need of multi-lane SPI.

    SPI3.user1.val          = 0;  // No address phase.

    SPI3.clock.clkcnt_l     = 0;  // Must be 0 in slave mode.
    SPI3.clock.clkcnt_h     = 0;  // Must be 0 in slave mode.

    SPI3.slave.trans_done   = 0;  // Interrupt conditions.
    SPI3.slave.trans_inten  = 1;  //

    SPI3.slv_rdbuf_dlen.bit_len   = 80;

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_cs],   PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_clk],  PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_mosi], PIN_FUNC_GPIO);

    gpio_set_direction(p_cs,   GPIO_MODE_INPUT);
    gpio_set_direction(p_clk,  GPIO_MODE_INPUT);
    gpio_set_direction(p_mosi, GPIO_MODE_INPUT);

    gpio_matrix_in( p_cs,   VSPICS0_IN_IDX,  false);
    gpio_matrix_in( p_clk,  VSPICLK_IN_IDX,  false);
    gpio_matrix_in( p_mosi, VSPID_IN_IDX,    false);

    periph_module_enable(PERIPH_VSPI_MODULE);
    esp_intr_alloc(ETS_SPI3_INTR_SOURCE, ESP_INTR_FLAG_IRAM, spi3_isr, nullptr, nullptr);
    _er_set_flag(CPLD_FLAG_SPI2_READY, true);
  }
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
  output->concatf("\n-- SPI2 (%sline) --------------------\n", (_er_flag(CPLD_FLAG_SPI1_READY)?"on":"OFF"));
  output->concatf("--\t Ops(HW/ISR): 0x%08x / 0x%04x\n", SPI2.slave.trans_cnt, spi2_op_counter_0);
  output->concatf("--\t op_count_1:  0x%04x\n", spi2_op_counter_1);
  output->concatf("--\t Last State:  0x%02x\n", (uint8_t) SPI2.slave.last_state);
  output->concatf("--\t Last CMD:    0x%02x\n", (uint8_t) SPI2.slave.last_command);
  output->concatf("--\t Ext2.State:  0x%02x\n", (uint8_t) SPI2.ext2.st);
  output->concatf("--\t mosi_dlen:   0x%08x\n", SPI2.mosi_dlen.val);
  output->concatf("--\t miso_dlen:   0x%08x\n", SPI2.miso_dlen.val);
  output->concatf("--\t cmd:         0x%08x\n", SPI2.cmd.val);
  output->concatf("--\t ctrl:        0x%08x\n", SPI2.ctrl.val);
  output->concatf("--\t ctrl1:       0x%08x\n", SPI2.ctrl1.val);
  output->concatf("--\t ctrl2:       0x%08x\n", SPI2.ctrl2.val);
  output->concatf("--\t rd_status:   0x%08x\n", SPI2.rd_status.val);
  output->concatf("--\t user:        0x%08x\n", SPI2.user.val);
  //output->concatf("--\t user1:       0x%08x\n", SPI2.user1.val);
  output->concatf("--\t user2:       0x%08x\n", SPI2.user2.val);

  //output->concatf("--\t pin:         0x%08x\n", SPI2.pin.val);
  //output->concatf("--\t clock:       0x%08x\n", SPI2.clock.val);
  output->concatf("--\t slave:       0x%08x\n", SPI2.slave.val);
  output->concatf("--\t slave1:      0x%08x\n", SPI2.slave1.val);
  output->concatf("--\t slave2:      0x%08x\n", SPI2.slave2.val);
  output->concatf("--\t slave3:      0x%08x\n", SPI2.slave3.val);
  //output->concatf("--\t ext0:        0x%08x\n", SPI2.ext0.val);
  //output->concatf("--\t ext1:        0x%08x\n", SPI2.ext1.val);
  //output->concatf("--\t ext2:        0x%08x\n", SPI2.ext2.val);
  //output->concatf("--\t ext3:        0x%08x\n", SPI2.ext3.val);
  //output->concatf("--\t date:        0x%08x\n", SPI2.date.val);
  output->concatf("--\t slv_wrbuf_dlen:  0x%06x\n", (uint32_t) SPI2.slv_wrbuf_dlen.bit_len  );
  output->concatf("--\t slv_rdbuf_dlen:  0x%06x\n", (uint32_t) SPI2.slv_rdbuf_dlen.bit_len  );
  output->concatf("--\t slv_rd_bit:      0x%06x\n--\n", (uint32_t) SPI2.slv_rd_bit.slv_rdata_bit);
  output->concatf("--\t dma_int_raw:     0x%08x\n", SPI2.dma_int_raw.val);
  output->concatf("--\t dma_conf:        0x%08x\n", SPI2.dma_conf.val);
  output->concatf("--\t dma_status (rx/tx):  %c / %c\n", SPI2.dma_status.rx_en?'1':'0', SPI2.dma_status.tx_en?'1':'0');
  //output->concatf("--\t   reserved2 (byte count?):  0x%08x\n--\n", SPI2.dma_status.reserved2);
  output->concatf("--\n--\t spi2_byte_sink:  (%p) 0x%08x\n", (uintptr_t) &spi2_byte_sink, spi2_byte_sink);
  output->concatf("--\t &_ll_(t/r)x_0:   %p / %p\n", (uintptr_t) &_ll_tx_0, (uintptr_t) &_ll_rx_0);
  output->concatf("--\t DMA_(OUT/IN)STATUS: 0x%08x / 0x%08x\n", SPI2.dma_rx_status, SPI2.dma_tx_status);
  output->concatf("--\t DMA_OUT_EOF_DESC_ADDR: %p\n--\n", (uintptr_t) SPI2.dma_out_eof_des_addr);
  output->concatf("--\t Current (out/in):      %p / %p\n", (uintptr_t) SPI2.dma_outlink_dscr, (uintptr_t) SPI2.dma_inlink_dscr);
  output->concatf("--\t outlink_dscr_bf(0/1):  %p / %p\n", (uintptr_t) SPI2.dma_outlink_dscr_bf0, (uintptr_t) SPI2.dma_outlink_dscr_bf1);
  output->concatf("--\t inlink_dscr_bf(0/1):   %p / %p\n", (uintptr_t) SPI2.dma_inlink_dscr_bf0, (uintptr_t) SPI2.dma_inlink_dscr_bf1);

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

  output->concatf("--\n-- SPI3 (%sline) --------------------\n", (_er_flag(CPLD_FLAG_SPI2_READY)?"on":"OFF"));
  output->concatf("--\t Ops:         0x%08x\n\n", SPI3.slave.trans_cnt);
  //output->concatf("--\t Last State:  0x%02x\n", (uint8_t) SPI3.slave.last_state);
  //output->concatf("--\t Last CMD:    0x%02x\n", (uint8_t) SPI3.slave.last_command);
  //output->concatf("--\t Ext2.State:  0x%02x\n", (uint8_t) SPI3.ext2.st);
  //output->concatf("--\t mosi_dlen:   0x%08x\n", SPI3.mosi_dlen.val);
  //output->concatf("--\t miso_dlen:   0x%08x\n", SPI3.miso_dlen.val);
  //output->concatf("--\t cmd:         0x%08x\n", SPI3.cmd.val);
  //output->concatf("--\t ctrl:        0x%08x\n", SPI3.ctrl.val);
  //output->concatf("--\t ctrl1:       0x%08x\n", SPI3.ctrl1.val);
  //output->concatf("--\t ctrl2:       0x%08x\n", SPI3.ctrl2.val);
  //output->concatf("--\t rd_status:   0x%08x\n", SPI3.rd_status.val);
  //output->concatf("--\t user:        0x%08x\n", SPI3.user.val);
  //output->concatf("--\t user1:       0x%08x\n", SPI3.user1.val);
  //output->concatf("--\t user2:       0x%08x\n", SPI3.user2.val);
  //output->concatf("--\t pin:         0x%08x\n", SPI3.pin.val);
  //output->concatf("--\t clock:       0x%08x\n", SPI3.clock.val);
  //output->concatf("--\t slave:       0x%08x\n", SPI3.slave.val);
  //output->concatf("--\t slave1:      0x%08x\n", SPI3.slave1.val);
  //output->concatf("--\t slave2:      0x%08x\n", SPI3.slave2.val);
  //output->concatf("--\t slave3:      0x%08x\n", SPI3.slave3.val);
  //output->concatf("--\t ext0:        0x%08x\n", SPI3.ext0.val);
  //output->concatf("--\t ext1:        0x%08x\n", SPI3.ext1.val);
  //output->concatf("--\t ext2:        0x%08x\n", SPI3.ext2.val);
  //output->concatf("--\t ext3:        0x%08x\n", SPI3.ext3.val);
  //output->concatf("--\t date:        0x%08x\n", SPI3.date.val);
  //output->concatf("--\t slv_wrbuf_dlen:  0x%06x\n", (uint32_t) SPI3.slv_wrbuf_dlen.bit_len  );
  //output->concatf("--\t slv_rdbuf_dlen:  0x%06x\n", (uint32_t) SPI3.slv_rdbuf_dlen.bit_len  );
  //output->concatf("--\t slv_rd_bit:      0x%06x\n", (uint32_t) SPI3.slv_rd_bit.slv_rdata_bit);
  //for (int i = 0; i < 16; i+=4) {
  //  output->concatf(
  //    "--\t SPI3.data_buf[%2d-%2d]:   0x%08x  0x%08x  0x%08x  0x%08x\n",
  //    i, i+3,
  //    (uint32_t) SPI3.data_buf[i + 0],
  //    (uint32_t) SPI3.data_buf[i + 1],
  //    (uint32_t) SPI3.data_buf[i + 2],
  //    (uint32_t) SPI3.data_buf[i + 3]
  //  );
  //}
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
  if (_threaded_op) {
    abort(XferFault::BUS_BUSY);
    return XferFault::BUS_BUSY;
  }

  switch (_param_len) {  // Length dictates our transfer setup.
    case 4:
    case 2:
      if (csAsserted()) {
        abort(XferFault::BUS_BUSY);
        return XferFault::BUS_BUSY;
      }
      set_state(XferState::INITIATE);  // Indicate that we now have bus control.
      _threaded_op = this;

      // Setup DMA. We should never be dealing with the SPI FIFO directly
      //   on this bus, since the hardware has linked-list support.
      reset_spi2_dma();
      break;

    case 3:
    case 1:
    case 0:
    default:
      abort(XferFault::BAD_PARAM);
      return XferFault::BAD_PARAM;
  }

  if (2 != _param_len) {
    set_state(XferState::ADDR);
    // This will be a DMA transfer. Code it as two linked lists with the initial
    //   4 bytes from the read-side of the transfer consigned to a bit-bucket.
    //   The first two bytes will contain the version and conf as always, but
    //   IMU traffic should not be bothered with validating this.
    _ll_tx_0.length = 4;
    _ll_tx_0.size   = 4;
    _ll_tx_0.owner  = LLDESC_HW_OWNED;
    _ll_tx_0.sosf   = 0;
    _ll_tx_0.offset = 0;
    _ll_tx_0.empty  = 0;
    _ll_tx_0.buf    = &xfer_params[0];

    //SPI2.mosi_dlen.usr_mosi_dbitlen = (32 + (buf_len << 3)) - 1;
    //SPI2.miso_dlen.usr_miso_dbitlen = (32 + (buf_len << 3)) - 1;
    SPI2.dma_out_link.addr  = ((uint32_t) &_ll_tx_0) & LLDESC_ADDR_MASK;  // TODO: Only 20 of these bits matter?

    switch (opcode) {
      case BusOpcode::TX:
        // For a transmission, the the buffer will contain outbound data.
        // There is no RX component here, so we can simply load in two lists.
        _ll_tx_0.empty  = (uint32_t) &_ll_tx_1;
        _ll_tx_0.sosf   = 1;
        _ll_tx_0.eof    = 0;

        _ll_tx_1.length = buf_len;   // TODO: Is this a blocksize/count situation?
        _ll_tx_1.size   = buf_len;   // TODO: Doc says this must be a multiple of 4.
        _ll_tx_1.owner  = LLDESC_HW_OWNED;
        _ll_tx_1.sosf   = 1;
        _ll_tx_1.offset = 0;
        _ll_tx_1.empty  = 0;
        _ll_tx_1.eof    = 1;
        _ll_tx_1.buf    = buf;
        break;

      case BusOpcode::RX:
        // For a reception, the the buffer will be filled from the bus.
        // We need to shunt the first four bytes to come back into a bit-bucket.
        _ll_tx_0.eof    = 1;   // Implies that the first TX is the only TX.

        _ll_rx_0.length = 4;
        _ll_rx_0.size   = 4;
        _ll_rx_0.owner  = LLDESC_HW_OWNED;
        _ll_rx_0.sosf   = 1;
        _ll_rx_0.offset = 0;
        _ll_rx_0.empty  = (uint32_t) &_ll_rx_1;
        _ll_rx_0.eof    = 0;
        _ll_rx_0.buf    = (uint8_t*) &spi2_byte_sink;

        _ll_rx_1.length = buf_len;   // TODO: Is this a blocksize/count situation?
        _ll_rx_1.size   = buf_len;   // TODO: Doc says this must be a multiple of 4.
        _ll_rx_1.owner  = LLDESC_HW_OWNED;
        _ll_rx_1.sosf   = 0;
        _ll_rx_1.offset = 0;
        _ll_rx_1.empty  = 0;
        _ll_rx_1.eof    = 1;
        _ll_rx_1.buf    = buf;

        SPI2.dma_in_link.addr   = ((uint32_t) &_ll_rx_0) & LLDESC_ADDR_MASK;  // TODO: Only 20 of these bits matter?
        SPI2.dma_in_link.start  = 1;  // Signify DMA readiness.
        break;

      default:
        break;
    }
    SPI2.dma_out_link.start = 1;  // Signify DMA readiness.
  }
  else {
    //SPI2.user.usr_mosi      = 1;  // We turn on the RX shift-register.
    //SPI2.user.usr_miso      = 1;  // We turn on the TX shift-register.
    // We know that we have two params.
    SPI2.data_buf[0] = xfer_params[0] + (xfer_params[1] << 8);
    if (0 == buf_len) {
      // We can afford to read two bytes into the same space as our xfer_params,
      // We don't need DMA. Load the transfer parameters into the FIFO.
      // LSB will be first over the bus.
      buf = &xfer_params[2];  // Careful....
      buf_len = 2;
      set_state(opcode == BusOpcode::TX ? XferState::TX_WAIT : XferState::RX_WAIT);
    }
    else {
      set_state(XferState::ADDR);
    }
  }
  SPI2.cmd.usr = 1;  // Start the transfer.
  //_assert_cs(true);

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
      return -1;

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

    case XferState::QUEUED:
    case XferState::STOP:
    case XferState::UNDEF:

    /* Below are the states that we shouldn't be in at this point... */
    case XferState::INITIATE:
    case XferState::IDLE:
      abort(XferFault::ILLEGAL_STATE);
      break;
  }

  return -1;
}
