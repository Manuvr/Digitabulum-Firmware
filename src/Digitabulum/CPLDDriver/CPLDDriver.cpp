/*
File:   CPLDDriver.cpp
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

#include "CPLDDriver.h"
#include "../LSM9DS1/IIU.h"
#include "../ManuLegend/ManuLegend.h"

extern "C" {
  volatile CPLDDriver* cpld = NULL;

  TIM_HandleTypeDef htim1;
  SPI_HandleTypeDef hspi1;
  SPI_HandleTypeDef hspi2;
  DMA_HandleTypeDef _dma_handle_spi2;

  /*
  *
  */
  void DMA1_Stream3_IRQHandler() {
    HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
    Kernel::log("DMA1_Stream3_IRQHandler\n");
  }

  /*
  *
  */
  void SPI1_IRQHandler() {
    //HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_SPI_IRQHandler(&hspi1);
  }

  void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if (NULL != CPLDDriver::current_queue_item) {
      CPLDDriver::current_queue_item->advance_operation(hspi->Instance->SR, hspi->Instance->DR);
    }
  }

  void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    //((CPLDDriver*)cpld)->advance_work_queue();
    if (NULL != CPLDDriver::current_queue_item) {
      CPLDDriver::current_queue_item->advance_operation(hspi->Instance->SR, hspi->Instance->DR);
    }
  }

  /*
  *
  */
  void SPI2_IRQHandler() {
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
    Kernel::log("SPI2_IRQHandler\n");
  }
}

volatile bool timeout_punch = false;


void callback_spi_timeout() {
  if (timeout_punch) {
    if (((CPLDDriver*) cpld)->current_queue_item != NULL) {
      Kernel::log("callback_spi_timeout()\n");
      if (((CPLDDriver*) cpld)->current_queue_item->isComplete()) {
        ((CPLDDriver*) cpld)->advance_work_queue();
      }
      else {
        ((CPLDDriver*) cpld)->current_queue_item->abort(XferFault::BUS_FAULT);
      }
    }
  }
  else {
    timeout_punch = true;
  }
}


/****************************************************************************************************
* DUBUG
****************************************************************************************************/

bool _soft_spi = false;
volatile static uint32_t _temp_spi_r = 0;
volatile static uint32_t _temp_spi_w = 0;
volatile static int _spi1_clks = 0;


/****************************************************************************************************
 ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄       ▄            ▄▄▄▄▄▄▄▄▄▄▄  ▄▄        ▄  ▄▄▄▄▄▄▄▄▄▄
▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌     ▐░▌          ▐░░░░░░░░░░░▌▐░░▌      ▐░▌▐░░░░░░░░░░▌
 ▀▀▀▀█░█▀▀▀▀ ▐░█▀▀▀▀▀▀▀▀▀ ▐░█▀▀▀▀▀▀▀█░▌     ▐░▌          ▐░█▀▀▀▀▀▀▀█░▌▐░▌░▌     ▐░▌▐░█▀▀▀▀▀▀▀█░▌
     ▐░▌     ▐░▌          ▐░▌       ▐░▌     ▐░▌          ▐░▌       ▐░▌▐░▌▐░▌    ▐░▌▐░▌       ▐░▌
     ▐░▌     ▐░█▄▄▄▄▄▄▄▄▄ ▐░█▄▄▄▄▄▄▄█░▌     ▐░▌          ▐░█▄▄▄▄▄▄▄█░▌▐░▌ ▐░▌   ▐░▌▐░▌       ▐░▌
     ▐░▌     ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌     ▐░▌          ▐░░░░░░░░░░░▌▐░▌  ▐░▌  ▐░▌▐░▌       ▐░▌
     ▐░▌      ▀▀▀▀▀▀▀▀▀█░▌▐░█▀▀▀▀█░█▀▀      ▐░▌          ▐░█▀▀▀▀▀▀▀█░▌▐░▌   ▐░▌ ▐░▌▐░▌       ▐░▌
     ▐░▌               ▐░▌▐░▌     ▐░▌       ▐░▌          ▐░▌       ▐░▌▐░▌    ▐░▌▐░▌▐░▌       ▐░▌
 ▄▄▄▄█░█▄▄▄▄  ▄▄▄▄▄▄▄▄▄█░▌▐░▌      ▐░▌      ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌       ▐░▌▐░▌     ▐░▐░▌▐░█▄▄▄▄▄▄▄█░▌
▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌▐░▌       ▐░▌     ▐░░░░░░░░░░░▌▐░▌       ▐░▌▐░▌      ▐░░▌▐░░░░░░░░░░▌
 ▀▀▀▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀▀▀▀▀  ▀         ▀       ▀▀▀▀▀▀▀▀▀▀▀  ▀         ▀  ▀        ▀▀  ▀▀▀▀▀▀▀▀▀▀

Interrupt service routine support functions...
A quick note is in order. These functions are static class members that are called directly from
  the ISRs in stm32f7xx_it.c. They are not themselves ISRs. Keep them as short as possible.
****************************************************************************************************/

/**
* ISR for CPLD GPIO.
*/
void cpld_gpio_isr_0() {
  Kernel::log("CPLD_GPIO_0\n");
}

/**
* ISR for CPLD GPIO.
*/
void cpld_gpio_isr_1() {
  Kernel::log("CPLD_GPIO_1\n");
}


/**
* ISR called when the SPI1_CLK falls.
* This is only used for software-driven SPI and will be removed after debug.
*/
void spi_clk_isr() {
  _spi1_clks++;
  _temp_spi_r = _temp_spi_r << 1;
  _temp_spi_r += (readPin(7) ? 1:0);

  HAL_GPIO_WritePin(
    GPIOA,
    GPIO_PIN_6,
    ((_temp_spi_w >> (31 - (_spi1_clks % 32))) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET
  );
}


/**
* ISR called when the SPI1_CS pin rises (is released by the CPLD).
* This is only used for software-driven SPI and will be removed after debug.
*/
void spi_bus_op_isr(){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  Kernel::log("spi_bus_op_isr()\n");
}

/**
* ISR called by the selectable IRQ source.
*/
void cpld_wakeup_isr(){
  Kernel::log("cpld_wakeup_isr()\n");
}



/****************************************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here. Initializers first, functions second.
****************************************************************************************************/

SPIBusOp  CPLDDriver::preallocated_bus_jobs[PREALLOCATED_SPI_JOBS];
SPIBusOp* CPLDDriver::current_queue_item = NULL;

const unsigned char MSG_ARGS_U8_FLOAT[] = {
  UINT8_FM, FLOAT_FM, 0
};

const unsigned char MSG_ARGS_IMU_READ[] = {
  UINT8_FM, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM, 0  // IMU id and a collection of readings.
};


const unsigned char MSG_ARGS_IMU_LEGEND[] = {
  UINT8_FM, UINT16_FM, UINT16_FM,
  UINT16_FM, UINT16_FM, UINT16_FM, UINT16_FM,
  UINT16_FM, UINT16_FM, UINT16_FM, UINT16_FM,
  UINT16_FM, UINT16_FM, UINT16_FM, UINT16_FM,
  UINT16_FM, UINT16_FM, UINT16_FM, UINT16_FM,
  0     // 37 bytes: An IMU Legend broadcast.
};

/* There are only two grammatical forms represented here. A zero-length, and a giant block of Vectors. */
const unsigned char MSG_ARGS_IMU_MAP_STATE[] = {
  VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
    VECT_4_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, VECT_3_FLOAT, FLOAT_FM,
  0   // (17 IMUs * 3 vectors + 1 float per IMU) * (4 bytes per float) = 680) bytes: Statement of debug map.
};    // 0 bytes: Request for present map.



const MessageTypeDef cpld_message_defs[] = {
  /* These are messages specific to Digitabulum. */
  {  DIGITABULUM_MSG_IMU_IRQ_RAISED       , 0x0000,               "IMU_IRQ_RAISED"     , ManuvrMsg::MSG_ARGS_NONE }, // IRQ asserted by CPLD.
  {  DIGITABULUM_MSG_IMU_READ             , 0x0000,               "IMU_READ"           , MSG_ARGS_IMU_READ },  // IMU read request. Argument is the ID.
  {  DIGITABULUM_MSG_IMU_MAP_STATE        , MSG_FLAG_EXPORTABLE,  "IMU_MAP_STATE"      , MSG_ARGS_IMU_MAP_STATE }, //
  {  DIGITABULUM_MSG_IMU_INIT             , MSG_FLAG_EXPORTABLE,  "IMU_INIT"           , ManuvrMsg::MSG_ARGS_U8 }, // Signal to build the IMUs.

  {  DIGITABULUM_MSG_CPLD_RESET_COMPLETE  , 0x0000,               "CPLD_RST_COMPLETE"  , ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_CPLD_RESET_CALLBACK  , 0x0000,               "CPLD_RST_CALLBACK"  , ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_IMU_LEGEND           , MSG_FLAG_EXPORTABLE,  "IMU_LEGEND"         , MSG_ARGS_IMU_LEGEND }, // No args? Asking for this legend. Many args: Legend provided.

  {  DIGITABULUM_MSG_IMU_QUAT_CRUNCH      , 0x0000,               "IMU_QUAT_CRUNCH"    , ManuvrMsg::MSG_ARGS_U8 }, //
  {  DIGITABULUM_MSG_IMU_TAP              , MSG_FLAG_EXPORTABLE,  "IMU_TAP"            , MSG_ARGS_U8_FLOAT }, // IMU id and optional threshold.
  {  DIGITABULUM_MSG_IMU_DOUBLE_TAP       , MSG_FLAG_EXPORTABLE,  "IMU_DOUBLE_TAP"     , MSG_ARGS_U8_FLOAT }, // IMU id and optional threshold.

  {  DIGITABULUM_MSG_SPI_QUEUE_READY      , 0x0000,               "SPI_Q_RDY"          , ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_SPI_CB_QUEUE_READY   , 0x0000,               "SPICB_RDY"          , ManuvrMsg::MSG_ARGS_NONE }, //
};


void CPLDDriver::transferSignal() {
  _temp_spi_r = 0;
  _spi1_clks = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}


void CPLDDriver::softSend(uint8_t val_0, uint8_t val_1) {
  if (_soft_spi) {
    _temp_spi_w = (val_0 << 24) + (val_1 << 16);
    HAL_GPIO_WritePin(
      GPIOA,
      GPIO_PIN_6,
      ((_temp_spi_w & 0x80000000) ? GPIO_PIN_SET : GPIO_PIN_RESET)
    );
    transferSignal();

    Kernel::log("\nI/O 16 bits...\n");
  }
}

void CPLDDriver::softSend(uint32_t val_0) {
  if (_soft_spi) {
    _temp_spi_w = val_0;
    HAL_GPIO_WritePin(
      GPIOA,
      GPIO_PIN_6,
      ((_temp_spi_w & 0x80000000) ? GPIO_PIN_SET : GPIO_PIN_RESET)
    );
    transferSignal();

    Kernel::log("\nI/O 32 bits...\n");
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

/**
* Constructor. Also populates the global pointer reference.
*/
CPLDDriver::CPLDDriver() {
  __class_initializer();

  if (NULL == cpld) {
    cpld = this;
    ManuvrMsg::registerMessages(cpld_message_defs, sizeof(cpld_message_defs) / sizeof(MessageTypeDef));
  }

  // Build some pre-formed Events.
  event_spi_callback_ready.repurpose(DIGITABULUM_MSG_SPI_CB_QUEUE_READY);
  event_spi_callback_ready.isManaged(true);
  event_spi_callback_ready.specific_target = (EventReceiver*) this;
  event_spi_callback_ready.originator      = (EventReceiver*) this;
  event_spi_callback_ready.priority        = 5;

  event_spi_queue_ready.repurpose(DIGITABULUM_MSG_SPI_QUEUE_READY);
  event_spi_queue_ready.isManaged(true);
  event_spi_queue_ready.specific_target    = (EventReceiver*) this;
  event_spi_queue_ready.originator         = (EventReceiver*) this;
  event_spi_queue_ready.priority           = 5;

  // Mark all of our preallocated SPI jobs as "No Reap" and pass them into the prealloc queue.
  for (uint8_t i = 0; i < PREALLOCATED_SPI_JOBS; i++) {
    preallocated_bus_jobs[i].returnToPrealloc(true);     // Implies SHOuLD_REAP = false.
    preallocated.insert(&preallocated_bus_jobs[i]);
  }

  current_queue_item = NULL;
}



/**
* Destructor. Should probably never be called.
*/
CPLDDriver::~CPLDDriver() {
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_7);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  __TIM1_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

  HAL_SPI_DeInit(&hspi1);
  HAL_SPI_DeInit(&hspi2);
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_SPI2_CLK_DISABLE();
}


/**
* Setup GPIO pins and their bindings to on-chip peripherals, if required.
*/
void CPLDDriver::gpioSetup() {
  _er_clear_flag(CPLD_FLAG_SPI2_READY);
  __HAL_RCC_SPI2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;

  /* These Port B pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 9     0      ~CPLD Reset
  * 14    0      SPI2_MISO  (SPI2 is slave and Rx-only)
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_9 | GPIO_PIN_14;
  GPIO_InitStruct.Mode       = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9|GPIO_PIN_14, GPIO_PIN_RESET);

  /* These Port C pins are inputs with a wakeup ISR attached to
  *    the rising-edge.
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 13    0      IRQ_WAKEUP
  */
  setPinFxn(45, CHANGE, cpld_wakeup_isr);

  /* These Port E pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 11    0      CPLD_GPIO_0
  * 14    0      CPLD_GPIO_1
  */
  setPinFxn(75, CHANGE, cpld_gpio_isr_0);
  setPinFxn(78, CHANGE, cpld_gpio_isr_1);

  /* These Port C pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 2     1      DEN_AG_CARPALS
  */
  GPIO_InitStruct.Pin   = GPIO_PIN_2;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

  /* These Port B pins are associated with the SPI2 peripheral:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 12  SPI2_CS
  * 13  SPI2_CLK
  * 15  SPI2_MOSI
  */
  GPIO_InitStruct.Pin       = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hspi2.Instance            = SPI2;
  hspi2.Init.Mode           = SPI_MODE_SLAVE;
  hspi2.Init.Direction      = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.NSS            = SPI_NSS_HARD_INPUT;
  hspi2.Init.DataSize       = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity    = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase       = SPI_PHASE_1EDGE;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode         = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial  = 7;
  hspi2.Init.CRCLength      = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode       = SPI_NSS_PULSE_DISABLED;
  _er_set_flag(CPLD_FLAG_SPI2_READY, (HAL_OK == HAL_SPI_Init(&hspi2)));

  // We handle SPI2 in this class. Setup the DMA members.
  //HAL_DMA_DeInit(&_dma_handle_spi2);

  _dma_handle_spi2.Instance                 = DMA1_Stream3;
  _dma_handle_spi2.Init.Channel             = DMA_CHANNEL_0;
  _dma_handle_spi2.Init.Direction           = DMA_PERIPH_TO_MEMORY;   // Receive
  _dma_handle_spi2.Init.PeriphInc           = DMA_PINC_DISABLE;
  _dma_handle_spi2.Init.MemInc              = DMA_MINC_ENABLE;
  _dma_handle_spi2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  _dma_handle_spi2.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  _dma_handle_spi2.Init.Mode                = DMA_NORMAL;
  _dma_handle_spi2.Init.Priority            = DMA_PRIORITY_LOW;
  _dma_handle_spi2.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;  // Required for differnt access-widths.
  _dma_handle_spi2.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  _dma_handle_spi2.Init.MemBurst            = DMA_MBURST_SINGLE;
  _dma_handle_spi2.Init.PeriphBurst         = DMA_PBURST_SINGLE;

  __HAL_DMA_CLEAR_FLAG(&_dma_handle_spi2, DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4);
  HAL_DMA_Init(&_dma_handle_spi2);

  /* Enable DMA Stream Transfer Complete interrupt */
  __HAL_DMA_ENABLE_IT(&_dma_handle_spi2, DMA_IT_TC);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);

  HAL_DMA_Start_IT(&_dma_handle_spi2, (uint32_t) hspi2.pTxBuffPtr, (uint32_t) _irq_data_0, 10);
}


/**
* Init the timer to provide the CPLD with an external clock. This clock is the
*   most-flexible, and we use it by default.
*/
bool CPLDDriver::_set_timer_base(uint16_t _period) {
  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = _period;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;  // TODO: Move this to 8 to reduce power?
  return (HAL_OK == HAL_TIM_Base_Init(&htim1));
}


/**
* Init the timer to provide the CPLD with an external clock. This clock is the
*   most-flexible, and we use it by default.
*/
void CPLDDriver::init_ext_clk() {
  __TIM1_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;

  /* These Port A pins are outputs at the discretion of the timer:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 8     1      CPLD_EXT_CLK
  */
  GPIO_InitStruct.Pin        = GPIO_PIN_8;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Speed      = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate  = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  _set_timer_base(0xFFF0);  // Make the clock real slow until we need it.

  TIM_ClockConfigTypeDef sClockSourceConfig;
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  HAL_TIM_OC_Init(&htim1);

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter      = 0;
  sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter     = 0;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
}


/**
* Init of SPI peripheral 1 in software mode.
* This will be struck when the CPLD is debugged.
*/
void CPLDDriver::init_spi_soft() {
  GPIO_InitTypeDef GPIO_InitStruct;

  _er_clear_flag(CPLD_FLAG_SPI1_READY | CPLD_FLAG_SPI2_READY);
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_SPI2_CLK_DISABLE();

  /* These Port A pins are associated with the SPI1 peripheral:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 4   SPI1_CS
  * 5   SPI1_CLK
  * 6   SPI1_MISO
  * 7   SPI1_MOSI
  */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  setPinFxn(5, FALLING, spi_clk_isr);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  setPinFxn(4, RISING_PULL_UP, spi_bus_op_isr);
  _soft_spi = true;
}


/**
* Init of SPI peripheral 1. This is broken out because we might be bringing it
*   up and down in a single runtime for debug reasons.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi(uint8_t cpol, uint8_t cpha) {
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t cpol_mode = (cpol) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
  uint8_t cpha_mode = (cpha) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;

  HAL_NVIC_DisableIRQ(SPI1_IRQn);

  if (_er_flag(CPLD_FLAG_SPI1_READY)) {
    _er_clear_flag(CPLD_FLAG_SPI1_READY);
    HAL_SPI_DeInit(&hspi1);
  }
  else {
    __HAL_RCC_SPI1_CLK_ENABLE();
  }

  /* These Port A pins are associated with the SPI1 peripheral:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 4   SPI1_CS
  * 5   SPI1_CLK
  * 6   SPI1_MISO
  * 7   SPI1_MOSI
  */
  GPIO_InitStruct.Pin       = GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_4;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hspi1.Instance            = SPI1;
  hspi1.Init.Mode           = SPI_MODE_SLAVE;
  hspi1.Init.Direction      = SPI_DIRECTION_2LINES;
  hspi1.Init.NSS            = SPI_NSS_HARD_INPUT;
  hspi1.Init.DataSize       = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity    = cpol_mode;
  hspi1.Init.CLKPhase       = cpha_mode;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode         = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial  = 7;
  hspi1.Init.CRCLength      = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode       = SPI_NSS_PULSE_DISABLED;
  _er_set_flag(CPLD_FLAG_SPI1_READY, (HAL_OK == HAL_SPI_Init(&hspi1)));

  HAL_NVIC_EnableIRQ(SPI1_IRQn);

  _soft_spi = false;
}


/**
* Resets the CPLD, and any registers within it.
* Any host-controlled pins affecting CPLD config are not altered by this call,
*   so it might be worth disabling some IRQs prior to doing something that might
*   make them seizure.
*/
void CPLDDriver::reset() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);  // Drive the reset pin low...
  externalOscillator(true);    // Turn on the oscillators...

  cpld_conf_value    = 0x00;     //   our register representations...
  cpld_version       = 0x00;
  cpld_wakeup_source = 0x00;
  cpld_status_value  = 0x00;

  bus_timeout_millis = 5;

  purge_queued_work();          // Purge the SPI queue...
  purge_stalled_job();

  // Fire the oneshot to bring us out of reset after several ms...
  raiseEvent(Kernel::returnEvent(DIGITABULUM_MSG_CPLD_RESET_CALLBACK));
}


/**
* This function evaluates the last-known state against the new state of the
*   the configuration register, and processes consequences.
*
* @param  nu  The updated value of the config register.
*/
void CPLDDriver::_process_conf_update(uint8_t nu) {
  uint8_t diff = cpld_conf_value ^ nu;
  if (diff & CPLD_CONF_BIT_INT_CLK) {
    if (nu & CPLD_CONF_BIT_INT_CLK) {
      // TODO: Optimize the branches out of this once it is shown to work.
      _er_set_flag(CPLD_FLAG_INT_OSC, true);
      // We needed to wait for the last write operation before doing this...
      externalOscillator(false);
    }
    else {
      _er_set_flag(CPLD_FLAG_INT_OSC, false);
    }
  }
  if (diff & CPLD_CONF_BIT_GPIO_0) {
  }
  if (diff & CPLD_CONF_BIT_GPIO_1) {
  }
  if (diff & CPLD_CONF_BIT_DEN_AG_0) {
    _er_set_flag(CPLD_CONF_BIT_DEN_AG_0, (nu & CPLD_CONF_BIT_DEN_AG_0));
  }
  cpld_conf_value = nu;
}



/****************************************************************************************************
*  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄    Members related to the work queue and SPI bus I/O
* ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀▀▀ ▐░█▀▀▀▀▀▀▀█░▌ ▀▀▀▀█░█▀▀▀▀
* ▐░▌          ▐░▌       ▐░▌     ▐░▌
* ▐░█▄▄▄▄▄▄▄▄▄ ▐░█▄▄▄▄▄▄▄█░▌     ▐░▌
* ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌     ▐░▌
*  ▀▀▀▀▀▀▀▀▀█░▌▐░█▀▀▀▀▀▀▀▀▀      ▐░▌
*           ▐░▌▐░▌               ▐░▌
*  ▄▄▄▄▄▄▄▄▄█░▌▐░▌           ▄▄▄▄█░█▄▄▄▄
* ▐░░░░░░░░░░░▌▐░▌          ▐░░░░░░░░░░░▌
*  ▀▀▀▀▀▀▀▀▀▀▀  ▀            ▀▀▀▀▀▀▀▀▀▀▀
*
* SPI transactions happen in two DMA phases:
* * Addressing phase (hereafter referred to as Aphase):
*     Take control of the bus and do either a 1 or 2 byte write, depending on if the address
*     will fit into a 8-bit space. Reason: registers that reside in the CPLD only need a single
*     byte of address, whereas the IMUs are subaddressed within themselves, mandating an extra byte.
* * Transfer phase (hereafter refrered to as TPhase):
*     Move all the bytes required by the operation and close the bus.
****************************************************************************************************/

/**
* When a bus operation completes, it is passed back to the class that created it.
*
* @param  _op  The bus operation that was completed.
* @return SPI_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t CPLDDriver::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasFault()) {
    if (getVerbosity() > 3) local_log.concat("io_op_callback() rejected a callback because the bus op failed.\n");
    return SPI_CALLBACK_ERROR;
  }

  if (BusOpcode::RX == op->get_opcode()) {
    switch (0x7F & op->getRegAddr()) {
      case CPLD_REG_VERSION:
        cpld_version = op->getTransferParam(3);
        if (getVerbosity() > 3) local_log.concatf("CPLD r%d.\n", cpld_version);
        if (0 < cpld_version) {
          Kernel::raiseEvent(DIGITABULUM_MSG_CPLD_RESET_COMPLETE, NULL);
        }
        else {
          if (getVerbosity() > 1) local_log.concatf("CPLD returned a bad version code: 0x%02x\n", cpld_version);
        }
        break;
      case CPLD_REG_STATUS:
        cpld_status_value = op->getTransferParam(3);
        break;
      default:
        if (getVerbosity() > 2) local_log.concatf("An SPIBusOp called back with an unknown register: 0x%02x\n", op->getRegAddr());
        break;
    }
  }
  else if (BusOpcode::TX == op->get_opcode()) {
    switch (op->getRegAddr()) {
      case CPLD_REG_CONFIG:
        _process_conf_update(op->getTransferParam(1));
        break;
      case CPLD_REG_WAKEUP_IRQ:
        cpld_wakeup_source = op->getTransferParam(1);
        break;
      default:
        if (getVerbosity() > 2) local_log.concatf("An SPIBusOp called back with an unknown register: 0x%02x\n", op->getRegAddr());
        break;
    }
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return SPI_CALLBACK_NOMINAL;
}


/**
* This is what we call when this class wants to conduct a transaction on the SPI bus.
* Note that this is different from other class implementations, in that it checks for
*   callback population before clobbering it. This is because this class is also the
*   SPI driver. This might end up being reworked later.
*
* @param  _op  The bus operation to execute.
* @return Zero on success, or appropriate error code.
*/
int8_t CPLDDriver::queue_io_job(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;

  if (NULL != op) {
    if (NULL == op->callback) {
      op->callback = (BusOpCallback*) this;
    }

    if ((getVerbosity() > 6) && (op->callback == (BusOpCallback*) this)) {
      op->profile(true);
    }

    if (op->get_state() != XferState::IDLE) {
      if (getVerbosity() > 3) Kernel::log("Tried to fire a bus op that is not in IDLE state.\n");
      return -4;
    }

    if ((NULL == current_queue_item) && (work_queue.size() == 0)){
      // If the queue is empty, fire the operation now.
      current_queue_item = op;
      advance_work_queue();
      if (bus_timeout_millis) event_spi_timeout.delaySchedule(bus_timeout_millis);  // Punch the timeout schedule.
    }
    else {    // If there is something already in progress, queue up.
      if (_er_flag(CPLD_FLAG_QUEUE_GUARD) && (cpld_max_bus_queue_depth <= work_queue.size())) {
        if (getVerbosity() > 3) Kernel::log("CPLDDriver::queue_io_job(): \t Bus queue at max size. Dropping transaction.\n");
        op->abort(XferFault::QUEUE_FLUSH);
        callback_queue.insertIfAbsent(op);
        if (callback_queue.size() == 1) Kernel::staticRaiseEvent(&event_spi_callback_ready);
        return -1;
      }

      if (0 > work_queue.insertIfAbsent(op)) {
        if (getVerbosity() > 2) {
          local_log.concat("CPLDDriver::queue_io_job(): \t Double-insertion. Dropping transaction with no status change.\n");
          op->printDebug(&local_log);
          Kernel::log(&local_log);
        }
        return -3;
      }
    }
    return 0;
  }
  return -5;
}


/**
* Execute any I/O callbacks that are pending. The function is present because
*   this class contains the bus implementation.
*
* @return the number of callbacks proc'd.
*/
int8_t CPLDDriver::service_callback_queue() {
  int8_t return_value = 0;
  SPIBusOp* temp_op = callback_queue.dequeue();

  while ((NULL != temp_op) && (return_value < spi_cb_per_event)) {
  //if (NULL != temp_op) {
    if (NULL != temp_op->callback) {
      if (getVerbosity() > 6) {
        local_log.concatf("Servicing the SPI callback to address 0x%08x.\n", (uint32_t) temp_op->callback);
        Kernel::log(&local_log);
      }

      int8_t cb_code = temp_op->callback->io_op_callback(temp_op);
      switch (cb_code) {
        case SPI_CALLBACK_RECYCLE:
          temp_op->set_state(XferState::IDLE);
          queue_io_job(temp_op);
          break;

        case SPI_CALLBACK_ERROR:
        case SPI_CALLBACK_NOMINAL:
          // No harm in this yet, since this fxn respects preforms and prealloc.
          reclaim_queue_item(temp_op);
          break;
        default:
          local_log.concatf("Unsure about SPI_CALLBACK_CODE %d.\n", cb_code);
          reclaim_queue_item(temp_op);
          break;
      }
    }
    else {
      // We are the responsible party.
      reclaim_queue_item(temp_op);
    }
    return_value++;
    temp_op = callback_queue.dequeue();
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}


/**
* Calling this function will advance the work queue after performing cleanup operations
*   on the present or pending operation.
*
* @return the number of bus operations proc'd.
*/
int8_t CPLDDriver::advance_work_queue() {
  int8_t return_value = 0;

  timeout_punch = false;
  if (current_queue_item != NULL) {
    if (getVerbosity() > 6) {
      local_log.concat("advance_work_queue() is about to operate on....\n");
      current_queue_item->printDebug(&local_log);
    }

    switch (current_queue_item->get_state()) {
       case XferState::IO_WAIT:
         if (current_queue_item->hasFault()) {
           if (getVerbosity() > 3) local_log.concat("CPLDDriver::advance_work_queue():\t Failed at IO_WAIT.\n");
         }
         else {
           current_queue_item->markComplete();
         }
         // No break on purpose.
       case XferState::COMPLETE:
         callback_queue.insert(current_queue_item);
         current_queue_item = NULL;
         if (callback_queue.size() == 1) Kernel::staticRaiseEvent(&event_spi_callback_ready);
         break;

       case XferState::IDLE:
       case XferState::INITIATE:
         switch (current_queue_item->begin()) {
           case 0:     // Nominal outcome. Transfer started with no problens...
             transferSignal();
             break;
           case -1:    // Bus appears to be in-use. State did not change.
             // Re-throw queue_ready event and try again later.
             if (getVerbosity() > 2) local_log.concat("  advance_work_queue() tried to clobber an existing transfer on chain.\n");
             //Kernel::staticRaiseEvent(&event_spi_queue_ready);  // Bypass our method. Jump right to the target.
             break;
           case -2:    // Began the transfer, and it barffed... was aborted.
             if (getVerbosity() > 3) local_log.concat("CPLDDriver::advance_work_queue():\t Failed to begin transfer after starting.\n");
             callback_queue.insert(current_queue_item);
             current_queue_item = NULL;
             if (callback_queue.size() == 1) Kernel::staticRaiseEvent(&event_spi_callback_ready);
             break;
         }
         break;

       /* Cases below ought to be handled by ISR flow... */
       case XferState::ADDR:
         current_queue_item->advance_operation(0, 0);
       case XferState::STOP:
         if (getVerbosity() > 5) local_log.concatf("State might be corrupted if we tried to advance_queue(). \n");
         break;
       default:
         if (getVerbosity() > 3) local_log.concatf("advance_work_queue() default state \n");
         break;
    }
  }


  if (current_queue_item == NULL) {
    current_queue_item = work_queue.dequeue();
    // Begin the bus operation.
    if (NULL != current_queue_item) {
      if (current_queue_item->begin()) {
        if (getVerbosity() > 2) local_log.concatf("advance_work_queue() tried to clobber an existing transfer on the pick-up.\n");
        Kernel::staticRaiseEvent(&event_spi_queue_ready);  // Bypass our method. Jump right to the target.
      }
      return_value++;
    }
    else {
      // No Queue! Relax...
      event_spi_timeout.enableSchedule(false);  // Punch the timeout schedule.
    }
  }

  if (local_log.length() > 0) Kernel::log(&local_log);

  return return_value;
}


/**
* Purges only the jobs belonging to the given device from the work_queue.
* Leaves the currently-executing job.
*
* @param  dev  The device pointer that owns jobs we wish purged.
*/
void CPLDDriver::purge_queued_work_by_dev(BusOpCallback *dev) {
  if (NULL == dev) return;
  SPIBusOp* current = NULL;

  if (work_queue.size() > 0) {
    int i = 0;
    while (i < work_queue.size()) {
      current = work_queue.get(i);
      if (current->callback == dev) {
        current->abort(XferFault::QUEUE_FLUSH);
        work_queue.remove(current);
        reclaim_queue_item(current);
      }
      else {
        i++;
      }
    }
  }

  // Lastly... initiate the next bus transfer if the bus is not sideways.
  advance_work_queue();
}


/**
* Purges only the work_queue. Leaves the currently-executing job.
*/
void CPLDDriver::purge_queued_work() {
  SPIBusOp* current = NULL;
  while (work_queue.hasNext()) {
    current = work_queue.dequeue();
    current->abort(XferFault::QUEUE_FLUSH);
    reclaim_queue_item(current);
  }

  // Check this last to head off any silliness with bus operations colliding with us.
  purge_stalled_job();
}


/**
* Return a vacant SPIBusOp to the caller, allocating if necessary.
*
* @return an SPIBusOp to be used. Only NULL if out-of-mem.
*/
SPIBusOp* CPLDDriver::issue_spi_op_obj() {
  SPIBusOp* return_value = preallocated.dequeue();
  if (NULL == return_value) {
    preallocation_misses++;
    return_value = new SPIBusOp();
    if (getVerbosity() > 5) Kernel::log("issue_spi_op_obj(): Fresh allocation!\n");
  }
  return return_value;
}


/**
* This fxn will either free() the memory associated with the SPIBusOp object, or it
*   will return it to the preallocation queue.
*
* @param item The SPIBusOp to be reclaimed.
*/
void CPLDDriver::reclaim_queue_item(SPIBusOp* op) {
  if (op->hasFault() && (getVerbosity() > 1)) {    // Print failures.
    StringBuilder log;
    op->printDebug(&log);
    Kernel::log(&log);
  }

  if (op->returnToPrealloc()) {
    if (getVerbosity() > 6) local_log.concatf("CPLDDriver::reclaim_queue_item(): \t About to wipe.\n");
    op->wipe();
    preallocated.insert(op);
  }
  else if (op->shouldReap()) {
    if (getVerbosity() > 6) local_log.concatf("CPLDDriver::reclaim_queue_item(): \t About to reap.\n");
    delete op;
    specificity_burden++;
  }
  else {
    /* If we are here, it must mean that some other class fed us a const SPIBusOp,
       and wants us to ignore the memory cleanup. But we should at least set it
       back to IDLE.*/
    if (getVerbosity() > 6) local_log.concatf("CPLDDriver::reclaim_queue_item(): \t Dropping....\n");
    op->set_state(XferState::IDLE);
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
}


/**
* Purges a stalled job from the active slot.
*/
void CPLDDriver::purge_stalled_job() {
  if (current_queue_item != NULL) {
    current_queue_item->abort(XferFault::QUEUE_FLUSH);
    reclaim_queue_item(current_queue_item);
    current_queue_item = NULL;
  }
}



/****************************************************************************************************
* CPLD register manipulation and integral hardware functions.                                       *
****************************************************************************************************/
/**
* Read an arbitrary CPLD register.
*
* @param  reg_addr  The address.
* @return 0 on success. Nonzero on failure.
*/
int8_t CPLDDriver::readRegister(uint8_t reg_addr) {
  SPIBusOp* temp = issue_spi_op_obj();
  temp->set_opcode(BusOpcode::RX);
  temp->setParams(reg_addr | 0x80, 0);  // Set the READ bit...
  queue_io_job(temp);
  return 0;
}

/**
* Write an arbitrary CPLD register.
*
* @param  reg_addr  The address.
* @param  val       The byte-sized value to place there.
* @return 0 on success. Nonzero on failure.
*/
int8_t CPLDDriver::writeRegister(uint8_t reg_addr, uint8_t val) {
  SPIBusOp* temp = issue_spi_op_obj();
  temp->set_opcode(BusOpcode::TX);
  temp->setParams(reg_addr, val);
  queue_io_job(temp);
  return 0;
}

/**
* Change the frequency of the timer-generated external CPLD clock.
*
* @param  _freq  The desired frequency, in Hz.
* @return 0 on success. Nonzero on failure.
*/
int CPLDDriver::setCPLDClkFreq(int _period) {
  return (_set_timer_base((uint16_t) _period) ? 1 : 0);
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
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  }
  else {
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
  }
}

/**
* The CPLD has an internal oscillator that can continue running if we put the
*   CPU to sleep.
* If we are about to disable the internal oscillator, be sure to fire up the
*   external clock first. Otherwise, the transfer will never complete. When the
*   I/O callback arrives, disable the timer.
*
* @param  on  Should the osciallator be enabled?
*/
void CPLDDriver::internalOscillator(bool on) {
  if (!on) {
    externalOscillator(true);
  }
  setCPLDConfig(CPLD_CONF_BIT_INT_CLK, on);
}

/**
* Calling this function will set the CPLD config register.
*
* @param  mask   Combination of flags to change.
* @param  state  Should the flags be cleared or set?
*/
void CPLDDriver::setCPLDConfig(uint8_t mask, bool state) {
  if (state) {
    writeRegister(CPLD_REG_CONFIG, (cpld_conf_value | mask));
  }
  else {
    writeRegister(CPLD_REG_CONFIG, (cpld_conf_value & ~mask));
  }
}

/**
* Reads and returns the CPLD version. We need to use this for keeping
*   compatability with older CPLD versions.
*
* @return The CPLD version, or 0 if it hasn't been read yet.
*/
uint8_t CPLDDriver::getCPLDVersion() {
  readRegister(CPLD_REG_VERSION);
  return cpld_version;
}



/****************************************************************************************************
* This is where IMU-related functions live.                                                         *
****************************************************************************************************/
/**
* Given an address, find the associated IIU.
*
* @param  test_addr The address to query.
* @return A pointer to the IIU responsible for the given address.
*/
IIU* CPLDDriver::fetch_iiu_by_bus_addr(uint8_t test_addr) {
  if (CPLD_REG_IMU_D5_D_M < test_addr) {
    // Too big. Not an IMU address.
    return NULL;
  }
  else if (CPLD_REG_IMU_D5_D_I < test_addr) {
    // Magnetic aspect.
    return LegendManager::getInstance()->fetchIIU(test_addr - CPLD_REG_IMU_D5_D_I);
  }
  else {
    // Inertial aspect. Bus address and index are equal.
    return LegendManager::getInstance()->fetchIIU(test_addr);
  }
}

/**
* Given an address, find the associated IIU.
*
* @param  test_addr The address to query.
* @return An index to the IIU responsible for the given address.
*/
int8_t CPLDDriver::fetch_iiu_index_by_bus_addr(uint8_t test_addr) {
  if (CPLD_REG_IMU_D5_D_M < test_addr) {
    // Too big. Not an IMU address.
    return -1;
  }
  else if (CPLD_REG_IMU_D5_D_I < test_addr) {
    // Magnetic aspect.
    return (test_addr - CPLD_REG_IMU_D5_D_I);
  }
  else {
    // Inertial aspect. Bus address and index are equal.
    return test_addr;
  }
}

/**
*
*
* @return The index of the IMU with an outstanding IRQ.
*/
int8_t CPLDDriver::iiu_group_irq() {
  int8_t return_value = 0;
  if (getVerbosity() > 2) local_log.concatf("CPLD iiu_group_irq: (0x%08x):  \n", 0);

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}



/****************************************************************************************************
*  ▄▄▄▄▄▄▄▄▄▄▄  ▄               ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄        ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░░▌▐░▌             ▐░▌▐░░░░░░░░░░░▌▐░░▌      ▐░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀▀▀  ▐░▌           ▐░▌ ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌░▌     ▐░▌ ▀▀▀▀█░█▀▀▀▀ ▐░█▀▀▀▀▀▀▀▀▀
* ▐░▌            ▐░▌         ▐░▌  ▐░▌          ▐░▌▐░▌    ▐░▌     ▐░▌     ▐░▌
* ▐░█▄▄▄▄▄▄▄▄▄    ▐░▌       ▐░▌   ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌ ▐░▌   ▐░▌     ▐░▌     ▐░█▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░░▌    ▐░▌     ▐░▌    ▐░░░░░░░░░░░▌▐░▌  ▐░▌  ▐░▌     ▐░▌     ▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀▀▀      ▐░▌   ▐░▌     ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌   ▐░▌ ▐░▌     ▐░▌      ▀▀▀▀▀▀▀▀▀█░▌
* ▐░▌                ▐░▌ ▐░▌      ▐░▌          ▐░▌    ▐░▌▐░▌     ▐░▌               ▐░▌
* ▐░█▄▄▄▄▄▄▄▄▄        ▐░▐░▌       ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌     ▐░▐░▌     ▐░▌      ▄▄▄▄▄▄▄▄▄█░▌
* ▐░░░░░░░░░░░▌        ▐░▌        ▐░░░░░░░░░░░▌▐░▌      ▐░░▌     ▐░▌     ▐░░░░░░░░░░░▌
*  ▀▀▀▀▀▀▀▀▀▀▀          ▀          ▀▀▀▀▀▀▀▀▀▀▀  ▀        ▀▀       ▀       ▀▀▀▀▀▀▀▀▀▀▀
*
* These are overrides from EventReceiver interface...
****************************************************************************************************/
/**
* Debug support function.
*
* @return a pointer to a string constant.
*/
const char* CPLDDriver::getReceiverName() {  return "CPLD";  }


/**
* Fire-up anything that depends on the Kernel...
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t CPLDDriver::bootComplete() {
  EventReceiver::bootComplete();

  gpioSetup();
  init_ext_clk();
  SPIBusOp::buildDMAMembers();

  init_spi(1, 0);  // CPOL=1, CPHA=0, HW-driven

  // An SPI transfer might hang (very unlikely). This will un-hang it.
  event_spi_timeout.alterSchedule(bus_timeout_millis, -1, false, callback_spi_timeout);
  event_spi_timeout.isManaged(true);

  reset();
  return 1;
}


/**
* If we find ourselves in this fxn, it means an event that this class built (the argument)
*   has been serviced and we are now getting the chance to see the results. The argument
*   to this fxn will never be NULL.
*
* Depending on class implementations, we might choose to handle the completed Event differently. We
*   might add values to event's Argument chain and return RECYCLE. We may also free() the event
*   ourselves and return DROP. By default, we will return REAP to instruct the Kernel
*   to either free() the event or return it to it's preallocate queue, as appropriate. If the event
*   was crafted to not be in the heap in its own allocation, we will return DROP instead.
*
* @param  event  The event for which service has been completed.
* @return A callback return code.
*/
int8_t CPLDDriver::callback_proc(ManuvrRunnable *event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = event->kernelShouldReap() ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->event_code) {
    case DIGITABULUM_MSG_SPI_QUEUE_READY:
      return_value = ((work_queue.size() > 0) || (NULL != current_queue_item)) ? EVENT_CALLBACK_RETURN_RECYCLE : return_value;
      break;
    case DIGITABULUM_MSG_SPI_CB_QUEUE_READY:
      return_value = (callback_queue.size() > 0) ? EVENT_CALLBACK_RETURN_RECYCLE : return_value;
      break;
    default:
      break;
  }

  return return_value;
}


int8_t CPLDDriver::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;
  switch (active_event->event_code) {
    /* General system events */
    case MANUVR_MSG_INTERRUPTS_MASKED:
      break;
    case MANUVR_MSG_SYS_REBOOT:
      break;
    case MANUVR_MSG_SYS_BOOTLOADER:
      break;

    /* Things that only this class is likely to care about. */
    case DIGITABULUM_MSG_IMU_IRQ_RAISED:
      return_value = 1;
      break;
    case DIGITABULUM_MSG_SPI_QUEUE_READY:
      advance_work_queue();
      return_value = 1;
      break;
    case DIGITABULUM_MSG_SPI_CB_QUEUE_READY:
      service_callback_queue();
      return_value = 1;
      break;
    case DIGITABULUM_MSG_CPLD_RESET_CALLBACK:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
      if (getVerbosity() > 4) local_log.concat("CPLD reset.\n");
      return_value = 1;
      //getCPLDVersion();
      break;
    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
  return return_value;
}



/****************************************************************************************************
*  ▄▄▄▄▄▄▄▄▄▄   ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄   ▄         ▄  ▄▄▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░▌ ▐░░░░░░░░░░░▌▐░░░░░░░░░░▌ ▐░▌       ▐░▌▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀█░▌▐░█▀▀▀▀▀▀▀▀▀ ▐░█▀▀▀▀▀▀▀█░▌▐░▌       ▐░▌▐░█▀▀▀▀▀▀▀▀▀
* ▐░▌       ▐░▌▐░▌          ▐░▌       ▐░▌▐░▌       ▐░▌▐░▌
* ▐░▌       ▐░▌▐░█▄▄▄▄▄▄▄▄▄ ▐░█▄▄▄▄▄▄▄█░▌▐░▌       ▐░▌▐░▌ ▄▄▄▄▄▄▄▄
* ▐░▌       ▐░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░▌ ▐░▌       ▐░▌▐░▌▐░░░░░░░░▌
* ▐░▌       ▐░▌▐░█▀▀▀▀▀▀▀▀▀ ▐░█▀▀▀▀▀▀▀█░▌▐░▌       ▐░▌▐░▌ ▀▀▀▀▀▀█░▌
* ▐░▌       ▐░▌▐░▌          ▐░▌       ▐░▌▐░▌       ▐░▌▐░▌       ▐░▌
* ▐░█▄▄▄▄▄▄▄█░▌▐░█▄▄▄▄▄▄▄▄▄ ▐░█▄▄▄▄▄▄▄█░▌▐░█▄▄▄▄▄▄▄█░▌▐░█▄▄▄▄▄▄▄█░▌
* ▐░░░░░░░░░░▌ ▐░░░░░░░░░░░▌▐░░░░░░░░░░▌ ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌
*  ▀▀▀▀▀▀▀▀▀▀   ▀▀▀▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀▀▀▀   ▀▀▀▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀▀▀▀▀
*
* Code in here only exists for as long as it takes to debug something. Don't write against these.
****************************************************************************************************/

uint8_t __hack_buffer[16];


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void CPLDDriver::printDebug(StringBuilder *output) {
  if (NULL == output) return;

  EventReceiver::printDebug(output);
  //output->concatf("0x%08x OSC IRQs thus far.\n", cpld_osc_irqs);
  output->concatf("-- Conf                0x%02x\n",      cpld_conf_value);
  output->concatf("-- Osc (Int/Ext)       %s/%s\n",       (_er_flag(CPLD_FLAG_INT_OSC) ? "on":"off"), (_er_flag(CPLD_FLAG_EXT_OSC) ? "on":"off"));
  if (_er_flag(CPLD_FLAG_EXT_OSC)) {
    output->concatf("-- Base GetState       0x%02x\n", HAL_TIM_Base_GetState(&htim1));
    output->concatf("-- PWM GetState        0x%02x\n", HAL_TIM_PWM_GetState(&htim1));
  }
  output->concatf("-- DEN_AG Main         %s\n", (_er_flag(CPLD_FLAG_DEN_AG_STATE) ? "on":"off"));
  output->concatf("-- Bus power conserve  %s\n", ((cpld_conf_value & CPLD_CONF_BIT_PWR_CONSRV) ? "on":"off"));

  output->concatf("--\n-- CPLD_GPIO (0/1)     %s/%s\n",       (readPin(75) ? "hi":"lo"), (readPin(78) ? "hi":"lo"));
  if (getVerbosity() > 6) output->concatf("-- volatile *cpld      0x%08x\n--\n", cpld);

  output->concatf("-- SPI1 (%sline) --------------------\n", (_er_flag(CPLD_FLAG_SPI1_READY)?"on":"OFF"));
  if (_soft_spi) {
    output->concatf("-- cs state            %s\n", (readPin(4) ? "hi":"lo"));
    output->concatf("-- SPI1_MOSI           %s\n", (readPin(7) ? "hi":"lo"));
    output->concatf("-- SPI1_CLK            %s\n", (readPin(5) ? "hi":"lo"));
    output->concatf("-- _spi1_clks          %d\n", _spi1_clks);
    output->concatf("-- _temp_spi_r         0x%08x\n", _temp_spi_r);
    output->concatf("-- _temp_spi_r_h       0x%08x\n", _temp_spi_w);
  }
  else {
    output->concatf("-- hspi1.State:        0x%08x\n", (unsigned long) hspi1.State);
    output->concatf("-- hspi1.ErrorCode:    0x%08x\n", (unsigned long) hspi1.ErrorCode);
    output->concatf("-- hspi1.TxXferCount:  0x%04x\n",     hspi1.TxXferCount);
    output->concatf("-- hspi1.RxXferCount:  0x%04x\n", hspi1.RxXferCount);
    output->concatf("-- __hack_buffer       0x%08x\n--\n", __hack_buffer);
  }

  //if (getVerbosity() > 2) {
  //  output->concatf("-- Guarding queue      %s\n",       (_er_flag(CPLD_FLAG_QUEUE_GUARD)?"yes":"no"));
  //  output->concatf("-- spi_cb_per_event    %d\n--\n",   spi_cb_per_event);
  //}
  //output->concatf("-- prealloc queue size %d\n",     preallocated.size());
  //output->concatf("-- prealloc_misses     %u\n",     (unsigned long) preallocation_misses);
  //output->concatf("-- total_transfers     %u\n",     (unsigned long) SPIBusOp::total_transfers);
  //output->concatf("-- failed_transfers    %u\n",     (unsigned long) SPIBusOp::failed_transfers);
  //output->concatf("-- specificity_burden  %u\n--\n", (unsigned long) specificity_burden);

  //output->concatf("-- bus queue depth:    %d\n-- callback q depth    %d\n\n", work_queue.size(), callback_queue.size());


  if (getVerbosity() > 3) {
    if (current_queue_item != NULL) {
      output->concat("\tCurrently being serviced:\n");
      current_queue_item->printDebug(output);
    }

    if (work_queue.size() > 0) {
      int print_depth = min(3, CPLD_SPI_MAX_QUEUE_PRINT);
      output->concatf("\nQueue Listing (top %d of %d total)\n", print_depth, work_queue.size());
      for (int i = 0; i < print_depth; i++) {
        work_queue.get(i)->printDebug(output);
      }
      output->concat("\n");
    }
  }
  output->concatf("\n-- SPI2 (%sline) --------------------\n", (_er_flag(CPLD_FLAG_SPI2_READY)?"on":"OFF"));
  output->concatf("-- IRQ service:        %sabled",   (_er_flag(CPLD_FLAG_SVC_IRQS)?"en":"dis"));
  output->concat("\n--    _irq_data_0:     ");
  for (int i = 0; i < 10; i++) { output->concatf("%02x", _irq_data_0[i]); }
  output->concat("\n--    _irq_data_1:     ");
  for (int i = 0; i < 10; i++) { output->concatf("%02x", _irq_data_1[i]); }
  output->concat("\n\n");
}


#if defined(__MANUVR_CONSOLE_SUPPORT)
void CPLDDriver::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = ((*(str) != 0) ? atoi((char*) str+1) : 0);

  switch (*(str)) {
    case 'i':        // Readback test
      switch (temp_byte) {
        case 1:
          getCPLDVersion();
          break;
        default:
          printDebug(&local_log);
          break;
      }
      break;

    case 's':     // SPI1 initialization...
      switch (temp_byte) {
        case 4:
          init_spi_soft(); // Direct GPIO manipulation.
          break;
        default:
          init_spi(1, 0);  // COL=1, CPHA=0, HW-driven
          break;
      }
      local_log.concat("Re-initialized SPI1.\n");
      break;

    case '*':
      raiseEvent(Kernel::returnEvent(DIGITABULUM_MSG_IMU_IRQ_RAISED));   // Raise an event
      local_log.concat("Manual IRQ raise.\n");
      break;

    case 'b':
      timeout_punch = false;
      bus_timeout_millis = temp_byte;
      if (0 == temp_byte) {
        if (event_spi_timeout.isScheduled()) {
          event_spi_timeout.isScheduled(false);
        }
      }
      else {
        event_spi_timeout.alterSchedulePeriod(bus_timeout_millis);
      }
      local_log.concatf("SPI bus timeout is now %u\n", bus_timeout_millis);
      break;

    case 't':     // Set the SPI1 timeout.
      if (temp_byte) SPIBusOp::spi_wait_timeout = temp_byte * 10;
      local_log.concatf("SPIBusOp::spi_wait_timeout is %uuS...\n", SPIBusOp::spi_wait_timeout);
      break;
    case 'g':     // SPI1 queue-guard (overflow protection).
      _er_flip_flag(CPLD_FLAG_QUEUE_GUARD);
      local_log.concatf("CPLD guarding SPI queue from overflow?  %s\n", _er_flag(CPLD_FLAG_QUEUE_GUARD)?"yes":"no");
      break;
    case 'm':     // Set the number of callbacks per event.
      if (temp_byte) spi_cb_per_event = temp_byte;
      local_log.concatf("CPLD spi_cb_per_event:  %d\n", spi_cb_per_event);
      break;


    case 'k':     // SPI IRQ service?
      _er_flip_flag(CPLD_FLAG_SVC_IRQS);
      local_log.concatf("CPLD servicing IRQs?  %s\n", _er_flag(CPLD_FLAG_SVC_IRQS)?"yes":"no");
      break;

    case 'o':        // CPLD internal oscillator. 1 to engage.
    case 'O':        // CPLD external oscillator. 1 to engage.
      local_log.concatf(
        "%sabling CPLD %sternal oscillator...\n",
        (temp_byte ? "En" : "Dis"),
        ('O' == *(str) ? "ex" : "in")
      );
      if ('O' == *(str)) {
        externalOscillator(temp_byte > 0);
      }
      else {
        internalOscillator(temp_byte > 0);
      }
      break;

    case 'p':     // Purge the SPI1 work queue.
      purge_queued_work();
      local_log.concat("SPI1 queue purged.\n");
      break;

    case 'd':
      switch (temp_byte) {
        case 0:
          local_log.concat("Disabling SPI interrupts...\n");
          //SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE | SPI_I2S_IT_TXE, DISABLE);
        case 1:
          local_log.concat("Disabling Stream3 interrupts...\n");
          //DMA_ITConfig(DMA2_Stream3, DMA_IT_TC | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME, DISABLE);
        case 2:
          local_log.concat("Disabling Stream0 interrupts...\n");
          //DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME, DISABLE);
        default:
          break;
      }
      break;

    case '+':
      local_log.concatf("Advanced CPLD SPI work queue.\n");
      Kernel::raiseEvent(DIGITABULUM_MSG_SPI_QUEUE_READY, NULL);   // Raise an event
      break;

    case '%':
      if (setCPLDClkFreq(temp_byte << 8)) {
        local_log.concatf("Set ext clock period to %d.\n", temp_byte << 8);
      }
      else {
        local_log.concat("Failed to set ext clock period.\n");
      }
      break;

    case '^':
      local_log.concatf("WAKEUP ISR bound to IRQ signal %d.\n", temp_byte);
      writeRegister(CPLD_REG_WAKEUP_IRQ, temp_byte);
      break;
    case 'E':
    case 'e':
      local_log.concatf("%s IRQ 73.\n", (*(str) == '_' ? "Clearing" : "Setting"));
      setCPLDConfig(CPLD_CONF_BIT_IRQ_73, (*(str) == 'E'));
      break;
    case 'A':
    case 'a':
      local_log.concatf("%sabling IRQ scanning.\n", (*(str) == 'A' ? "En" : "Dis"));
      setCPLDConfig(CPLD_CONF_BIT_IRQ_SCAN, (*(str) == 'A'));
      break;
    case 'W':
    case 'w':
      local_log.concatf("%sabling constant IRQ streaming.\n", (*(str) == 'W' ? "En" : "Dis"));
      setCPLDConfig(CPLD_CONF_BIT_IRQ_STREAM, (*(str) == 'W'));
      break;

    case ':':
    case ';':
      local_log.concatf("%sabling bus power conservation.\n", (*(str) == ':' ? "En" : "Dis"));
      setCPLDConfig(CPLD_CONF_BIT_PWR_CONSRV, (*(str) == ':'));
      break;

    case '-':
    case '_':
      local_log.concatf("%s CPLD_DEN_AG_0.\n", (*(str) == '_' ? "Clearing" : "Setting"));
      setCPLDConfig(CPLD_CONF_BIT_DEN_AG_0, (*(str) == '-'));
      break;

    case '[':
    case '{':
      local_log.concatf("%s CPLD_GPIO_0.\n", (*(str) == '[' ? "Clearing" : "Setting"));
      setCPLDConfig(CPLD_CONF_BIT_GPIO_0, (*(str) == '{'));
      break;

    case ']':
    case '}':
      local_log.concatf("%s CPLD_GPIO_1.\n", (*(str) == ']' ? "Clearing" : "Setting"));
      setCPLDConfig(CPLD_CONF_BIT_GPIO_1, (*(str) == '}'));
      break;

    //case 'R':
    //  if (temp_byte) {
    //    local_log.concat("CPLD reset pin driven high.\n");
    //    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);  // Drive the reset pin high...
    //  }
    //  else {
    //    local_log.concat("CPLD reset pin driven low.\n");
    //    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);  // Drive the reset pin low...
    //  }
    //  break;

    case 'r':
      reset();
      break;

    //case 'Z':
    //  local_log.concatf("softSend(0x28, %d)\n", temp_byte);
    //  softSend(0x28, temp_byte);
    //  break;

    //case 'X':
    //  local_log.concatf("softSend(0x29, %d)\n", temp_byte);
    //  softSend(0x29, temp_byte);
    //  break;

    //case 'z':
    //  local_log.concat("Reading version register.\n");
    //  softSend(0xA8, 0);
    //  break;

    //case 'x':
    //  local_log.concat("Reading status register.\n");
    //  softSend(0xA9, 0);
    //  break;

    case 'c':    // Individual IMU access tests...
      if (temp_byte < 0x22) {
        SPIBusOp* op = NULL;
        for (int z = 0; z < 16; z++) __hack_buffer[z] = 0;
        op = issue_spi_op_obj();
        op->set_opcode(BusOpcode::TX_WAIT_RX);
        op->setParams((temp_byte | 0x80), 0x01, 0x01, 0x8F);
        op->setBuffer(__hack_buffer, 16);
        queue_io_job((BusOp*) op);
      }
      else {
        local_log.concat("IMU out of bounds.\n");
      }
      break;

    default:
      EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
}
#endif  //__MANUVR_CONSOLE_SUPPORT
