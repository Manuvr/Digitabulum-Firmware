/*
File:   target_stm32f746.cpp
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
This file contains functions for running firmware on hardware revision 1.
*/

extern "C" {
  #include <stm32f7xx_hal_dma.h>
  #include <stm32f7xx_hal_tim.h>
  #include <stm32f7xx_hal_def.h>
  #include <stm32f7xx_hal_gpio.h>
  #include <stm32f7xx_hal_spi.h>
  #include <stm32f7xx_hal.h>

  TIM_HandleTypeDef htim1;
  SPI_HandleTypeDef hspi1;
  SPI_HandleTypeDef hspi2;
  DMA_HandleTypeDef _spi2_dma;
  DMA_HandleTypeDef _dma_r;  // SPI1
  DMA_HandleTypeDef _dma_w;  // SPI1

  /*
  * Notated like a const, but should NOT be a const, because we use this as a DMA read sink as well
  *   as a sort of /dev/zero. This never contains any meaningful data.
  */
  static uint32_t STATIC_ZERO = 0;
  static uint32_t STATIC_SINK = 0;
}

/*
* The HAL library does not break this out, and it doesn't support double-buffer.
* Replicated definition from stm32f7xx_hal_dma.c
*/
typedef struct {
  __IO uint32_t ISR;       /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;      /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;



/**
* Should undo all the effects of the init functions.
*/
void CPLDDriver::_deinit() {
  //HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_7);
  //HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  //__TIM1_CLK_DISABLE();
  //HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

  //HAL_SPI_DeInit(&hspi1);
  //HAL_SPI_DeInit(&hspi2);
  //__HAL_RCC_SPI1_CLK_DISABLE();
  //__HAL_RCC_SPI2_CLK_DISABLE();
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


int8_t CPLDDriver::bus_init() {
  init_spi(1, 0);   // CPOL=1, CPHA=0, HW-driven
  return 0;
}

int8_t CPLDDriver::bus_deinit() {
  return 0;
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
    __HAL_SPI_DISABLE(&hspi1);
  }
  else {
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
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

  _dma_r.Instance                  = DMA2_Stream2;
  _dma_r.Init.Channel              = DMA_CHANNEL_3;
  _dma_r.Init.Direction            = DMA_PERIPH_TO_MEMORY;   // Receive
  _dma_r.Init.PeriphInc            = DMA_PINC_DISABLE;
  _dma_r.Init.MemInc               = DMA_MINC_ENABLE;
  _dma_r.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
  _dma_r.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
  _dma_r.Init.Mode                 = DMA_NORMAL;
  _dma_r.Init.Priority             = DMA_PRIORITY_LOW;
  _dma_r.Init.FIFOMode             = DMA_FIFOMODE_DISABLE;  // Required for differnt access-widths.
  _dma_r.Init.FIFOThreshold        = DMA_FIFO_THRESHOLD_FULL;
  _dma_r.Init.MemBurst             = DMA_MBURST_SINGLE;
  _dma_r.Init.PeriphBurst          = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&_dma_r);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  _dma_w.Instance                 = DMA2_Stream3;
  _dma_w.Init.Channel             = DMA_CHANNEL_3;
  _dma_w.Init.Direction           = DMA_MEMORY_TO_PERIPH;   // Transmit
  _dma_w.Init.PeriphInc           = DMA_PINC_DISABLE;
  _dma_w.Init.MemInc              = DMA_MINC_ENABLE;
  _dma_w.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  _dma_w.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  _dma_w.Init.Mode                = DMA_NORMAL;
  _dma_w.Init.Priority            = DMA_PRIORITY_HIGH;
  _dma_w.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;  // Required for differnt access-widths.
  _dma_w.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  _dma_w.Init.MemBurst            = DMA_MBURST_SINGLE;
  _dma_w.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&_dma_w);

  hspi1.Instance            = SPI1;
  hspi1.hdmarx              = &_dma_r;
  hspi1.hdmatx              = &_dma_w;
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
}


/**
* Init of SPI peripheral 1. This is broken out because we might be bringing it
*   up and down in a single runtime for debug reasons.
*
* @param  cpol  Clock polartiy
* @param  cpha  Clock phase
*/
void CPLDDriver::init_spi2(uint8_t cpol, uint8_t cpha) {
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t cpol_mode = (cpol) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
  uint8_t cpha_mode = (cpha) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;

  HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);

  if (_er_flag(CPLD_FLAG_SPI2_READY)) {
    _er_clear_flag(CPLD_FLAG_SPI2_READY);
  }
  else {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
  }

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
  hspi2.Init.DataSize       = SPI_DATASIZE_8BIT;  // TODO: Is this OK with half-word DMA?
  hspi2.Init.CLKPolarity    = cpol_mode;
  hspi2.Init.CLKPhase       = cpha_mode;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode         = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial  = 7;
  hspi2.Init.CRCLength      = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode       = SPI_NSS_PULSE_DISABLED;
  _er_set_flag(CPLD_FLAG_SPI2_READY, (HAL_OK == HAL_SPI_Init(&hspi2)));

  SPI2->CR2 &= ~(SPI_CR2_LDMARX);  // Even byte count, always.
  SPI2->CR1 |= SPI_CR1_SPE;

  // We handle SPI2 in this class. Setup the DMA members.
  _spi2_dma.Instance                 = DMA1_Stream3;
  _spi2_dma.Init.Channel             = DMA_CHANNEL_0;
  _spi2_dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;   // Receive
  _spi2_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
  _spi2_dma.Init.MemInc              = DMA_MINC_ENABLE;
  _spi2_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  _spi2_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  _spi2_dma.Init.Mode                = DMA_CIRCULAR;
  _spi2_dma.Init.Priority            = DMA_PRIORITY_LOW;
  _spi2_dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;  // Required for differnt access-widths.
  _spi2_dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  _spi2_dma.Init.MemBurst            = DMA_MBURST_SINGLE;
  _spi2_dma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&_spi2_dma);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  /* Configure DMA for the IRQ aggregator. */
  _spi2_dma.State = HAL_DMA_STATE_BUSY;
  __HAL_DMA_DISABLE(&_spi2_dma);
  DMA1_Stream3->NDTR  = 10;
  DMA1_Stream3->PAR   = (uint32_t)&SPI2->DR;
  DMA1_Stream3->M0AR  = (uint32_t)_irq_data_1;
  DMA1_Stream3->M1AR  = (uint32_t)_irq_data_0;
  DMA1_Stream3->CR  |= (uint32_t) (DMA_SxCR_DBM | DMA_SxCR_CT);
  DMA1_Stream3->CR  |= (uint32_t) (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
  DMA1_Stream3->FCR |= (uint32_t) DMA_IT_FE;
  __HAL_DMA_ENABLE(&_spi2_dma);

  SPI2->CR2 |= SPI_CR2_RXDMAEN;   // Let'ir rip. Should now manage itself.
}



extern "C" {
  /**
  * DMA ISR. This DMA stream is responsible for..
  * 1) signaling the software that the IRQ aggregator have delivered us a new
  *      set of signals.
  * 2) Maintaining the buffer index so that software reads the correct buffer.
  */
  void DMA1_Stream3_IRQHandler() {
    DMA_Base_Registers* regs = (DMA_Base_Registers*)_spi2_dma.StreamBaseAddress;
    int streamIndex = _spi2_dma.StreamIndex;

    /* Transfer Error Interrupt management */
    if ((regs->ISR & (DMA_FLAG_TEIF0_4 << streamIndex)) != RESET) {
      if(__HAL_DMA_GET_IT_SOURCE(&_spi2_dma, DMA_IT_TE) != RESET) {
        /* Disable the transfer error interrupt */
        __HAL_DMA_DISABLE_IT(&_spi2_dma, DMA_IT_TE);
        /* Clear the transfer error flag */
        regs->IFCR = DMA_FLAG_TEIF0_4 << streamIndex;
        /* Update error code */
        _spi2_dma.ErrorCode |= HAL_DMA_ERROR_TE;
        /* Change the DMA state */
        _spi2_dma.State = HAL_DMA_STATE_ERROR;
        Kernel::log("DMA1_Stream3 Error (Transfer)\n");
      }
    }
    /* FIFO Error Interrupt management */
    if ((regs->ISR & (DMA_FLAG_FEIF0_4 << streamIndex)) != RESET) {
      if(__HAL_DMA_GET_IT_SOURCE(&_spi2_dma, DMA_IT_FE) != RESET) {
        /* Disable the FIFO Error interrupt */
        __HAL_DMA_DISABLE_IT(&_spi2_dma, DMA_IT_FE);
        /* Clear the FIFO error flag */
        regs->IFCR = DMA_FLAG_FEIF0_4 << streamIndex;
        /* Update error code */
        _spi2_dma.ErrorCode |= HAL_DMA_ERROR_FE;
        /* Change the DMA state */
        _spi2_dma.State = HAL_DMA_STATE_ERROR;
        Kernel::log("DMA1_Stream3 Error (FIFO)\n");
      }
    }
    /* Transfer Complete Interrupt management */
    if ((regs->ISR & (DMA_FLAG_TCIF0_4 << streamIndex)) != RESET) {
      if(__HAL_DMA_GET_IT_SOURCE(&_spi2_dma, DMA_IT_TC) != RESET) {
        /* Clear the transfer complete flag */
        regs->IFCR = DMA_FLAG_TCIF0_4 << streamIndex;
      }

      /* Update error code */
      _spi2_dma.ErrorCode |= HAL_DMA_ERROR_NONE;

      if((DMA1_Stream3->CR & (uint32_t)(DMA_SxCR_DBM)) != 0) {
        uint8_t* completed_buf;
        uint8_t* previous_buf;
        if((DMA1_Stream3->CR & DMA_SxCR_CT) == 0) {
          _irq_data_ptr = (uint8_t*) _irq_data_1;  // The consumer of this buffer should start here.
          completed_buf = (uint8_t*) _irq_data_0;
          previous_buf  = (uint8_t*) _irq_data_1;
          //Kernel::log("DMA1_Stream3 Circular Bank 0\n");
          _spi2_dma.State = HAL_DMA_STATE_READY_MEM0;
        }
        else {
          _irq_data_ptr = (uint8_t*) _irq_data_0;  // The consumer of this buffer should start here.
          completed_buf = (uint8_t*) _irq_data_1;
          previous_buf  = (uint8_t*) _irq_data_0;
          //Kernel::log("DMA1_Stream3 Circular Bank 1\n");
          _spi2_dma.State = HAL_DMA_STATE_READY_MEM1;
        }

        *(uint32_t*)(_irq_diff + 0) = *(uint32_t*)(previous_buf + 0) ^ *(uint32_t*)(completed_buf + 0);
        *(uint32_t*)(_irq_diff + 4) = *(uint32_t*)(previous_buf + 4) ^ *(uint32_t*)(completed_buf + 4);
        *(uint16_t*)(_irq_diff + 8) = *(uint16_t*)(previous_buf + 8) ^ *(uint16_t*)(completed_buf + 8);
      }
      else {
        ///* Change the DMA state */
        //_spi2_dma.State = HAL_DMA_STATE_READY_MEM0;
        //__HAL_DMA_DISABLE(&_spi2_dma);
        //if (_irq_data_ptr == _irq_data_0) {
        //  _irq_data_ptr = _irq_data_1;  // The consumer of this buffer should start here.
        //  DMA1_Stream3->M0AR  = (uint32_t)_irq_data_0;
        //  Kernel::log("DMA1_Stream3 Bank 0\n");
        //}
        //else {
        //  _irq_data_ptr = _irq_data_0;  // The consumer of this buffer should start here.
        //  DMA1_Stream3->M0AR  = (uint32_t)_irq_data_1;
        //  Kernel::log("DMA1_Stream3 Bank 1\n");
        //}
        //DMA1_Stream3->CR   &= (uint32_t)(~DMA_SxCR_DBM);
        //DMA1_Stream3->PAR   = (uint32_t)&SPI2->DR;
        //DMA1_Stream3->NDTR  = 10;
        //DMA1_Stream3->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
        //DMA1_Stream3->FCR |= DMA_IT_FE;
        //__HAL_DMA_ENABLE(&_spi2_dma);
      }
      Kernel::isrRaiseEvent(&_irq_data_arrival);
    }
  }

  /*
  * SPI1 ISR.
  * TODO: Take full responsibility for this ISR. Do not rely on HAL driver.
  */
  void SPI1_IRQHandler() {
    HAL_SPI_IRQHandler(&hspi1);
  }


  void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if (hspi == &hspi1) {
      SPIBusOp* c = ((CPLDDriver*) cpld)->currentJob();
      if (c) {
        c->advance_operation(hspi->Instance->SR, hspi->Instance->DR);
      }
    }
  }


  void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    if (hspi == &hspi1) {
      SPIBusOp* c = ((CPLDDriver*) cpld)->currentJob();
      if (c) {
        c->advance_operation(hspi->Instance->SR, hspi->Instance->DR);
      }
    }
  }


  void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
    if (hspi == &hspi1) {
      SPIBusOp* c = ((CPLDDriver*) cpld)->currentJob();
      if (c) {
        c->advance_operation(hspi->Instance->SR, hspi->Instance->DR);
      }
    }
  }


  void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
    if (hspi == &hspi1) {
      SPIBusOp* c = ((CPLDDriver*) cpld)->currentJob();
      if (c) {
        c->abort(XferFault::BUS_FAULT);
      }
    }
  }


  /**
  * DMA ISR. Rx
  */
  void DMA2_Stream2_IRQHandler() {
    Kernel::log("DMA2_Stream2_IRQHandler()\n");
    DMA_Base_Registers* regs = (DMA_Base_Registers*)_dma_r.StreamBaseAddress;
    int streamIndex = _dma_r.StreamIndex;
    /* Transfer Error Interrupt management */
    if ((regs->ISR & (DMA_FLAG_TEIF0_4 << streamIndex)) != RESET) {
      if(__HAL_DMA_GET_IT_SOURCE(&_dma_r, DMA_IT_TE) != RESET) {
        /* Disable the transfer error interrupt */
        __HAL_DMA_DISABLE_IT(&_dma_r, DMA_IT_TE);
        /* Clear the transfer error flag */
        regs->IFCR = DMA_FLAG_TEIF0_4 << streamIndex;
        /* Update error code */
        _dma_r.ErrorCode |= HAL_DMA_ERROR_TE;
        /* Change the DMA state */
        _dma_r.State = HAL_DMA_STATE_ERROR;
        Kernel::log("DMA2_Stream2 Error (Transfer)\n");
      }
    }
    /* FIFO Error Interrupt management */
    if ((regs->ISR & (DMA_FLAG_FEIF0_4 << streamIndex)) != RESET) {
      if(__HAL_DMA_GET_IT_SOURCE(&_dma_r, DMA_IT_FE) != RESET) {
        /* Disable the FIFO Error interrupt */
        __HAL_DMA_DISABLE_IT(&_dma_r, DMA_IT_FE);
        /* Clear the FIFO error flag */
        regs->IFCR = DMA_FLAG_FEIF0_4 << streamIndex;
        /* Update error code */
        _dma_r.ErrorCode |= HAL_DMA_ERROR_FE;
        /* Change the DMA state */
        _dma_r.State = HAL_DMA_STATE_ERROR;
        Kernel::log("DMA2_Stream2 Error (FIFO)\n");
      }
    }
    /* Transfer Complete Interrupt management */
    if ((regs->ISR & (DMA_FLAG_TCIF0_4 << streamIndex)) != RESET) {
      Kernel::log("DMA2_Stream2 TC\n");
      if(__HAL_DMA_GET_IT_SOURCE(&_dma_r, DMA_IT_TC) != RESET) {
        /* Clear the transfer complete flag */
        regs->IFCR = DMA_FLAG_TCIF0_4 << streamIndex;
      }
      /* Update error code */
      _dma_r.ErrorCode |= HAL_DMA_ERROR_NONE;
      /* Change the DMA state */
      _dma_r.State = HAL_DMA_STATE_READY_MEM0;
      __HAL_DMA_DISABLE(&_dma_r);
      SPIBusOp* c = ((CPLDDriver*) cpld)->currentJob();
      if (c) {
        c->advance_operation(0, 0);
      }
    }
  }


  /*
  * DMA ISR. Tx
  */
  void DMA2_Stream3_IRQHandler(void) {
    Kernel::log("DMA2_Stream3_IRQHandler()\n");
    __HAL_DMA_DISABLE_IT(&_dma_r, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
    __HAL_DMA_DISABLE_IT(&_dma_w, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
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
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  }
  else {
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
  }
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void CPLDDriver::printHardwareState(StringBuilder *output) {
  output->concatf("-- SPI1 (%sline) --------------------\n", (_er_flag(CPLD_FLAG_SPI1_READY)?"on":"OFF"));
  output->concatf("-- hspi1.State:        0x%08x\n", (unsigned long) hspi1.State);
  if (hspi2.ErrorCode) {
    output->concatf("-- hspi1.ErrorCode:    0x%08x\n", (unsigned long) hspi1.ErrorCode);
  }
  output->concatf("-- hspi1.RxXferCount:  0x%04x\n", hspi1.RxXferCount);
  output->concatf("-- hspi1.TxXferCount:  0x%04x\n", hspi1.TxXferCount);
  output->concatf("-- __hack_buffer       0x%08x\n--\n", __hack_buffer);
  output->concatf("-- _dma_r State      0x%04x\n", _dma_r.State);
  output->concatf("-- _dma_r->CR        0x%08x\n", DMA2_Stream2->CR);
  output->concatf("-- _dma_r->FCR       0x%08x\n", DMA2_Stream2->FCR);
  output->concatf("-- _dma_r->NDTR      0x%08x\n", DMA2_Stream2->NDTR);
}






/**
* Used to disable the DMA IRQs at the NVIC.
*
* @param bool enable the interrupts?
*/
void enableSPI_DMA(bool enable) {
  if (enable) {
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  }
  else {
    NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    NVIC_DisableIRQ(DMA2_Stream3_IRQn);
  }
}


// Useful trick to mask warnings that the compiler raises, but which we know are
//   intentional.
//   http://stackoverflow.com/questions/3378560/how-to-disable-gcc-warnings-for-a-few-lines-of-code
//   https://gcc.gnu.org/onlinedocs/gcc/Diagnostic-Pragmas.html
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

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

  if (SPI1->SR & SPI_FLAG_BSY) {
    Kernel::log("SPI op aborted before taking bus control.\n");
    abort(XferFault::BUS_BUSY);
    return XferFault::BUS_BUSY;
  }

  set_state(XferState::INITIATE);  // Indicate that we now have bus control.

  if ((opcode == BusOpcode::TX) || (2 < _param_len)) {
    set_state((0 == buf_len) ? XferState::TX_WAIT : XferState::ADDR);
    //__HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_TXE));
    HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) xfer_params, (uint8_t*) &STATIC_SINK, _param_len);
  }
  else {
    set_state((0 == buf_len) ? XferState::RX_WAIT : XferState::ADDR);
    // We can afford to read two bytes into the same space as our xfer_params...
    HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) xfer_params, (uint8_t*)(xfer_params + 2), 2);
  }

  _assert_cs(true);
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
        uint16_t tmpreg = hspi1.Instance->DR;
        tmpreg = hspi1.Instance->DR;
        if (opcode == BusOpcode::TX) {
          set_state(XferState::TX_WAIT);
          HAL_SPI_Transmit_IT(&hspi1, (uint8_t*) buf, buf_len);
        }
        else {
          set_state(XferState::RX_WAIT);
          HAL_SPI_Receive_DMA(&hspi1, buf, buf_len);
          //HAL_NVIC_DisableIRQ(SPI1_IRQn);
          //HAL_DMA_Init(&_dma_r);
          //_dma_r.State = HAL_DMA_STATE_BUSY;
          //__HAL_DMA_DISABLE(&_dma_r);
          //DMA2_Stream2->PAR   = (uint32_t) &hspi1.Instance->DR;
          //DMA2_Stream2->M0AR  = (uint32_t) buf;
          //DMA2_Stream2->NDTR  = buf_len;
          //DMA2_Stream2->CR   &= ~((uint32_t) (DMA_SxCR_DBM | DMA_SxCR_CT));
          //DMA2_Stream2->CR   |= (uint32_t) (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
          //DMA2_Stream2->FCR  |= (uint32_t) DMA_IT_FE;
          //__HAL_DMA_ENABLE(&_dma_r);
          //__HAL_SPI_ENABLE(&hspi1);
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
#pragma GCC diagnostic pop
