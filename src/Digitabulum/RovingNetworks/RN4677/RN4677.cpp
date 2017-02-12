/*
File:   RN4677.cpp
Author: J. Ian Lindsay
Date:   2016.04.17

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


2016.04.30   Migration log
The RN42HID driver from which this class descended is possibly the least-portable
thing I ever wrote. This is the end of an extension of classes with a hierarchy like this...
  EventReceiver
    ManuvrXport
      RNBase
        This class
I will attempt to collect fail in this driver until it is properly abstracted and brought
up to par.

*/

#include "RN4677.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_usart.h>
#include <stm32f7xx_hal_gpio.h>


/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef _dma_handle;

/**
* Used to disable the DMA IRQs at the NVIC.
*
* @param bool enable the interrupts?
*/
void enable_DMA_IRQ(bool enable) {
  if (!enable) {
    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  }
  else {
    // Clear all DMA flags.
    __HAL_DMA_CLEAR_FLAG(&_dma_handle, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6);
    // Clear all possible pending interrupts.
    //DMA_ClearITPendingBit(DMA1_Stream6, ());

    // Allow the DMA module IRQ in the interrupt controller.
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  }
}


void buildDMAMembers() {
  HAL_DMA_DeInit(&_dma_handle);

  _dma_handle.Instance                 = DMA1_Stream6;
  _dma_handle.Init.Channel             = DMA_CHANNEL_4;
  _dma_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;   // Transmit
  _dma_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  _dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
  _dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  _dma_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  _dma_handle.Init.Mode                = DMA_NORMAL;
  _dma_handle.Init.Priority            = DMA_PRIORITY_LOW;
  _dma_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;  // Required for differnt access-widths.
  _dma_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  _dma_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
  _dma_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;

  /* Enable DMA Stream Transfer Complete interrupt */
  //enable_DMA_IRQ(false);
  //__HAL_DMA_ENABLE_IT(&_dma_handle, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE));
}



/*
* This is actually the function that does the work of sending things to
*   the host. It is to be the last stop for a buffer prior to being fed
*   to USART2's DMA channel.
*
* Thank you again, clive1
* https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2Fcortex_mx_stm32%2FSTM32F4%20Discovery%20UART%20DMA%20TX%20Problem&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=380
*
* DMA_InitStructure.DMA_BufferSize         = (uint16_t) buf_len;   // Why did clive1 have (len-1)??   // I know why. clive1 made a mistake.
*/
int8_t init_dma() {
//  // Disable the DMA Tx Stream.
//  if (HAL_DMA_GetState(&_dma_handle) != HAL_DMA_STATE_RESET) __HAL_DMA_DISABLE(&_dma_handle);
//
//  HAL_DMA_Init(&_dma_handle);
//  HAL_DMA_Start_IT(&_dma_handle, (uint32_t) buf, (uint32_t) huart2.pTxBuffPtr, (uint32_t) buf_len);
//
//  enable_DMA_IRQ(true);
//
//  /* Enable the USART Tx DMA request */
//  //USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
  return 0;
}


void RN4677::bt_gpio_5() {
  RN4677* inst = ((RN4677*) RNBase::getInstance());
  if (nullptr == inst) return;
  unsigned long ms = millis();
  if (inst->_last_gpio_5_event != 0) {
    unsigned long delta = wrap_accounted_delta(ms, inst->_last_gpio_5_event);
    if (delta < 500) {
      // Probably the 10Hz signal. Means we are in command mode.
//      if (INSTANCE != nullptr) {
//        if (!((RNBase*) INSTANCE)->_er_flag(RNBASE_FLAG_CMD_MODE)) {
//          ((RNBase*) INSTANCE)->_er_set_flag(RNBASE_FLAG_CMD_MODE);
//          ((RNBase*) INSTANCE)->_er_clear_flag(RNBASE_FLAG_CMD_PEND);
//          ManuvrMsg *nu_event = Kernel::returnEvent(MANUVR_MSG_BT_ENTERED_CMD_MODE);
//          Kernel::isrRaiseEvent(nu_event);
//        }
//      }
    }
    else if (delta < 3500) {
      // Probably the 1Hz signal. Means we are discoverable and waiting for connection.
//      if (INSTANCE != nullptr) {
//        if (((RNBase*) INSTANCE)->_er_flag(RNBASE_FLAG_CMD_MODE)) {
//          ((RNBase*) INSTANCE)->_er_clear_flag(RNBASE_FLAG_CMD_MODE | RNBASE_FLAG_CMD_PEND | RNBASE_FLAG_LOCK_OUT);
//          ManuvrMsg *nu_event = Kernel::returnEvent(MANUVR_MSG_BT_EXITED_CMD_MODE);
//          Kernel::isrRaiseEvent(nu_event);
//        }
//      }
//    }
//    else {
//      //Kernel::raiseEvent(MANUVR_MSG_BT_CONNECTION_GAINED, nullptr);
//      // Should watch GPIO2 for this.
    }
  }
  inst->_last_gpio_5_event = ms;

  #ifdef MANUVR_DEBUG
    if (inst->getVerbosity() > 6) {
      StringBuilder _log;
      _log.concatf("BT GPIO 5: %lu.\n", ms);
      Kernel::log(&_log);
    }
  #endif
}



/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/

extern "C" {
  StringBuilder _tx_buf;  // TODO: Should be a class member. Tired...

  static volatile unsigned char _rx_buf[RN_MAX_UART_STR_LEN];
  static volatile uint8_t _rx_buf_len = 0;

  uint32_t read_millis_0 = 0;
  uint32_t read_millis_1 = 0;

  static volatile bool _tx_in_progress = false;

  /*
  *
  */
  void USART2_IRQHandler() {
    if((__HAL_UART_GET_IT(&huart2, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE) != RESET)) {
      _rx_buf[_rx_buf_len++ % RN_MAX_UART_STR_LEN] = (uint8_t) huart2.Instance->RDR;
      // END SPLICE FROM HAL DRIVER
      ((RN4677*) RNBase::getInstance())->rx_wakeup();
      /* Clear RXNE interrupt flag */
      __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
    }
    else {
      // TX and errors, we let HAL deal with for now.
      HAL_UART_IRQHandler(&huart2);
      USART2->CR1 |= USART_CR1_RXNEIE;
    }
  }



  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    _tx_in_progress = false;
    ((RN4677*) RNBase::getInstance())->tx_wakeup();
  }


  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    Kernel::log("UART2 Error.\n");
  }
}




/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

RN4677::RN4677(RN4677Pins* p) : RNBase("RN4677", (RNPins*) p) {
  _cmd_return_str = RNBASE_STATUS_CMD;
  _cmd_exit_str   = RNBASE_STATUS_END;
  memcpy(&_pins, p, sizeof(RN4677Pins));
}


RN4677::~RN4677() {
  __USART2_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8);
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3 | GPIO_PIN_4);
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3 | GPIO_PIN_6);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
}


void RN4677::tx_wakeup() {
  read_abort_event.fireNow();
}


void RN4677::rx_wakeup() {
  read_abort_event.fireNow();
}



/*******************************************************************************
* ___________                                                  __
* \__    ___/___________    ____   ____________   ____________/  |_
*   |    |  \_  __ \__  \  /    \ /  ___/\____ \ /  _ \_  __ \   __\
*   |    |   |  | \// __ \|   |  \\___ \ |  |_> >  <_> )  | \/|  |
*   |____|   |__|  (____  /___|  /____  >|   __/ \____/|__|   |__|
*                       \/     \/     \/ |__|
* These members are particular to the transport driver and any implicit
*   protocol it might contain.
*******************************************************************************/

int8_t RN4677::connect() {
  return 0;
}


int8_t RN4677::listen() {
  return 0;
}


bool RN4677::write_port(unsigned char* out, int out_len) {
  //HAL_UART_Transmit_DMA
  local_log.concatf("Sending via UART2 (%d bytes)\n", out_len);
  flushLocalLog();
  _tx_in_progress = (HAL_OK == HAL_UART_Transmit_IT(&huart2, out, out_len));
  return _tx_in_progress;
}


int8_t RN4677::read_port() {
  int n = _rx_buf_len;
  if (n > 0) {
    bytes_received += n;
    size_t _rx_buf_taken = feed_rx_buffer((unsigned char*) &_rx_buf[0], n);
    _rx_buf_len = 0;
    read_millis_1 = millis();
    return n;
  }
  return 0;
}


/*******************************************************************************
* RN-Module member fxns
*******************************************************************************/

/**
* Setup GPIO pins and their bindings to on-chip peripherals, if required.
*/
void RN4677::gpioSetup() {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* These Port B pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 2     -      BT_PIO_04
  * 3     -      BT_PIO_15
  * 8     -      BT_PIO_37 (Tentative)
  */
  if (-1 < _pins.p04) gpioDefine(_pins.p04, INPUT);
  if (-1 < _pins.p15) gpioDefine(_pins.p15, INPUT);
  if (-1 < _pins.p37) gpioDefine(_pins.p37, INPUT);

  /* These Port C pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 3     -      BT_LED_1
  */
  if (-1 < _pins.led) setPinFxn(_pins.led, CHANGE, bt_gpio_5);

  /* These Port D pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 6     -      BT_PIO_05 (Configurable) Default: No use
  */
  if (-1 < _pins.p05) gpioDefine(_pins.p05, INPUT);

  /* These Port E pins are inputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 0     -      BT_PIO_34 (Configurable) Default: Pairing key
  * 1     -      BT_PIO_33 (Configurable) Default: Rx_indicator
  * 2     -      BT_PIO_32 (Configurable) Default: Link drop
  * 3     -      BT_PIO_31 (Configurable) Default: Inquiry configure
  */
  if (-1 < _pins.p31) gpioDefine(_pins.p31, INPUT);
  if (-1 < _pins.p32) gpioDefine(_pins.p32, INPUT);
  if (-1 < _pins.p33) gpioDefine(_pins.p33, INPUT);
  if (-1 < _pins.p34) gpioDefine(_pins.p34, INPUT);

  /* These Port B pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 5     0      BT_PIO_20
  */
  if (-1 < _pins.p20) {
    gpioDefine(_pins.p20, OUTPUT);
    setPin(_pins.p20, true);
  }

  /* These Port C pins are open-drain outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 4     1      WAKE_SW
  */
  if (-1 < _pins.swu) {
    gpioDefine(_pins.swu, OUTPUT_OD);
    setPin(_pins.swu, false);
  }

  /* These Port D pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 3     0      BT_SW_BTN
  */
  if (-1 < _pins.sbt) {
    gpioDefine(_pins.sbt, OUTPUT);
    setPin(_pins.sbt, true);
  }

  /* These Port E pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 4     0      ~BT_RESET
  * 5     1      BT_EAN
  * 6     1      BT_PIO_24
  */
  if (-1 < _pins.reset) {
    gpioDefine(_pins.reset, OUTPUT_OD);
    setPin(_pins.reset, false);
  }
  if (-1 < _pins.ean) {
    gpioDefine(_pins.ean, OUTPUT);
    setPin(_pins.ean, false);
  }
  if (-1 < _pins.p24) {
    gpioDefine(_pins.p24, OUTPUT);
    setPin(_pins.p24, true);
  }

  // Setting up USART2 to deal with the module...
  __USART2_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USART2_FORCE_RESET();
  __HAL_RCC_USART2_RELEASE_RESET();

  GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_0;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //gpioDefine(0, INPUT);
  //gpioDefine(1, OUTPUT);
  //setPin(1, false);

  GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}



/*
*/
void RN4677::set_bitrate(int _bitrate) {
//  enterCommandMode();
//  StringBuilder *temp = new StringBuilder("U,921K,N");
//  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
//  exitCommandMode();

  HAL_NVIC_DisableIRQ(USART2_IRQn);
  HAL_UART_DeInit(&huart2);

  configured_bitrate = _bitrate;

  /* Now the USART_InitStruct is used to define the
   * properties of USART2
   */
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = configured_bitrate;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  //HAL_UART_Init(&huart2);
  //USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_IDLEIE;

  HAL_UART_Init(&huart2);    // finally this enables the complete USART2 peripheral
  USART2->CR1 |= USART_CR1_RXNEIE;
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
}



/*
* Since the RN42 retains settings across power cycles, we might need the
*   ability to put the thing into a known state before trying to talk to it.
* This fxn should retrace the init procedures only to the point of config. Once
*   this class works reasonably well, we should never be running in this mode,
*   except for recovery from serious amnesia.
*/
void RN4677::force_9600_mode(bool force_low_speed) {
  // TODO: This convention and nomenclature are inverted.
  if (force_low_speed) {
    set_bitrate(9600);
  }
  else {
    set_bitrate(115200);
  }
  reset();
}

void RN4677::factoryReset() {
  // TODO: This should probably be done...
}


int8_t RN4677::modulePower(bool power) {
  if (power ^ _module_power) {
    _module_power = power;
    setPin(_pins.sbt, power);
    return 1;
  }
  return 0;
}


int8_t RN4677::moduleSleep(bool sleep) {
  if (sleep ^ _module_sleep) {
    _module_sleep = sleep;
    setPin(_pins.swu, !sleep);  // Active-low
    return 1;
  }
  return 0;
}


/*******************************************************************************
* ######## ##     ## ######## ##    ## ########  ######
* ##       ##     ## ##       ###   ##    ##    ##    ##
* ##       ##     ## ##       ####  ##    ##    ##
* ######   ##     ## ######   ## ## ##    ##     ######
* ##        ##   ##  ##       ##  ####    ##          ##
* ##         ## ##   ##       ##   ###    ##    ##    ##
* ########    ###    ######## ##    ##    ##     ######
*
* These are overrides from EventReceiver interface...
*******************************************************************************/
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
int8_t RN4677::callback_proc(ManuvrMsg* event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = (0 == event->refCount()) ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    case MANUVR_MSG_XPORT_SEND:
      event->clearArgs();
      break;
    default:
      return_value = RNBase::callback_proc(event);
      break;
  }

  return return_value;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void RN4677::printGPIOInfo(StringBuilder* output) {
  output->concatf("-- Module mode:            %s\n", RN4677Pins::getModuleModeString(_pins.getModuleMode()));
  output->concatf("--\t p05: %s \t p31: %s\n", (readPin(_pins.p05) ? "hi" : "lo"), (readPin(_pins.p31) ? "hi" : "lo"));
  output->concatf("--\t p32: %s \t p33: %s\n", (readPin(_pins.p32) ? "hi" : "lo"), (readPin(_pins.p33) ? "hi" : "lo"));
  output->concatf("--\t p34: %s \t p37: %s\n", (readPin(_pins.p34) ? "hi" : "lo"), (readPin(_pins.p37) ? "hi" : "lo"));
  if (getVerbosity() > 5) {
    output->concatf("-- last_gpio5              %u\n\n", _last_gpio_5_event);
  }
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void RN4677::printDebug(StringBuilder* output) {
  RNBase::printDebug(output);
  output->concatf("-- huart2.State:           0x%08x\n", (unsigned long) huart2.State);
  if (huart2.ErrorCode) {
    output->concatf("-- huart2.ErrorCode:       0x%08x\n", (unsigned long) huart2.ErrorCode);
  }
  output->concatf("-- _tx_in_progress:        %s\n", (_tx_in_progress ? "yes" : "no"));
  output->concatf("-- _rx_buf (%02d bytes:     ", _rx_buf_len);
  for (int i = 0; i < _rx_buf_len; i++) { output->concatf(" %02x", _rx_buf[i]); }
  output->concatf("\n-- read_millis_0:          %lu\n", read_millis_0);
  output->concatf("-- read_millis_1:          %lu\n", read_millis_1);
}



int8_t RN4677::notify(ManuvrMsg* active_event) {
  int8_t return_value = 0;

  switch (active_event->eventCode()) {
    default:
      return_value += RNBase::notify(active_event);
      break;
  }

  flushLocalLog();
  return return_value;
}


#if defined(MANUVR_CONSOLE_SUPPORT)
void RN4677::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = 0;
  if (*(str) != 0) {
    temp_byte = atoi((char*) str+1);
  }

  switch (*(str)) {
    case 'S':
    case 's':
      if (moduleSleep((*(str) == 'S'))) {
        local_log.concatf("RN4677 %s.\n", (*(str) == 'S' ? "asleep" : "awake"));
      }
      break;
    case 'P':
    case 'p':
      if (modulePower((*(str) == 'P'))) {
        local_log.concatf("RN4677 o%s.\n", (*(str) == 'P' ? "n" : "ff"));
      }
      break;
    case 'e':
      read_port();
      break;
    case 'i':
      if (2 == temp_byte) {
        printGPIOInfo(&local_log);
      }
      else {
        RNBase::procDirectDebugInstruction(input);
      }
      break;
    default:
      RNBase::procDirectDebugInstruction(input);
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT




/*******************************************************************************
* THIS STUFF WILL UNDERGO MITOSIS WHEN IT GETS BIG ENOUGH
*******************************************************************************/

const char* RN4677Pins::getModuleModeString(RN4677ModuleMode m) {
  switch (m) {
    case RN4677ModuleMode::LINK_DATA:     return "LINK_DATA";
    case RN4677ModuleMode::LINK_NO_DATA:  return "LINK_IDLE";
    case RN4677ModuleMode::ACCESS:        return "ACCESS";
    case RN4677ModuleMode::SHUTDOWN:      return "SHUTDOWN";
  }
  return "<UNDEF>";
}


RN4677ModuleMode RN4677Pins::getModuleMode() {
  uint8_t return_value = 0;
  if (readPin(p04)) return_value += 2;
  if (readPin(p15)) return_value += 1;
  return (RN4677ModuleMode) return_value;
}
