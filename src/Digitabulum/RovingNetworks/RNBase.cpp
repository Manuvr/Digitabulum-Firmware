/*
File:   RNBase.cpp
Author: J. Ian Lindsay
Date:   2014.05.22

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

#include <stm32f7xx_hal_dma.h>
#include "RNBase.h"
#include "XenoSession/XenoSession.h"


#define MAX_UART_STR_LEN 255
static volatile unsigned char uart2_received_string[MAX_UART_STR_LEN];
static volatile uint8_t uart2_rec_cnt = 0;


uint32_t read_millis_0 = 0;
uint32_t read_millis_1 = 0;

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

volatile RNBase* RNBase::INSTANCE = NULL;
volatile unsigned long RNBase::last_gpio_5_event = 0;
BTQueuedOperation* RNBase::current_work_item = NULL;

BTQueuedOperation RNBase::__prealloc_pool[PREALLOCATED_BT_Q_OPS];

uint32_t RNBase::prealloc_starves       = 0;
uint32_t RNBase::_heap_instantiations   = 0;
uint32_t RNBase::_heap_frees            = 0;
uint32_t RNBase::rejected_host_messages = 0;
uint32_t RNBase::queue_floods           = 0;


PriorityQueue<BTQueuedOperation*> RNBase::preallocated;     // Messages that we've allocated ahead of time.


BTQueuedOperation* RNBase::fetchPreallocation() {
  BTQueuedOperation* return_value;

  if (0 == preallocated.size()) {
    // We have exhausted our preallocated measurements. Note it.
    prealloc_starves++;
    return_value = new BTQueuedOperation();
    _heap_instantiations++;
  }
  else {
    return_value = preallocated.dequeue();
  }
  return return_value;
}



/**
* Reclaims the given Measurement so its memory can be re-used.
*
* At present, our criteria for preallocation is if the pointer address passed in
*   falls within the range of our __prealloc array. I see nothing "non-portable"
*   about this, it doesn't require a flag or class member, and it is fast to check.
* However, this strategy only works for types that are never used in DMA or code
*   execution on the STM32F4. It may work for other architectures (PIC32, x86?).
*   I also feel like it ought to be somewhat slower than a flag or member, but not
*   by such an amount that the memory savings are not worth the CPU trade-off.
* Consider writing all new cyclical queues with preallocated members to use this
*   strategy. Also, consider converting the most time-critical types to this strategy
*   up until we hit the boundaries of the STM32 CCM.
*                                 ---J. Ian Lindsay   Mon Apr 13 10:51:54 MST 2015
*
* @param Measurement* obj is the pointer to the object to be reclaimed.
*/
void RNBase::reclaimPreallocation(BTQueuedOperation* obj) {
  unsigned int obj_addr = ((uint32_t) obj);
  unsigned int pre_min  = ((uint32_t) INSTANCE->__prealloc_pool);
  unsigned int pre_max  = pre_min + (sizeof(BTQueuedOperation) * PREALLOCATED_BT_Q_OPS);

  obj->wipe();
  if ((obj_addr < pre_max) && (obj_addr >= pre_min)) {
    // If we are in this block, it means obj was preallocated. wipe and reclaim it.
    preallocated.insert(obj);
  }
  else {
    // We were created because our prealloc was starved. we are therefore a transient heap object.
    _heap_frees++;
    delete obj;
  }
}


/****************************************************************************************************
* Contain callback soup here, pl0x...
****************************************************************************************************/
/* Static */
void RNBase::expire_lockout() {
  if (INSTANCE == NULL) return;

  if (((EventReceiver*) INSTANCE)->getVerbosity() > 4) Kernel::log("oneshot_rn42_reenable()\n");
  INSTANCE->lockout_active = false;
  ((RNBase*) INSTANCE)->idleService();
}

/* Scheduler one-shot. Re-allows modules communication. */
void oneshot_rn42_reenable() {
  RNBase::expire_lockout();
}

/* Instance */
void RNBase::start_lockout(uint32_t milliseconds) {
  if (INSTANCE == NULL) return;

  // TODO: Need a cleaner way to accomplish this...
  __kernel->addSchedule(new ManuvrRunnable(milliseconds,  0, true, oneshot_rn42_reenable));
  lockout_active = true;
}


/* Static. Reset callback. */
void RNBase::unreset() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  ((RNBase*) INSTANCE)->start_lockout(550);   // Spec says to wait 500ms after reset.
  ((RNBase*) INSTANCE)->initialized(true);
}

/* Scheduler one-shot. Disasserts the reset pin and starts the lockout timer rolling. */
void oneshot_rn42_reset_disassert() {
  RNBase::unreset();   // Spec says to wait 500ms after reset.
}



/* Scheduler one-shot. Flushes the RX queue. */
void host_read_abort() {
  if (0 == read_millis_0) {   // There is nothing in the buffer. Expire.
    return;
  }
  uint32_t current_millis = millis();
  if (CHARACTER_CHRONOLOGICAL_BREAK < (max(current_millis, read_millis_1) - min(current_millis, read_millis_1))) {
    RNBase::hostRxFlush();
    read_millis_0 = 0;
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

RNBase::RNBase() {
  __class_initializer();
  BTQueuedOperation::buildDMAMembers();
  INSTANCE = this;

  /* Populate all the static preallocation slots for messages. */
  for (uint16_t i = 0; i < PREALLOCATED_BT_Q_OPS; i++) {
    __prealloc_pool[i].wipe();
    preallocated.insert(&__prealloc_pool[i]);
  }

  command_mode_pend = false;
  command_mode      = false;
  lockout_active    = false;
  autoconnect_mode  = false;

  current_work_item = NULL;

  gpioSetup();
  connected(GPIOB->IDR & GPIO_PIN_10);

  // Build some pre-formed Events.
  read_abort_event.repurpose(MANUVR_MSG_XPORT_QUEUE_RDY);
  read_abort_event.isManaged(true);
  read_abort_event.specific_target = (EventReceiver*) this;
  read_abort_event.originator      = (EventReceiver*) this;
  read_abort_event.alterScheduleRecurrence(-1);
  read_abort_event.alterSchedulePeriod(CHARACTER_CHRONOLOGICAL_BREAK);
  read_abort_event.autoClear(false);
  read_abort_event.enableSchedule(false);
  __kernel->addSchedule(&read_abort_event);

  force_9600_mode(false);   // Init the UART.
}


RNBase::~RNBase() {
  // TODO:
  // Should do?
  // Will never be torn down, I'd think.... Maybe in low power modes?
  // Serialize the essentials and pack up?
  reset();           // we effectively park the module in reset state.
}


/*
* If we call this before the boot process is complete, we will not come out of reset.
*/
int8_t RNBase::reset() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);     // Drive reset pin low.
  initialized(false);

  while (work_queue.hasNext()) {
    reclaimPreallocation(work_queue.dequeue());
  }
  if (NULL != current_work_item) {
    reclaimPreallocation(current_work_item);
    current_work_item = NULL;
  }

  lockout_active    = true;
  command_mode_pend = false;

  connected(0 != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10));
  queue_floods      = 0;
  autoconnect_mode  = false;

  tx_buf.clear();

  // Used to disassert the reset line.  TODO: Need a cleaner way to accomplish this...
  __kernel->addSchedule(new ManuvrRunnable(510,  0, true, oneshot_rn42_reset_disassert));
  return 0;
}



/****************************************************************************************************
* Command macros
****************************************************************************************************/

/**
* Send the given string to the module as a command.
*
* @param cmd  A pointer to the string to be sent to the module.
*/
void RNBase::sendGeneralCommand(StringBuilder *cmd) {
  enterCommandMode();
  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, cmd);
  exitCommandMode();
  if (getVerbosity() > 5) Kernel::log(__PRETTY_FUNCTION__, 2, "Sent command %s.\n", cmd->string());
}


/**
* Send the given string to the module as a command.
*
* @param cmd  A pointer to the string to be sent to the module.
*/
void RNBase::sendGeneralCommand(const char *cmd) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(cmd);
  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
  exitCommandMode();
  if (getVerbosity() > 5) Kernel::log(__PRETTY_FUNCTION__, 2, "Sent command %s.\n", cmd);
}


/**
* Put the module into HID mode. This setting persists across runtimes.
*/
void RNBase::setHIDMode(void) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(RNBASE_MODE_HID);
  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
  exitCommandMode();
  if (getVerbosity() > 5) Kernel::log(__PRETTY_FUNCTION__, 2, "Tried to enter HID mode.");
}


/**
* Put the module into SPP mode. This setting persists across runtimes.
*/
void RNBase::setSPPMode(void) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(RNBASE_PROTO_SPP);
  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
  exitCommandMode();
  if (getVerbosity() > 5) Kernel::log(__PRETTY_FUNCTION__, 2, "Tried to enter SPP mode.");
}


/**
* Change the name of the module. This setting persists across runtimes.
*
* @param nu_name The desired name of the bluetooth device as seen by other devices.
*/
void RNBase::setDevName(char *nu_name) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(RNBASE_CMD_CHANGE_NAME);
  if (strlen(nu_name) > 20) {
    // Max length for this field is 20 bytes. Truncate if necessary.
    *(nu_name + 19) = 0;
  }
  temp->concat(nu_name);
  temp->concat("\r\n");
  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
  start_lockout(1100);
  exitCommandMode();
  //sendRebootCommand();
}


/**
* Sends the soft-reboot command to the module.
*/
void RNBase::sendRebootCommand(void) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(RNBASE_CMD_REBOOT);
  insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
  start_lockout(3000);
}



/**
* Send a break sequence to the module.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::sendBreak() {
    StringBuilder *temp = new StringBuilder("\r\n");
    insert_into_work_queue(RNBASE_OP_CODE_CMD_TX, temp);
    return 0;
}

/**
* Put the module into command mode.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::enterCommandMode() {
    StringBuilder *temp = new StringBuilder(RNBASE_MODE_COMMAND);
    insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
    //start_lockout(1100);
    return 0;
}


/**
* Take the module out of command mode.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::exitCommandMode() {
    StringBuilder *temp = new StringBuilder(RNBASE_MODE_EXITCOMMAND);
    insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
    start_lockout(1500);
    return 0;
}



void RNBase::setAutoconnect(bool autocon) {
  if (autoconnect_mode ^ autocon) {
    enterCommandMode();
    autoconnect_mode = autocon;
    StringBuilder *temp = new StringBuilder(autocon ? RNBASE_MODE_AUTOCONNECT : RNBASE_MODE_MANUCONNECT);
    insert_into_work_queue(RNBASE_OP_CODE_CMD_TX_WAIT_RX, temp);
    if (getVerbosity() > 4) Kernel::log(__PRETTY_FUNCTION__, 2, "Autoconnect is now %sabled.", (autocon ? "en" : "dis"));
    exitCommandMode();
    sendRebootCommand();
  }
  else {
    if (getVerbosity() > 4) Kernel::log(__PRETTY_FUNCTION__, 2, "Autoconnect mode was already %sabled.", (autocon ? "en" : "dis"));
  }
}



/****************************************************************************************************
* Asynchronous support fxns
****************************************************************************************************/

/*
* This gets called to service queue events.
*/
int8_t RNBase::idleService(void) {
  /* Prepare to generate bugs....
  I normally wouldn't write something this way. The idea is to force an exit
  through one of the states. We don't know going into this call how many states
  we will traverse before our work is done.
  */
  while (true) {
    if (current_work_item != NULL) {
      if (current_work_item->completed) {   // Is it completed?
        if (current_work_item->xenomsg_id) {
          // If we have a xenomsg_id, we should tell the session that it completed.
          if (NULL != session) {
            //if (current_work_item->opcode) {
              if (getVerbosity() > 4) Kernel::log(__PRETTY_FUNCTION__, 2, "About to mark message complete.\n");
              //session->markMessageComplete(current_work_item->xenomsg_id);
            //}
          }
        }
        reclaimPreallocation(current_work_item);
        current_work_item = NULL;
      }
      else if (current_work_item->initiated) {   // Is it initiated?
        // We'd probably be interfering with somehting if we do anything. So do nothing.
        return -2;
      }
      else if (!lockout_active) {
        // If it isn't completed or initiated, and we aren't locked out, let's kick it off...
        switch (current_work_item->opcode) {
          case RNBASE_OP_CODE_CMD_TX_WAIT_RX:
          case RNBASE_OP_CODE_CMD_TX:
            command_mode_pend = (!command_mode);
            current_work_item->begin();
            break;
          case RNBASE_OP_CODE_TX:
            if (connected()) {
              current_work_item->begin();
            }
            else {
              // Could have started a counterparty-bound message and didn't because: NO COUNTERPARTY
              return -3;
            }
            break;
          default:
            if (getVerbosity() > 1) Kernel::log("idleService(): We should not be here (initiation block).\n");
            break;
        }
        return 1;   // We fired off a transaction.
      }
      else {
        return -1;   // We could have done something, but lockout was active.
      }
    }
    else {
      current_work_item = work_queue.dequeue();
      if (NULL == current_work_item) {
        return 0;  // Nothing more to process.
      }
    }
  }
}



/**
* Put the module into master mode.
* This setting persists across runtimes.
*
* @param force_master_mode Set to true to make the module behave as if it were a host.
*/
void RNBase::master_mode(bool force_master_mode) {
  if (force_master_mode) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  }
  else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  }
  reset();
}


// Call to displace or abort a transaction to a host that disconnected.
// Return 0 indicates caller should burn or wait.
// Return 1 indicates we recycled.
int8_t RNBase::burn_or_recycle_current() {
  if ((NULL != current_work_item) && (current_work_item->opcode == RNBASE_OP_CODE_TX)) {
    // If there is something in-process and it is meant for a counterparty....
    if (! current_work_item->initiated) {
      // If there is an un-initiated counterparty transaction waiting, displace it.
      // Re-queue with retry priority.
      work_queue.insert(current_work_item, RNBASE_RETRY_PRIORITY);
      current_work_item = NULL;   // Let the downstream code do its job...
      return 1;
    }
    else {
      return 0;
    }
  }
  return -1;  // Question didn't make sense.
}



uint32_t RNBase::insert_into_work_queue(uint8_t opcode, StringBuilder* data) {
  BTQueuedOperation *nu = fetchPreallocation();
  nu->set_data(opcode, data);

  //BTQueuedOperation *nu = new BTQueuedOperation(opcode, data->string(), data->length());  // TODO: Preallocate these!
  uint32_t return_value = nu->txn_id;

  /* We need commands to fly to the head of the queue so that they get preferrential service
  even if the data messages are stalled. But to do that we need to add commands with a higher
  priority.
  */
  int priority = RNBASE_DEFAULT_PRIORITY;
  switch (opcode) {
    case RNBASE_OP_CODE_CMD_TX_WAIT_RX:
    case RNBASE_OP_CODE_CMD_TX:
      if (!burn_or_recycle_current()) {
        // Burn
      }

      if ((NULL != current_work_item) && (current_work_item->opcode == RNBASE_OP_CODE_CMD_TX)) {
        if (! current_work_item->initiated) {
          // If there is an un-initiated counterparty transaction waiting, displace it.
          // Re-queue with retry priority.
          work_queue.insert(current_work_item, RNBASE_RETRY_PRIORITY);
          current_work_item = NULL;   // Let the downstream code do its job...
        }
      }
      priority = RNBASE_CMD_PRIORITY;
      work_queue.insert(nu, priority);
      break;
    default:
      if (NULL == current_work_item) {
        current_work_item = nu;
        current_work_item->begin();
      }
      else if (work_queue.size() <= RNBASE_MAX_BT_Q_DEPTH) {
        work_queue.insert(nu, priority);
      }
      else {
        return_value = 0;
        if (getVerbosity() > 3) {
          Kernel::log("Dropping BT send. Queue too large.\n");
        }
        queue_floods++;
        reclaimPreallocation(nu);
      }
      break;
  }
  raiseEvent(&event_bt_queue_ready);

  return return_value;
}


/**************************************************************************
* Private class members...                                                *
**************************************************************************/


/*
* This is a static that gets called when we timeout the read operation.
*/
void RNBase::hostRxFlush(void) {
  if (NULL == INSTANCE) return;

  // TODO: Disable. Yuck... I hate the way this works....
  ((RNBase*)INSTANCE)->read_abort_event.enableSchedule(false);

  read_millis_0 = 0;

  if (uart2_rec_cnt > 0) {
    ((RNBase*)INSTANCE)->feed_rx_buffer((unsigned char*) uart2_received_string, uart2_rec_cnt);
    if (((EventReceiver*) INSTANCE)->getVerbosity() > 4) Kernel::log("Flushed bytes\n");
    uart2_rec_cnt = 0;
  }
  else {
    //Kernel::log(__PRETTY_FUNCTION__, 6, "Flushed 0 bytes.");
  }

  if (INSTANCE->current_work_item != NULL) {
    if (RNBASE_OP_CODE_CMD_TX_WAIT_RX == INSTANCE->current_work_item->opcode) {
      INSTANCE->current_work_item->completed = true;
    }
    Kernel::isrRaiseEvent(&(((RNBase*)INSTANCE)->event_bt_queue_ready));
  }
}


volatile void RNBase::usart2_character_rx(unsigned char c) {
  if (uart2_rec_cnt == 0) {
    // If this is our first character from the host since we last emptied
    // the buffer, raise an event so that we can timeout the transmission.
    read_millis_0 = millis();
    read_millis_1 = read_millis_0;
    ManuvrRunnable *nu_event = Kernel::returnEvent(MANUVR_MSG_BT_RX_BUF_NOT_EMPTY);
    Kernel::isrRaiseEvent(nu_event);
  }
  uart2_received_string[uart2_rec_cnt++] = c;

  if (NULL == INSTANCE) return;

  if ((c == '\n') || (uart2_rec_cnt == MAX_UART_STR_LEN)) {
    /* There is a good chance that the downstream call will result in a malloc() at
    *    some point, and probably two or more fxn calls. So we buffer the received string
    *    for a bit until either...
    *  A) We fill our static buffer, or...
    *  B) Our scheduler timeout expires, or...
    *  C) We get a new line character, which may indicate that the host has finished sending.
    *
    * This will reduce memory fragmentation induced by StringBuilder, and keep friction low
    *   for the ISR.
    */
    ((RNBase*)INSTANCE)->feed_rx_buffer((unsigned char*) uart2_received_string, uart2_rec_cnt);
    read_millis_1 = millis();
    uart2_rec_cnt = 0;
  }
}


/**
* Seek out the first work-queue item that is waiting on inbound data.
* If one is found, write the data to its buffer.
* If there is no such item, create one and start appending to its buffer.
*
* If the buffer contains a '\n', cut it off there and mark the active
*   queue item 'complete'. Raise the idle service event, and put the remainder
*   of the inbound buffer (after the '\n') into a new work-queue item.
*
* A pre-allocated StringBuilder object should be kept at the ready to prevent
*   the need to allocate memory in the middle of an ISR. If that pre-allocated
*   object is used, raise an idle flag so that the main loop allocates another
*   when we aren't so busy.
*
* TODO: This all needs to be DMA driven. Until then, try and remember... you are in an ISR.
*/
void RNBase::feed_rx_buffer(unsigned char *nu, uint8_t len) {
  if (command_mode || command_mode_pend) {
    BTQueuedOperation *local_work_item = (NULL != current_work_item) ? current_work_item : work_queue.get();
    if (NULL != local_work_item) {
        switch (local_work_item->opcode) {
          case RNBASE_OP_CODE_CMD_TX_WAIT_RX:
            // We make sure that we tell the class if we are expecting a change in this state.
            command_mode_pend = (!command_mode);
            local_work_item->data.concat(nu, len);
            //output.concatf("Rx Local work item length: %d\n", local_work_item->data.length());
            break;

          case RNBASE_OP_CODE_CMD_TX:
          case RNBASE_OP_CODE_TX:
          default:
            if (getVerbosity() > 2) {
              local_log.concatf("Don't know what to do with data. In command_mode (or pending), but have wrong opcode for work_queue item.\n\t");
              for (int i = 0; i < len; i++) {
                local_log.concatf("0x%02x ", *(nu + i));
              }
              local_log.concat("\n\n");
            }
            break;
        }
    }
    else {
      if (getVerbosity() > 2) {
        local_log.concatf("Don't know what to do with data. In command_mode (or pending), but have no work_queue item to feed.\n\t");
        for (int i = 0; i < len; i++) {
          local_log.concatf("0x%02x ", *(nu + i));
        }
        local_log.concat("\n\n");
      }
    }
  }
  else {   // This data must be meant for a session... (So we hope)
    if (NULL != session) {   // If we don't yet have a session, create one.
      session->bin_stream_rx(nu, len);  // Feed the session...
    }
  }

  if (local_log.length() > 0) {
    if (getVerbosity() > 6) local_log.prepend("feed_rx_buffer():\t");
    Kernel::log(&local_log);
  }
}



/*
*
*/
int8_t RNBase::sendBuffer(StringBuilder* _to_send) {
  if (NULL == _to_send) return -1;
  if (getVerbosity() > 3) local_log.concatf("We about to print %d bytes to the host.\n", _to_send->length());
  printToHost(_to_send);
  return 0;
}



/*
*
*/
volatile void RNBase::irqServiceBT_data_activity(void) {
  if (NULL == INSTANCE) return;
  if (((EventReceiver*) INSTANCE)->getVerbosity() > 6) Kernel::log(__PRETTY_FUNCTION__, 6, "We aren't doing anything here yet.");
}

/*
* Accept a buffer dump from the UART2 ISR. Feed the RNBASE. This should ultimately be DMA.
*/
volatile void RNBase::irqServiceBT_data_receive(unsigned char* nu, uint8_t len) {
  if (NULL == INSTANCE) return;
  ((RNBase*) INSTANCE)->feed_rx_buffer(nu, len);
}


volatile void RNBase::isr_bt_queue_ready() {
  Kernel::isrRaiseEvent(&(((RNBase*) INSTANCE)->event_bt_queue_ready));
}



volatile void RNBase::bt_gpio_5(unsigned long ms) {
  if (NULL == INSTANCE) return;

  if (last_gpio_5_event != 0) {
    unsigned long delta = max(ms, (unsigned long) last_gpio_5_event) - min(ms, (unsigned long) last_gpio_5_event);
    if (delta < 500) {
      // Probably the 10Hz signal. Means we are in command mode.
      if (INSTANCE != NULL) {
        if (!INSTANCE->command_mode) {
          INSTANCE->command_mode = true;
          INSTANCE->command_mode_pend = false;
          ManuvrRunnable *nu_event = Kernel::returnEvent(MANUVR_MSG_BT_ENTERED_CMD_MODE);
          Kernel::isrRaiseEvent(nu_event);
        }
      }
    }
    else if (delta < 3500) {
      // Probably the 1Hz signal. Means we are discoverable and waiting for connection.
      if (INSTANCE != NULL) {
        if (INSTANCE->command_mode) {
          INSTANCE->command_mode      = false;
          INSTANCE->command_mode_pend = false;
          INSTANCE->lockout_active    = false;
          ManuvrRunnable *nu_event = Kernel::returnEvent(MANUVR_MSG_BT_EXITED_CMD_MODE);
          Kernel::isrRaiseEvent(nu_event);
        }
      }
    }
    else {
      //Kernel::raiseEvent(MANUVR_MSG_BT_CONNECTION_GAINED, NULL);
      // Should watch GPIO2 for this.
    }
  }
  if (((EventReceiver*) INSTANCE)->getVerbosity() > 6) Kernel::log(__PRETTY_FUNCTION__, 0, "BT GPIO 5: %lu.\n", ms);
  last_gpio_5_event = ms;
}



/****************************************************************************************************
 ▄▄▄▄▄▄▄▄▄▄▄  ▄               ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄        ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄
▐░░░░░░░░░░░▌▐░▌             ▐░▌▐░░░░░░░░░░░▌▐░░▌      ▐░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌
▐░█▀▀▀▀▀▀▀▀▀  ▐░▌           ▐░▌ ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌░▌     ▐░▌ ▀▀▀▀█░█▀▀▀▀ ▐░█▀▀▀▀▀▀▀▀▀
▐░▌            ▐░▌         ▐░▌  ▐░▌          ▐░▌▐░▌    ▐░▌     ▐░▌     ▐░▌
▐░█▄▄▄▄▄▄▄▄▄    ▐░▌       ▐░▌   ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌ ▐░▌   ▐░▌     ▐░▌     ▐░█▄▄▄▄▄▄▄▄▄
▐░░░░░░░░░░░▌    ▐░▌     ▐░▌    ▐░░░░░░░░░░░▌▐░▌  ▐░▌  ▐░▌     ▐░▌     ▐░░░░░░░░░░░▌
▐░█▀▀▀▀▀▀▀▀▀      ▐░▌   ▐░▌     ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌   ▐░▌ ▐░▌     ▐░▌      ▀▀▀▀▀▀▀▀▀█░▌
▐░▌                ▐░▌ ▐░▌      ▐░▌          ▐░▌    ▐░▌▐░▌     ▐░▌               ▐░▌
▐░█▄▄▄▄▄▄▄▄▄        ▐░▐░▌       ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌     ▐░▐░▌     ▐░▌      ▄▄▄▄▄▄▄▄▄█░▌
▐░░░░░░░░░░░▌        ▐░▌        ▐░░░░░░░░░░░▌▐░▌      ▐░░▌     ▐░▌     ▐░░░░░░░░░░░▌
 ▀▀▀▀▀▀▀▀▀▀▀          ▀          ▀▀▀▀▀▀▀▀▀▀▀  ▀        ▀▀       ▀       ▀▀▀▀▀▀▀▀▀▀▀

These are overrides from EventReceiver interface...
****************************************************************************************************/
/**
* Fire-up anything that depends on the Kernel...
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t RNBase::bootComplete() {
  EventReceiver::bootComplete();

  __kernel->addSchedule(&read_abort_event);

  event_bt_queue_ready.repurpose(MANUVR_MSG_BT_QUEUE_READY);
  event_bt_queue_ready.isManaged(true);
  event_bt_queue_ready.specific_target = (EventReceiver*) this;
  event_bt_queue_ready.originator      = (EventReceiver*) this;

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
int8_t RNBase::callback_proc(ManuvrRunnable *event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = event->kernelShouldReap() ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->event_code) {
    case MANUVR_MSG_XPORT_SEND:
      event->clearArgs();
      break;
    default:
      break;
  }

  return return_value;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void RNBase::printDebug(StringBuilder *temp) {
  ManuvrXport::printDebug(temp);
  temp->concatf("--- queue_floods            %u\n---\n", (unsigned long) rejected_host_messages);

  temp->concatf("--- __prealloc_pool addres: 0x%08x\n", (uint32_t) __prealloc_pool);
  temp->concatf("--- prealloc depth:         %d\n", preallocated.size());
  temp->concatf("--- queue_floods:           %u\n", (unsigned long) queue_floods);
  temp->concatf("--- _heap_instantiations:   %u\n", (unsigned long) _heap_instantiations);
  temp->concatf("--- _heap_frees:            %u\n---\n", (unsigned long) _heap_frees);

  temp->concatf("--- bitrate to module       %d\n", configured_bitrate);
  temp->concatf("--- lockout_active          %s\n", (lockout_active ? "yes" : "no"));
  temp->concatf("--- command mode (pend)     %s\n", (command_mode_pend ? "yes" : "no"));
  temp->concatf("--- command mode            %s\n", (command_mode ? "yes" : "no"));
  temp->concatf("--- autoconnect             %s\n", (autoconnect_mode ? "yes" : "no"));
  if (getVerbosity() > 5) {
    temp->concatf("--- last_gpio5              %u\n", last_gpio_5_event);
    temp->concatf("--- gpio5 state             %s\n\n", ((GPIOB->IDR & GPIO_PIN_10) ? "high" : "low"));
  }

  BTQueuedOperation* q_item = NULL;

  if (NULL != current_work_item) {
    temp->concat("\n--- In working slot:");
    current_work_item->printDebug(temp);
  }

  if (getVerbosity() > 3) {
    if (work_queue.hasNext()) {
      temp->concatf("\n--- Queue Listing (top %d of %d)", RNBASE_MAX_QUEUE_PRINT, work_queue.size());
      for (int i = 0; i < min(work_queue.size(), RNBASE_MAX_QUEUE_PRINT); i++) {
        q_item = work_queue.get(i);
        q_item->printDebug(temp);
      }
    }
    else {
      temp->concat("--- No Queue\n\n");
    }
  }
}


//int8_t teardownSession() {
//}


int8_t RNBase::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;
  switch (active_event->event_code) {
    case MANUVR_MSG_BT_RX_BUF_NOT_EMPTY:
      // We just received something at the BT port. Start the timeout running...
      read_abort_event.delaySchedule();
      return_value++;
      break;
    case MANUVR_MSG_BT_CONNECTION_LOST:
      connected(false);
      if (getVerbosity() > 3) local_log.concat("We lost our bluetooth connection. About to tear down the session...\n");
      burn_or_recycle_current();
      // Purge the queue.
      for (int i = 0; i < work_queue.size(); i++) {
        BTQueuedOperation* current = work_queue.dequeue();
        if (RNBASE_OP_CODE_TX == current->opcode) {
          reclaimPreallocation(current);
        }
        else {
          work_queue.insert(current);
        }
      }
      return_value++;
      break;

    case MANUVR_MSG_BT_CONNECTION_GAINED:
      connected(true);
      return_value++;
    case MANUVR_MSG_BT_ENTERED_CMD_MODE:
      idleService();
      return_value++;
      break;

    case MANUVR_MSG_XPORT_SEND:
      {
        StringBuilder* temp_sb;
        if (0 == active_event->getArgAs(&temp_sb)) {
          if (getVerbosity() > 3) local_log.concatf("We about to print %d bytes to the host.\n", temp_sb->length());
          printToHost(temp_sb);
          //active_event->clearArgs();
        }
        else {
          idleService();
        }
        return_value++;
      }
      break;

    case MANUVR_MSG_BT_EXITED_CMD_MODE:
    case MANUVR_MSG_BT_QUEUE_READY:
      idleService();
      return_value++;
      break;

    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}



void RNBase::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = 0;
  if (*(str) != 0) {
    temp_byte = atoi((char*) str+1);
  }

  switch (*(str)) {
    case 'x':   // Purge the queue.
      while (work_queue.hasNext()) {
        reclaimPreallocation(work_queue.dequeue());
      }
      if (NULL != current_work_item) {
        reclaimPreallocation(current_work_item);
        current_work_item = NULL;
      }
      break;
    case 'm':
      {
        int tmp_str_len = strlen((char*) str+1);
        if (tmp_str_len > 0) {
          *((char*) str+tmp_str_len) = 0x00; // Careful... tricky...
          StringBuilder *temp = new StringBuilder((char*) str+1);
          local_log.concatf("Sending \"%s\" to RN42.\n", ((char*) str+1));
          //temp->concat("\r\n");
          printToHost(temp);
        }
      }
      break;

    case 'M':
      {
        int tmp_str_len = strlen((char*) str+1);
        if (tmp_str_len > 0) {
          *((char*) str+tmp_str_len) = 0x00; // Careful... tricky...
          StringBuilder *temp = new StringBuilder((char*) str+1);
          temp->concat("\r\n");
          local_log.concatf("Sending command \"%s\" to RN42.\n", temp->string());
          sendGeneralCommand(temp);
        }
      }
      break;

    case 'h':
      switch (temp_byte) {
        case 0:   // Send reboot command
          sendRebootCommand();
          break;
        case 1:   // Set module in SPP mode.
          setSPPMode();
          break;
        case 2:   // Set module in HID mode.
          setHIDMode();
          break;

        case 3:   // Show firmware revision.
          sendGeneralCommand(RNBASE_CMD_GET_FIRMWARE_REV);
          break;
        case 4:
          sendGeneralCommand(RNBASE_CMD_CONNECT_STATUS);
          break;
        case 5:
          sendGeneralCommand(RNBASE_CMD_HELP);
          break;
        case 6:
          sendGeneralCommand(RNBASE_CMD_REMOTE_MODEM_STS);
          break;
        case 7:
          sendGeneralCommand(RNBASE_CMD_DEV_SCAN);
          break;
        case 8:
          set_bitrate(0);
          break;
        case 9:
          sendGeneralCommand("U,921K,N\r\n");
          break;

        case 200:
          setDevName((char*) "ManuDelIon");
          break;
        default:
          break;
      }
      break;

    case 'u':
      local_log.concatf("idleService() returns %d\n", idleService());
      break;
    case 'N':
      temp_byte = atoi((char*) str+1);
      local_log.concatf("master_mode(%s)\n", (temp_byte > 0) ? "true":"false");
      master_mode(temp_byte > 0);
      break;
    case 'R':
      temp_byte = atoi((char*) str+1);
      local_log.concatf("force_9600_mode(%s)\n", (temp_byte > 0) ? "true":"false");
      force_9600_mode(temp_byte > 0);
      break;

    default:
      ManuvrXport::procDirectDebugInstruction(input);
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
}
