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
#include <Drivers/BusQueue/BusQueue.h>


#define MAX_UART_STR_LEN 255
static volatile unsigned char uart2_received_string[MAX_UART_STR_LEN];
static volatile uint8_t uart2_rec_cnt = 0;


uint32_t read_millis_0 = 0;
uint32_t read_millis_1 = 0;

// Messages that are specific to Digitabulum.
const MessageTypeDef rn_module_message_defs[] = {
  /*
    For messages that have arguments, we have the option of defining inline lables for each parameter.
    This is advantageous for debugging and writing front-ends. We case-off here to make this choice at
    compile time.
  */
  #if defined (__ENABLE_MSG_SEMANTICS)
  {  MANUVR_MSG_BT_EXIT_RESET        , 0x000,                "RN_RESET"             , ManuvrMsg::MSG_ARGS_NONE }, //
  #else
  {  MANUVR_MSG_BT_EXIT_RESET        , 0x000,                "RN_RESET"             , ManuvrMsg::MSG_ARGS_NONE, NULL }, //
  #endif
};


/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/
/*
*
*/
void USART2_IRQHandler(void) {
}

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
* @param BTQueuedOperation* obj is the pointer to the object to be reclaimed.
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

  if (((EventReceiver*) INSTANCE)->getVerbosity() > 4) Kernel::log("oneshot_rn_reenable()\n");
  ((RNBase*) INSTANCE)->_er_clear_flag(RNBASE_FLAG_LOCK_OUT);
  ((RNBase*) INSTANCE)->idleService();
}

/* Scheduler one-shot. Re-allows modules communication. */
void oneshot_rn_reenable() {
  RNBase::expire_lockout();
}

/* Instance */
void RNBase::start_lockout(uint32_t milliseconds) {
  if (INSTANCE == NULL) return;

  // TODO: Need a cleaner way to accomplish this...
  __kernel->addSchedule(new ManuvrRunnable(milliseconds,  0, true, oneshot_rn_reenable));
  _er_set_flag(RNBASE_FLAG_LOCK_OUT);
}


/* Scheduler one-shot. Flushes the RX queue. */
void host_read_abort() {
  if (0 == read_millis_0) {   // There is nothing in the buffer. Expire.
    return;
  }
  uint32_t current_millis = millis();
  if (CHARACTER_CHRONOLOGICAL_BREAK < (std::max(current_millis, read_millis_1) - std::min(current_millis, read_millis_1))) {
    RNBase::hostRxFlush();
    read_millis_0 = 0;
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

RNBase::RNBase(uint8_t rst_pin) : ManuvrXport() {
  setReceiverName("RNBase");
  //BTQueuedOperation::buildDMAMembers();
  if (NULL == INSTANCE) {
    INSTANCE = this;
    ManuvrMsg::registerMessages(
      rn_module_message_defs,
      sizeof(rn_module_message_defs) / sizeof(MessageTypeDef)
    );
  }

  _reset_pin = rst_pin;

  /* Populate all the static preallocation slots for messages. */
  for (uint16_t i = 0; i < PREALLOCATED_BT_Q_OPS; i++) {
    __prealloc_pool[i].wipe();
    preallocated.insert(&__prealloc_pool[i]);
  }

  // Clear all the flags.
  _er_clear_flag(RNBASE_FLAG_LOCK_OUT | RNBASE_FLAG_CMD_MODE);
  _er_clear_flag(RNBASE_FLAG_CMD_PEND | RNBASE_FLAG_AUTOCONN);

  //connected(GPIOB->IDR & GPIO_PIN_10);

  //// Build some pre-formed Events.
  //read_abort_event.repurpose(MANUVR_MSG_XPORT_QUEUE_RDY);
  //read_abort_event.isManaged(true);
  //read_abort_event.specific_target = (EventReceiver*) this;
  //read_abort_event.originator      = (EventReceiver*) this;
  //read_abort_event.alterScheduleRecurrence(-1);
  //read_abort_event.alterSchedulePeriod(CHARACTER_CHRONOLOGICAL_BREAK);
  //read_abort_event.autoClear(false);
  //read_abort_event.enableSchedule(false);
  //__kernel->addSchedule(&read_abort_event);
}


RNBase::~RNBase() {
  // TODO:
  // Should do?
  // Will never be torn down, I'd think.... Maybe in low power modes?
  // Serialize the essentials and pack up?
  reset();           // we effectively park the module in reset state.
}


/**
* Setup GPIO pins and their bindings to on-chip peripherals, if required.
*/
void RNBase::gpioSetup() {
  gpioDefine(_reset_pin, OUTPUT_OD);
  setPin(_reset_pin, false);
}


/*******************************************************************************
*  _       _   _        _
* |_)    _|_ _|_ _  ._ |_) o ._   _
* |_) |_| |   | (/_ |  |   | |_) (/_
*                            |
* Overrides and addendums to BufferPipe.
*******************************************************************************/
/**
* Inward toward the transport.
*
* @param  buf    A pointer to the buffer.
* @param  len    How long the buffer is.
* @param  mm     A declaration of memory-management responsibility.
* @return A declaration of memory-management responsibility.
*/
int8_t RNBase::toCounterparty(StringBuilder* buf, int8_t mm) {
  switch (mm) {
    case MEM_MGMT_RESPONSIBLE_CALLER:
      // NOTE: No break. This might be construed as a way of saying CREATOR.
    case MEM_MGMT_RESPONSIBLE_CREATOR:
      /* The system that allocated this buffer either...
          a) Did so with the intention that it never be free'd, or...
          b) Has a means of discovering when it is safe to free.  */
      {
        StringBuilder *temp = new StringBuilder();
        temp->concatHandoff(buf);
        insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
      }
      return mm;

    case MEM_MGMT_RESPONSIBLE_BEARER:
      /* We are now the bearer. That means that by returning non-failure, the
          caller will expect _us_ to manage this memory.  */
      // TODO: Freeing the buffer?
      {
        StringBuilder *temp = new StringBuilder();
        temp->concatHandoff(buf);
        insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
      }
      return mm;

    default:
      /* This is more ambiguity than we are willing to bear... */
      return MEM_MGMT_RESPONSIBLE_ERROR;
  }
  return MEM_MGMT_RESPONSIBLE_ERROR;
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

int8_t RNBase::connect() {
  return 0;
}


int8_t RNBase::listen() {
  return 0;
}


/*
* If we call this before the boot process is complete, we will not come out of reset.
*/
int8_t RNBase::reset() {
  setPin(_reset_pin, false);     // Drive reset pin low.
  initialized(false);

  while (work_queue.hasNext()) {
    reclaimPreallocation(work_queue.dequeue());
  }
  if (NULL != current_work_item) {
    reclaimPreallocation(current_work_item);
    current_work_item = NULL;
  }

  _er_set_flag(RNBASE_FLAG_LOCK_OUT);
  _er_clear_flag(RNBASE_FLAG_CMD_PEND | RNBASE_FLAG_AUTOCONN);

  connected(0 != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10));
  queue_floods      = 0;

  tx_buf.clear();

  // Used to disassert the reset line.  TODO: Need a cleaner way to accomplish this...
  ManuvrRunnable* event = Kernel::returnEvent(MANUVR_MSG_BT_EXIT_RESET);
  event->addArg((EventReceiver*) this);
  event->originator      = (EventReceiver*) this;
  event->specific_target = (EventReceiver*) this;
  event->alterScheduleRecurrence(0);
  event->alterSchedulePeriod(510);
  event->autoClear(true);
  event->enableSchedule(true);
  __kernel->addSchedule(event);
  return 0;
}


bool RNBase::write_port(unsigned char* out, int out_len) {
  return true;
}


int8_t RNBase::read_port() {
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
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, cmd);
  exitCommandMode();
  #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 5) {
      local_log.concatf("Sent command %s.\n", cmd->string());
      Kernel::log(&local_log);
    }
  #endif
}


/**
* Send the given string to the module as a command.
*
* @param cmd  A pointer to the string to be sent to the module.
*/
void RNBase::sendGeneralCommand(const char *cmd) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(cmd);
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
  exitCommandMode();
  #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 5) {
      local_log.concatf("Sent command %s.\n", cmd);
      Kernel::log(&local_log);
    }
  #endif
}


/**
* Put the module into HID mode. This setting persists across runtimes.
*/
void RNBase::setHIDMode(void) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(RNBASE_MODE_HID);
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
  exitCommandMode();
  #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 5) Kernel::log("Tried to enter HID mode.\n");
  #endif
}


/**
* Put the module into SPP mode. This setting persists across runtimes.
*/
void RNBase::setSPPMode(void) {
  enterCommandMode();
  StringBuilder *temp = new StringBuilder(RNBASE_PROTO_SPP);
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
  exitCommandMode();
  #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 5) Kernel::log("Tried to enter SPP mode.\n");
  #endif
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
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
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
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
  start_lockout(3000);
}



/**
* Send a break sequence to the module.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::sendBreak() {
    StringBuilder *temp = new StringBuilder("\r\n");
    insert_into_work_queue(BusOpcode::TX_CMD, temp);
    return 0;
}

/**
* Put the module into command mode.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::enterCommandMode() {
    StringBuilder *temp = new StringBuilder(RNBASE_MODE_COMMAND);
    insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
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
    insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
    start_lockout(1500);
    return 0;
}



void RNBase::setAutoconnect(bool autocon) {
  if (_er_flag(RNBASE_FLAG_AUTOCONN) ^ autocon) {
    enterCommandMode();
    _er_set_flag(RNBASE_FLAG_CMD_PEND, autocon);
    StringBuilder *temp = new StringBuilder(autocon ? RNBASE_MODE_AUTOCONNECT : RNBASE_MODE_MANUCONNECT);
    insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, temp);
    #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 4) {
      local_log.concatf("Autoconnect is now %sabled.", (autocon ? "en" : "dis"));
      Kernel::log(&local_log);
    }
    #endif
    exitCommandMode();
    sendRebootCommand();
  }
  else {
    #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 4) {
      local_log.concatf("Autoconnect mode was already %sabled.", (autocon ? "en" : "dis"));
      Kernel::log(&local_log);
    }
    #endif
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
          if (haveFar()) {
            //if (current_work_item->opcode) {
              #ifdef __MANUVR_DEBUG
                if (getVerbosity() > 4) Kernel::log("RNBase About to mark message complete.\n");
              #endif
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
      else if (!_er_flag(RNBASE_FLAG_LOCK_OUT)) {
        // If it isn't completed or initiated, and we aren't locked out, let's kick it off...
        switch (current_work_item->opcode) {
          case BusOpcode::TX_CMD_WAIT_RX:
          case BusOpcode::TX_CMD:
            _er_set_flag(RNBASE_FLAG_CMD_PEND, !_er_flag(RNBASE_FLAG_CMD_MODE));
            current_work_item->begin();
            break;
          case BusOpcode::TX:
            if (connected()) {
              current_work_item->begin();
            }
            else {
              // Could have started a counterparty-bound message and didn't because: NO COUNTERPARTY
              return -3;
            }
            break;
          default:
            #ifdef __MANUVR_DEBUG
              if (getVerbosity() > 1) Kernel::log("idleService(): We should not be here (initiation block).\n");
            #endif
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
  if ((NULL != current_work_item) && (current_work_item->opcode == BusOpcode::TX)) {
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



uint32_t RNBase::insert_into_work_queue(BusOpcode opcode, StringBuilder* data) {
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
    case BusOpcode::TX_CMD_WAIT_RX:
    case BusOpcode::TX_CMD:
      if (!burn_or_recycle_current()) {
        // Burn
      }

      if ((NULL != current_work_item) && (current_work_item->opcode == BusOpcode::TX_CMD)) {
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
    case BusOpcode::TX:
      if (NULL == current_work_item) {
        current_work_item = nu;
        current_work_item->begin();
      }
      else if (work_queue.size() <= RNBASE_MAX_BT_Q_DEPTH) {
        work_queue.insert(nu, priority);
      }
      else {
        return_value = 0;
        #ifdef __MANUVR_DEBUG
          if (getVerbosity() > 3) Kernel::log("Dropping BT send. Queue too large.\n");
        #endif
        queue_floods++;
        reclaimPreallocation(nu);
      }
      break;

    default:
      Kernel::log("RNBase: Unknown opcode.\n");
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
    #ifdef __MANUVR_DEBUG
      if (((EventReceiver*) INSTANCE)->getVerbosity() > 4) Kernel::log("Flushed bytes\n");
    #endif
    uart2_rec_cnt = 0;
  }
  else {
    //Kernel::log(__PRETTY_FUNCTION__, 6, "Flushed 0 bytes.");
  }

  if (INSTANCE->current_work_item != NULL) {
    if (BusOpcode::TX_CMD_WAIT_RX == INSTANCE->current_work_item->opcode) {
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
  if (_er_flag(RNBASE_FLAG_CMD_MODE | RNBASE_FLAG_CMD_PEND)) {
    BTQueuedOperation *local_work_item = (NULL != current_work_item) ? current_work_item : work_queue.get();
    if (NULL != local_work_item) {
        switch (local_work_item->opcode) {
          case BusOpcode::TX_CMD_WAIT_RX:
            // We make sure that we tell the class if we are expecting a change in this state.
            _er_set_flag(RNBASE_FLAG_CMD_PEND, !_er_flag(RNBASE_FLAG_CMD_MODE));
            local_work_item->data.concat(nu, len);
            //output.concatf("Rx Local work item length: %d\n", local_work_item->data.length());
            break;

          case BusOpcode::TX_CMD:
          case BusOpcode::TX:
            #ifdef __MANUVR_DEBUG
            if (getVerbosity() > 2) {
              local_log.concat("Don't know what to do with data. In command_mode (or pending), but have wrong opcode for work_queue item.\n\t");
              for (int i = 0; i < len; i++) {
                local_log.concatf("0x%02x ", *(nu + i));
              }
              local_log.concat("\n\n");
            }
            #endif
            break;
          default:
            #ifdef __MANUVR_DEBUG
            if (getVerbosity() > 2) local_log.concat("RNBase: Unknown opcode.\n");
            #endif
            break;
        }
    }
    else {
      #ifdef __MANUVR_DEBUG
      if (getVerbosity() > 2) {
        local_log.concatf("Don't know what to do with data. In command_mode (or pending), but have no work_queue item to feed.\n\t");
        for (int i = 0; i < len; i++) {
          local_log.concatf("0x%02x ", *(nu + i));
        }
        local_log.concat("\n\n");
      }
      #endif
    }
  }
  else {   // This data must be meant for a session... (So we hope)
    BufferPipe::fromCounterparty(nu, len, MEM_MGMT_RESPONSIBLE_BEARER);
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
}



/*
*
*/
int8_t RNBase::sendBuffer(StringBuilder* _to_send) {
  if (NULL == _to_send) return -1;
  #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 3) local_log.concatf("We about to print %d bytes to the host.\n", _to_send->length());
  #endif
  printToHost(_to_send);
  return 0;
}



/*
*
*/
volatile void RNBase::irqServiceBT_data_activity(void) {
  if (NULL == INSTANCE) return;
  // We aren't doing anything here yet.
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
    unsigned long delta = std::max(ms, (unsigned long) last_gpio_5_event) - std::min(ms, (unsigned long) last_gpio_5_event);
    if (delta < 500) {
      // Probably the 10Hz signal. Means we are in command mode.
      if (INSTANCE != NULL) {
        if (!((RNBase*) INSTANCE)->_er_flag(RNBASE_FLAG_CMD_MODE)) {
          ((RNBase*) INSTANCE)->_er_set_flag(RNBASE_FLAG_CMD_MODE);
          ((RNBase*) INSTANCE)->_er_clear_flag(RNBASE_FLAG_CMD_PEND);
          ManuvrRunnable *nu_event = Kernel::returnEvent(MANUVR_MSG_BT_ENTERED_CMD_MODE);
          Kernel::isrRaiseEvent(nu_event);
        }
      }
    }
    else if (delta < 3500) {
      // Probably the 1Hz signal. Means we are discoverable and waiting for connection.
      if (INSTANCE != NULL) {
        if (((RNBase*) INSTANCE)->_er_flag(RNBASE_FLAG_CMD_MODE)) {
          ((RNBase*) INSTANCE)->_er_clear_flag(RNBASE_FLAG_CMD_MODE | RNBASE_FLAG_CMD_PEND | RNBASE_FLAG_LOCK_OUT);
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
  last_gpio_5_event = ms;

  #ifdef __MANUVR_DEBUG
    if (((EventReceiver*) INSTANCE)->getVerbosity() > 6) {
      StringBuilder _log;
      _log.concatf("BT GPIO 5: %lu.\n", ms);
      Kernel::log(&_log);
    }
  #endif
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

  gpioSetup();
  //force_9600_mode(false);   // Init the UART.

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
  switch (event->eventCode()) {
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
  temp->concatf("-- queue_floods            %u\n--\n", (unsigned long) rejected_host_messages);

  temp->concatf("-- __prealloc_pool addres: 0x%08x\n", (uint32_t) __prealloc_pool);
  temp->concatf("-- prealloc depth:         %d\n", preallocated.size());
  temp->concatf("-- queue_floods:           %u\n", (unsigned long) queue_floods);
  temp->concatf("-- _heap_instantiations:   %u\n", (unsigned long) _heap_instantiations);
  temp->concatf("-- _heap_frees:            %u\n--\n", (unsigned long) _heap_frees);

  temp->concatf("-- bitrate to module       %d\n", configured_bitrate);
  temp->concatf("-- lockout_active          %s\n", (_er_flag(RNBASE_FLAG_LOCK_OUT) ? "yes" : "no"));
  temp->concatf("-- command mode (pend)     %s\n", (_er_flag(RNBASE_FLAG_CMD_PEND) ? "yes" : "no"));
  temp->concatf("-- command mode            %s\n", (_er_flag(RNBASE_FLAG_CMD_MODE) ? "yes" : "no"));
  temp->concatf("-- autoconnect             %s\n", (_er_flag(RNBASE_FLAG_AUTOCONN) ? "yes" : "no"));
  if (getVerbosity() > 5) {
    temp->concatf("-- last_gpio5              %u\n", last_gpio_5_event);
    temp->concatf("-- gpio5 state             %s\n\n", ((GPIOB->IDR & GPIO_PIN_10) ? "high" : "low"));
  }

  BTQueuedOperation* q_item = NULL;

  if (NULL != current_work_item) {
    temp->concat("\n-- In working slot:");
    current_work_item->printDebug(temp);
  }

  if (getVerbosity() > 3) {
    if (work_queue.hasNext()) {
      temp->concatf("\n-- Queue Listing (top %d of %d)", RNBASE_MAX_QUEUE_PRINT, work_queue.size());
      for (int i = 0; i < std::min(work_queue.size(), RNBASE_MAX_QUEUE_PRINT); i++) {
        q_item = work_queue.get(i);
        q_item->printDebug(temp);
      }
    }
    else {
      temp->concat("-- No Queue\n\n");
    }
  }
}


//int8_t teardownSession() {
//}


int8_t RNBase::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;
  switch (active_event->eventCode()) {
    case MANUVR_MSG_BT_RX_BUF_NOT_EMPTY:
      // We just received something at the BT port. Start the timeout running...
      read_abort_event.delaySchedule();
      return_value++;
      break;
    case MANUVR_MSG_BT_CONNECTION_LOST:
      connected(false);
      #ifdef __MANUVR_DEBUG
        if (getVerbosity() > 3) local_log.concat("We lost our bluetooth connection. About to tear down the session...\n");
      #endif
      burn_or_recycle_current();
      // Purge the queue.
      for (int i = 0; i < work_queue.size(); i++) {
        BTQueuedOperation* current = work_queue.dequeue();
        if (BusOpcode::TX == current->opcode) {
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
          #ifdef __MANUVR_DEBUG
            if (getVerbosity() > 3) local_log.concatf("We about to print %d bytes to the host.\n", temp_sb->length());
          #endif
          printToHost(temp_sb);
          //active_event->clearArgs();
        }
        else {
          idleService();
        }
        return_value++;
      }
      break;

    case MANUVR_MSG_BT_EXIT_RESET:
      setPin(_reset_pin, true);     // Drive reset pin high.
      start_lockout(550);   // Spec says to wait 500ms after reset.
      initialized(true);
      return_value++;
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


#if defined(__MANUVR_CONSOLE_SUPPORT)
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
          local_log.concatf("Sending \"%s\" to RN.\n", ((char*) str+1));
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
          local_log.concatf("Sending command \"%s\" to RN.\n", temp->string());
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
#endif  //__MANUVR_CONSOLE_SUPPORT
