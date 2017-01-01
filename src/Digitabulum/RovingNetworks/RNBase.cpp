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

//#include <stm32f7xx_hal_dma.h>
#include "RNBase.h"
#include <XenoSession/XenoSession.h>
#include <Drivers/BusQueue/BusQueue.h>


/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/

#if defined(__BUILD_HAS_THREADS)
  // Threaded platforms will need this to compensate for a loss of ISR.
  extern void* xport_read_handler(void* active_xport);

#endif


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
volatile RNBase* RNBase::INSTANCE = nullptr;

BTQueuedOperation  RNBase::__prealloc_pool[PREALLOCATED_BT_Q_OPS];
//template<> PriorityQueue<BTQueuedOperation*> BusAdapter<BTQueuedOperation>::preallocated;

// Messages that are specific to Digitabulum.
const MessageTypeDef rn_module_message_defs[] = {
  {  MANUVR_MSG_BT_EXIT_RESET    , 0x0000,    "RN_RESET"    , ManuvrMsg::MSG_ARGS_NONE }, //
  {  MANUVR_MSG_BT_EXPIRE_LOCKOUT, 0x0000,    "RN_LOCK_LIFT", ManuvrMsg::MSG_ARGS_NONE }, //
};


/**
* Reclaims the given BTQueuedOperation so its memory can be re-used.
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
  uintptr_t obj_addr = ((uintptr_t) obj);
  uintptr_t pre_min  = ((uintptr_t) INSTANCE->__prealloc_pool);
  uintptr_t pre_max  = pre_min + (sizeof(BTQueuedOperation) * PREALLOCATED_BT_Q_OPS);

  if ((obj_addr < pre_max) && (obj_addr >= pre_min)) {
    // If we are in this block, it means obj was preallocated. wipe and reclaim it.
    BusAdapter::return_op_to_pool(obj);
  }
  else {
    // We were created because our prealloc was starved. we are therefore a transient heap object.
    _heap_frees++;
    delete obj;
  }
}


/*******************************************************************************
* Contain callback soup here, pl0x...
*******************************************************************************/

/* Instance */
void RNBase::start_lockout(uint32_t milliseconds) {
  if (INSTANCE == nullptr) return;

  ManuvrMsg* bt_lo_expiry = Kernel::returnEvent(MANUVR_MSG_BT_EXPIRE_LOCKOUT);
  bt_lo_expiry->specific_target = (EventReceiver*) this;
  bt_lo_expiry->alterSchedulePeriod(milliseconds);
  bt_lo_expiry->autoClear(true);
  bt_lo_expiry->enableSchedule(true);
  bt_lo_expiry->alterScheduleRecurrence(0);
  platform.kernel()->addSchedule(bt_lo_expiry);
  _er_set_flag(RNBASE_FLAG_LOCK_OUT);
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

RNBase::RNBase(const char* nom, RNPins* pins) : ManuvrXport(nom), BusAdapter(RNBASE_MAX_BT_Q_DEPTH) {
  set_xport_state(MANUVR_XPORT_FLAG_STREAM_ORIENTED);
  //BTQueuedOperation::buildDMAMembers();
  if (nullptr == INSTANCE) {
    INSTANCE = this;
    ManuvrMsg::registerMessages(
      rn_module_message_defs,
      sizeof(rn_module_message_defs) / sizeof(MessageTypeDef)
    );
  }

  _reset_pin = pins->reset;
  setPin(_reset_pin, false);  // Park the module in reset state.

  /* Populate all the static preallocation slots for messages. */
  for (uint16_t i = 0; i < PREALLOCATED_BT_Q_OPS; i++) {
    __prealloc_pool[i].wipe();
    preallocated.insert(&__prealloc_pool[i]);
  }

  // Clear all the flags.
  _er_clear_flag(RNBASE_FLAG_LOCK_OUT | RNBASE_FLAG_CMD_MODE);
  _er_clear_flag(RNBASE_FLAG_CMD_PEND);

  //connected(GPIOB->IDR & GPIO_PIN_10);
}


RNBase::~RNBase() {
  // TODO:
  // Should do?
  // Will never be torn down, I'd think.... Maybe in low power modes?
  // Serialize the essentials and pack up?
  setPin(_reset_pin, false);  // Park the module in reset state.
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
        insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, buf);
      }
      return mm;

    case MEM_MGMT_RESPONSIBLE_BEARER:
      /* We are now the bearer. That means that by returning non-failure, the
          caller will expect _us_ to manage this memory.  */
      // TODO: Freeing the buffer?
      {
        insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, buf);
      }
      return mm;

    default:
      /* This is more ambiguity than we are willing to bear... */
      return MEM_MGMT_RESPONSIBLE_ERROR;
  }
  return MEM_MGMT_RESPONSIBLE_ERROR;
}



int8_t RNBase::bus_init() {
  return 0;
}

int8_t RNBase::bus_deinit() {
  return 0;
}


/**
* Return a vacant SPIBusOp to the caller, allocating if necessary.
*
* @param  _op   The desired bus operation.
* @param  _req  The device pointer that is requesting the job.
* @return an RNBase to be used. Only NULL if out-of-mem.
*/
BTQueuedOperation* RNBase::new_op(BusOpcode _op, BusOpCallback* _req) {
  BTQueuedOperation* return_value = BusAdapter::new_op();
  return_value->set_opcode(_op);
  return_value->callback = _req;
  return return_value;
}


/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return SPI_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t RNBase::io_op_callback(BusOp* _op) {
  BTQueuedOperation* op = (BTQueuedOperation*) _op;
  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (getVerbosity() > 2) {
    local_log.concatf("Probably shouldn't be in the default callback case...\n");
    op->printDebug(&local_log);
  }

  flushLocalLog();
  return 0;
}


/*
* This is the function that should be called to queue-up a bus operation.
* It may or may not be started immediately.
*/
int8_t RNBase::queue_io_job(BusOp* op) {
  BTQueuedOperation* nu = (BTQueuedOperation*) op;
	if (current_job) {
		// Something is already going on with the bus. Queue...
		work_queue.insert(nu);
	}
	else {
		// Bus is idle. Put this work item in the active slot and start the bus operations...
		current_job = nu;
	  nu->begin();
	}
	return 0;
}


///*
//* This function needs to be called to move the queue forward.
//*/
//int8_t RNBase::advance_work_queue() {
//	if (current_job) {
//		if (current_job->isComplete()) {
//			if (current_job->hasFault()) {
//			  #ifdef __MANUVR_DEBUG
//			  if (getVerbosity() > 3) {
//          local_log.concatf("Destroying failed job.\n");
//          if (getVerbosity() > 4) current_job->printDebug(&local_log);
//        }
//			  #endif
//			}
//
//			// Hand this completed operation off to the class that requested it. That class will
//			//   take what it wants from the buffer and, when we return to execution here, we will
//			//   be at liberty to clean the operation up.
//			if (current_job->callback) {
//				// TODO: need some minor reorg to make this not so obtuse...
//				current_job->callback->io_op_callback(current_job);
//			}
//
//			delete current_job;
//			current_job = work_queue.dequeue();
//		}
//	}
//	else {
//		// If there is nothing presently being serviced, we should promote an operation from the
//		//   queue into the active slot and initiate it in the block below.
//		current_job = work_queue.dequeue();
//	}
//
//	if (current_job) {
//		if (!current_job->has_bus_control()) {
//			current_job->begin();
//		}
//	}
//
//	flushLocalLog();
//  return 0;
//}


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
  if (current_job) {
    reclaimPreallocation(current_job);
    current_job = nullptr;
  }

  _er_set_flag(RNBASE_FLAG_LOCK_OUT);
  _er_clear_flag(RNBASE_FLAG_CMD_PEND);

  //connected(0 != HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10));
  _queue_floods = 0;

  // Used to disassert the reset line.  TODO: Need a cleaner way to accomplish this...
  ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_BT_EXIT_RESET);
  event->setOriginator((EventReceiver*) this);
  event->specific_target = (EventReceiver*) this;
  event->alterScheduleRecurrence(1);
  event->alterSchedulePeriod(510);
  event->autoClear(true);
  event->enableSchedule(true);
  platform.kernel()->addSchedule(event);
  return 0;
}


bool RNBase::write_port(unsigned char* out, int out_len) {
  return true;
}


int8_t RNBase::read_port() {
  return 0;
}




/*******************************************************************************
* Command macros
*******************************************************************************/

/**
* Send the given string to the module as a command.
*
* @param cmd  A pointer to the string to be sent to the module.
*/
void RNBase::sendGeneralCommand(StringBuilder *cmd) {
  #ifdef __MANUVR_DEBUG
    if (getVerbosity() > 5) {
      local_log.concatf("Sending command %s.\n", cmd->string());
      Kernel::log(&local_log);
    }
  #endif
  enterCommandMode();
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, cmd);
  exitCommandMode();
}


/**
* Send the given string to the module as a command.
*
* @param cmd  A pointer to the string to be sent to the module.
*/
void RNBase::sendGeneralCommand(const char *cmd) {
  StringBuilder temp(cmd);
  sendGeneralCommand(&temp);
}


/**
* Put the module into SPP mode. This setting persists across runtimes.
*/
void RNBase::setSPPMode(void) {
  sendGeneralCommand(RNBASE_PROTO_SPP);
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
  StringBuilder temp(RNBASE_CMD_CHANGE_NAME);
  if (strlen(nu_name) > 20) {
    // Max length for this field is 20 bytes. Truncate if necessary.
    *(nu_name + 19) = 0;
  }
  temp.concatf("%s\r\n", nu_name);
  sendGeneralCommand(&temp);
  start_lockout(1100);
}


/**
* Sends the soft-reboot command to the module.
*/
void RNBase::sendRebootCommand(void) {
  enterCommandMode();
  StringBuilder temp(RNBASE_CMD_REBOOT);
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, &temp);
  start_lockout(3000);
}


/**
* Send a break sequence to the module.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::sendBreak() {
  StringBuilder temp("\r\n");
  insert_into_work_queue(BusOpcode::TX_CMD, &temp);
  return 0;
}


/**
* Put the module into command mode.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::enterCommandMode() {
  StringBuilder temp(RNBASE_MODE_COMMAND);
  insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, &temp);
  _er_set_flag(RNBASE_FLAG_CMD_PEND, true);
  //start_lockout(1100);
  return 0;
}


/**
* Take the module out of command mode.
*
* @return 0 on success. Non-zero on failure.
*/
int8_t RNBase::exitCommandMode() {
    StringBuilder temp(RNBASE_MODE_EXITCOMMAND);
    insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, &temp);
    start_lockout(1500);
    return 0;
}



void RNBase::setAutoconnect(bool autocon) {
  if (autoConnect() ^ autocon) {
    autoConnect(autocon);
    enterCommandMode();
    StringBuilder temp(autocon ? RNBASE_MODE_AUTOCONNECT : RNBASE_MODE_MANUCONNECT);
    insert_into_work_queue(BusOpcode::TX_CMD_WAIT_RX, &temp);
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



/*******************************************************************************
* Asynchronous support fxns
*******************************************************************************/

/*
* This gets called to service queue events.
*/
int8_t RNBase::advance_work_queue() {
  read_port();
  /* Prepare to generate bugs....
  I normally wouldn't write something this way. The idea is to force an exit
  through one of the states. We don't know going into this call how many states
  we will traverse before our work is done.
  */
  while (true) {
    if (current_job) {
      if (current_job->isComplete()) {   // Is it completed?
        if (BusOpcode::RX == current_job->get_opcode()) {
          // This is meant for the session.
          if (haveFar()) {
            #ifdef __MANUVR_DEBUG
              if (getVerbosity() > 4) Kernel::log("RNBase sending RX to application...\n");
            #endif
          }
        }
        reclaimPreallocation(current_job);
        current_job = nullptr;
      }
      else if (current_job->inProgress()) {   // Is it initiated?
        // We'd probably be interfering with somehting if we do anything. So do nothing.
        return -2;
      }
      else if (!_er_flag(RNBASE_FLAG_LOCK_OUT)) {
        // If it isn't completed or initiated, and we aren't locked out, let's kick it off...
        switch (current_job->get_opcode()) {
          case BusOpcode::TX_CMD_WAIT_RX:
          case BusOpcode::TX_CMD:
            //_er_set_flag(RNBASE_FLAG_CMD_PEND, !_er_flag(RNBASE_FLAG_CMD_MODE));
            current_job->begin();
            write_port(current_job->buf, current_job->buf_len);
            break;
          case BusOpcode::TX:
            if (connected()) {
              current_job->begin();
              write_port(current_job->buf, current_job->buf_len);
            }
            else {
              // Could have started a counterparty-bound message and didn't because: NO COUNTERPARTY
              return -3;
            }
            break;
          default:
            #ifdef __MANUVR_DEBUG
              if (getVerbosity() > 1) Kernel::log("advance_work_queue(): We should not be here.\n");
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
      current_job = work_queue.dequeue();
      if (nullptr == current_job) {
        return 0;  // Nothing more to process.
      }
    }
  }
  return 0;
}



/**
* Put the module into master mode.
* This setting persists across runtimes.
*
* @param force_master_mode Set to true to make the module behave as if it were a host.
*/
void RNBase::master_mode(bool force_master_mode) {
  if (force_master_mode) {
  }
  else {
  }
  reset();
}


// Call to displace or abort a transaction to a host that disconnected.
// Return 0 indicates caller should burn or wait.
// Return 1 indicates we recycled.
int8_t RNBase::burn_or_recycle_current() {
  if ((nullptr != current_job) && (current_job->get_opcode() == BusOpcode::TX)) {
    // If there is something in-process and it is meant for a counterparty....
    if (! current_job->inProgress()) {
      // If there is an un-initiated counterparty transaction waiting, displace it.
      // Re-queue with retry priority.
      work_queue.insert(current_job, RNBASE_RETRY_PRIORITY);
      current_job->markQueued();
      current_job = nullptr;   // Let the downstream code do its job...
      return 1;
    }
    else {
      return 0;
    }
  }
  return -1;  // Question didn't make sense.
}



uint32_t RNBase::insert_into_work_queue(BusOpcode opcode, StringBuilder* data) {
  BTQueuedOperation *nu = BusAdapter::new_op();
  nu->set_data(opcode, data);

  //BTQueuedOperation *nu = new BTQueuedOperation(opcode, data->string(), data->length());  // TODO: Preallocate these!
  uint32_t return_value = 0;

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

      if ((nullptr != current_job) && (current_job->get_opcode() == BusOpcode::TX_CMD)) {
        if (! current_job->inProgress()) {
          // If there is an un-initiated counterparty transaction waiting, displace it.
          // Re-queue with retry priority.
          work_queue.insert(current_job, RNBASE_RETRY_PRIORITY);
          current_job->markQueued();
          current_job = nullptr;   // Let the downstream code do its job...
        }
      }
      priority = RNBASE_CMD_PRIORITY;
      work_queue.insert(nu, priority);
      nu->markQueued();
      break;
    case BusOpcode::TX:
      if (nullptr == current_job) {
        current_job = nu;
        current_job->begin();
        write_port(current_job->buf, current_job->buf_len);
      }
      else if (roomInQueue()) {
        work_queue.insert(nu, priority);
        nu->markQueued();
      }
      else {
        return_value = 0;
        #ifdef __MANUVR_DEBUG
          if (getVerbosity() > 5) Kernel::log("Dropping BT send. Queue flooded.\n");
        #endif
        _queue_floods++;
        reclaimPreallocation(nu);
      }
      break;

    default:
      Kernel::log("RNBase: Unknown opcode.\n");
      break;
  }
  read_abort_event.fireNow();

  return return_value;
}


/*******************************************************************************
* Private class members...                                                     *
*******************************************************************************/

void RNBase::process_connection_change(bool conn) {
  connected(conn);
  if (!conn) {
    #ifdef __MANUVR_DEBUG
      if (getVerbosity() > 3) Kernel::log("We lost connection...\n");
    #endif
    burn_or_recycle_current();
    // Purge the queue.
    for (int i = 0; i < work_queue.size(); i++) {
      BTQueuedOperation* current = work_queue.dequeue();
      if (BusOpcode::TX == current->get_opcode()) {
        current->abort(XferFault::QUEUE_FLUSH);
        reclaimPreallocation(current);
      }
      else {
        work_queue.insert(current);
        current->markQueued();
      }
    }
  }
  else {
    advance_work_queue();
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
*/
size_t RNBase::feed_rx_buffer(unsigned char *nu, size_t len) {
  size_t remaining_len = len;
  while (remaining_len > 0) {
    if (_er_flag(RNBASE_FLAG_CMD_PEND)) {
      // Reasoning... If the module is in RNBASE_FLAG_CMD_PEND, it can only be $$$.
      if (current_job) {
        if (BusOpcode::TX_CMD_WAIT_RX == current_job->get_opcode()) {
          size_t temp_len = strict_min((uint32_t) strlen(_cmd_return_str), (uint32_t) remaining_len);
          if ((remaining_len > temp_len) && (0 != memcmp(nu, _cmd_return_str, temp_len))) {
            // This is the command prompt.
            _er_set_flag(RNBASE_FLAG_CMD_PEND, false);
            _er_set_flag(RNBASE_FLAG_CMD_MODE, true);
            remaining_len -= temp_len;
            current_job->markComplete();
          }
        }
      }

      if (_er_flag(RNBASE_FLAG_CMD_PEND)) {
        Kernel::log("RNBase: Something has gone bad wrong in feed_rx_buffer().\n");
        return (len - remaining_len);
      }
    }
    else if (_er_flag(RNBASE_FLAG_CMD_MODE)) {
      uint8_t* end_idx = (uint8_t*) memchr(nu, '\n', remaining_len);
      if (nullptr == end_idx) {
        return (len - remaining_len);
      }
      else {
        remaining_len = end_idx - nu;
      }

      if (current_job) {
        if (BusOpcode::TX_CMD_WAIT_RX == current_job->get_opcode()) {
          size_t temp_len = strict_min((uint32_t) strlen(_cmd_exit_str), (uint32_t) remaining_len);
          if ((remaining_len > temp_len) && (0 != memcmp(nu, _cmd_exit_str, temp_len))) {
            _er_set_flag(RNBASE_FLAG_CMD_MODE, true);
            remaining_len -= temp_len;
            current_job->markComplete();
          }
        }
        else {
          #ifdef __MANUVR_DEBUG
          if (getVerbosity() > 2) {
            Kernel::log("RNBase: In command_mode (or pending), but have wrong opcode for work_queue item.\n\t");
            for (size_t i = 0; i < remaining_len; i++) {
              local_log.concatf("0x%02x ", *(nu + i));
            }
            local_log.concat("\n\n");
          }
          #endif
          current_job->abort(XferFault::ILLEGAL_STATE);
          return (len - remaining_len);
        }
      }
      else {
        #ifdef __MANUVR_DEBUG
        // TODO: I have _never_ seen this happen.
        if (getVerbosity() > 2) {
          Kernel::log("Don't know what to do with data. In command_mode (or pending), but have no work_queue item to feed.\n\t");
          for (size_t i = 0; i < remaining_len; i++) {
            local_log.concatf("0x%02x ", *(nu + i));
          }
          local_log.concat("\n\n");
        }
        #endif
        return (len - remaining_len);
      }
    }
    else {   // This data must be meant for a session... (So we hope)
      // TODO: This is the most-likely case. Promote to top.
      BufferPipe::fromCounterparty(nu, remaining_len, MEM_MGMT_RESPONSIBLE_BEARER);
      remaining_len = 0;
    }
    flushLocalLog();
  }
  return (len - remaining_len);
}



volatile void RNBase::isr_bt_queue_ready() {
  ((RNBase*)INSTANCE)->read_abort_event.fireNow();
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
* This is called when the kernel attaches the module.
* This is the first time the class can be expected to have kernel access.
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t RNBase::attached() {
  if (EventReceiver::attached()) {
    // Build some pre-formed Events.
    read_abort_event.repurpose(MANUVR_MSG_XPORT_QUEUE_RDY, (EventReceiver*) this);
    read_abort_event.incRefs();
    read_abort_event.specific_target = (EventReceiver*) this;
    read_abort_event.priority(2);
    // Tolerate 30ms of latency on the line before flushing the buffer.
    read_abort_event.alterSchedulePeriod(CHARACTER_CHRONOLOGICAL_BREAK);
    read_abort_event.autoClear(false);
    reset();

    read_abort_event.alterScheduleRecurrence(-1);
    platform.kernel()->addSchedule(&read_abort_event);

    gpioSetup();
    force_9600_mode(false);   // Init the UART.
    return 1;
  }
  return 0;
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
int8_t RNBase::callback_proc(ManuvrMsg* event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = (0 == event->refCount()) ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    case MANUVR_MSG_BT_EXIT_RESET:
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
void RNBase::printQueue(StringBuilder* temp) {
  BTQueuedOperation* q_item = nullptr;

  if (current_job) {
    temp->concat("\n-- In working slot:\n");
    current_job->printDebug(temp);
  }

  if (getVerbosity() > 3) {
    if (work_queue.hasNext()) {
      int q_read_count = strict_min((int16_t) work_queue.size(), (int16_t) RNBASE_MAX_QUEUE_PRINT);
      temp->concatf("\n-- Queue Listing (top %d of %d)", q_read_count, work_queue.size());
      for (int i = 0; i < q_read_count; i++) {
        q_item = work_queue.get(i);
        q_item->printDebug(temp);
        temp->concat("\n");
      }
    }
    else {
      temp->concat("-- No Queue\n\n");
    }
  }
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void RNBase::printDebug(StringBuilder* output) {
  ManuvrXport::printDebug(output);
  BusAdapter::printAdapter((BusAdapter*)this, output);

  output->concatf("-- bitrate:        %u\n", configured_bitrate);
  output->concatf("-- lockout_active: %s\n", (_er_flag(RNBASE_FLAG_LOCK_OUT) ? "yes" : "no"));

  const char* cmd_mode_str;
  if (_er_flag(RNBASE_FLAG_CMD_MODE))      cmd_mode_str = "CMD";
  else if (_er_flag(RNBASE_FLAG_CMD_PEND)) cmd_mode_str = "CMD_PENDING";
  else cmd_mode_str = "DATA";
  output->concatf("-- cmd mode:       %s\n", cmd_mode_str);
  BusAdapter::printWorkQueue((BusAdapter*)this, output, RNBASE_MAX_QUEUE_PRINT);
}



int8_t RNBase::notify(ManuvrMsg* active_event) {
  int8_t return_value = 0;
  switch (active_event->eventCode()) {
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
          advance_work_queue();
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

    case MANUVR_MSG_BT_EXPIRE_LOCKOUT:
      if (getVerbosity() > 5) Kernel::log("RN lockout expired.\n");
      _er_clear_flag(RNBASE_FLAG_LOCK_OUT);
      advance_work_queue();
      return_value++;
      break;

    case MANUVR_MSG_BT_ENTERED_CMD_MODE:
    case MANUVR_MSG_BT_EXITED_CMD_MODE:
    case MANUVR_MSG_XPORT_QUEUE_RDY:
      advance_work_queue();
      return_value++;
      break;

    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  flushLocalLog();
  return return_value;
}


#if defined(MANUVR_CONSOLE_SUPPORT)
void RNBase::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = 0;
  if (*(str) != 0) {
    temp_byte = atoi((char*) str+1);
  }

  switch (*(str)) {
    case 'i':        // Readback test
      switch (temp_byte) {
        case 1:
          printQueue(&local_log);
          break;
        default:
          printDebug(&local_log);
          break;
      }
      break;

    case 'x':   // Purge the queue.
      while (work_queue.hasNext()) {
        reclaimPreallocation(work_queue.dequeue());
      }
      if (current_job) {
        reclaimPreallocation(current_job);
        current_job = nullptr;
      }
      break;
    case 'c':
      local_log.concat("RNBase: enterCommandMode()\n");
      enterCommandMode();
      break;
    case 'C':
      local_log.concat("RNBase: exitCommandMode()\n");
      exitCommandMode();
      break;

    case 'M':
    case 'm':
      {
        int tmp_str_len = strlen((char*) str+1);
        if (tmp_str_len > 0) {
          *((char*) str+tmp_str_len) = 0x00; // Careful... tricky...
          StringBuilder *temp = new StringBuilder((char*) str+1);
          local_log.concatf("Sending to RN: (%s)\n\"%s\"\n", ('M' == *str ? "CMD" : "HOST"), ((char*) str+1));
          temp->concat("\r\n");
          if ('M' == *str) {
            sendGeneralCommand(temp);
          }
          else {
            printToHost(temp);
          }
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
          setDevName((char*) "Digitabulum");
          break;
        default:
          break;
      }
      break;

    case 'u':
      local_log.concatf("advance_work_queue() returns %d\n", advance_work_queue());
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

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT
