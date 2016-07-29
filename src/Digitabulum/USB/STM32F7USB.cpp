/*
File:   STM32F7USB.cpp
Author: J. Ian Lindsay
Date:   2016.07.29

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


Transport driver for the STM32F7 USB peripheral.
*/

#if defined(STM32F7XX) | defined(STM32F746xx)

#if defined(ENABLE_USB_VCP)
  #include "tm_stm32_usb_device.h"
  #include "tm_stm32_usb_device_cdc.h"
#endif

#include "STM32F7USB.h"
#include "FirmwareDefs.h"

#include <Kernel.h>
#include <Platform/Platform.h>

#include <cstdio>
#include <stdlib.h>
#include <unistd.h>



/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/
StringBuilder _accumulator;  // TODO: Should be a class member. Tired...

extern "C" {
  static char _cmd_buf[MANUVR_USB_BUF_SIZE];
  static int  _cmd_buf_ptr = 0;

  static volatile bool _tx_in_progress = false;
  static volatile bool _rx_ready       = false;


  void VCP_Rx_Notify(uint8_t*, int) {
    _rx_ready = true;
  }

  void VCP_Tx_Complete() {
    _tx_in_progress = false;
    if (_accumulator.length()) {
      char* working_chunk = _accumulator.position(0);
      if (((STM32F7USB*) STM32F7USB::INSTANCE)->write_port((uint8_t*) working_chunk, strlen(working_chunk))) {
        _accumulator.drop_position(0);
      }
    }
  }
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
volatile STM32F7USB* STM32F7USB::INSTANCE = NULL;


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/
/**
* Constructor.
*
*/
STM32F7USB::STM32F7USB() : ManuvrXport() {
  __class_initializer();
  _xport_mtu = MANUVR_USB_BUF_SIZE;
}

/**
* Destructor
*/
STM32F7USB::~STM32F7USB() {
  __kernel->unsubscribe(this);
  _accumulator.clear();
}

/**
* This is here for compatibility with C++ standards that do not allow for definition and declaration
*   in the header file. Takes no parameters, and returns nothing.
*/
void STM32F7USB::__class_initializer() {
  for (unsigned int i = 0; i < _xport_mtu; i++) *(_cmd_buf + i) = '\0';
  INSTANCE = this;

  // We are the software nearest to the counterparty, and we do not
  //   allocate buffers we send.
  setNear(this, MEM_MGMT_RESPONSIBLE_BEARER);

  // Build some pre-formed Events.
  read_abort_event.repurpose(MANUVR_MSG_XPORT_QUEUE_RDY);
  read_abort_event.isManaged(true);
  read_abort_event.specific_target = (EventReceiver*) this;
  read_abort_event.originator      = (EventReceiver*) this;
  read_abort_event.priority        = 5;
  read_abort_event.addArg(xport_id);  // Add our assigned transport ID to our pre-baked argument.
}


/*******************************************************************************
*  _       _   _        _
* |_)    _|_ _|_ _  ._ |_) o ._   _
* |_) |_| |   | (/_ |  |   | |_) (/_
*                            |
* Overrides and addendums to BufferPipe.
*******************************************************************************/
/**
* Log has reached the end of its journey. This class will render it to the user.
*
* @param  buf    A pointer to the buffer.
* @param  len    How long the buffer is.
* @param  mm     A declaration of memory-management responsibility.
* @return A declaration of memory-management responsibility.
*/
int8_t STM32F7USB::toCounterparty(StringBuilder* buf, int8_t mm) {
  _accumulator.concatHandoff(buf);
  if (!_tx_in_progress) {
    char* working_chunk = _accumulator.position(0);
    if (write_port((uint8_t*) working_chunk, strlen(working_chunk))) {
      _accumulator.drop_position(0);
    }
  }
  return MEM_MGMT_RESPONSIBLE_BEARER;  // We took the buffer.
}



/**
* Log has reached the end of its journey. This class will render it to the user.
*
* @param  buf    A pointer to the buffer.
* @param  len    How long the buffer is.
* @param  mm     A declaration of memory-management responsibility.
* @return A declaration of memory-management responsibility.
*/
int8_t STM32F7USB::toCounterparty(uint8_t* buf, unsigned int len, int8_t mm) {
  if (!_tx_in_progress) {
    char* working_chunk = (char*) buf;
    if (_accumulator.length()) {
      _accumulator.concat(buf, len);
      working_chunk = _accumulator.position(0);
    }

    if (write_port((uint8_t*) working_chunk, strlen(working_chunk))) {
      if (_accumulator.length()) {
        // TODO: Ugly ugly ugly...
        _accumulator.drop_position(0);
      }
    }
  }
  else {
    _accumulator.concat(buf, len);
  }
  return MEM_MGMT_RESPONSIBLE_BEARER;  // We took the buffer.
}

/**
* The buffer contains keyboard input.
*
* @param  buf    A pointer to the buffer.
* @param  len    How long the buffer is.
* @param  mm     A declaration of memory-management responsibility.
* @return A declaration of memory-management responsibility.
*/
int8_t STM32F7USB::fromCounterparty(uint8_t* buf, unsigned int len, int8_t mm) {
  switch (mm) {
    case MEM_MGMT_RESPONSIBLE_CALLER:
      // NOTE: No break. This might be construed as a way of saying CREATOR.
    case MEM_MGMT_RESPONSIBLE_CREATOR:
      /* The system that allocated this buffer either...
          a) Did so with the intention that it never be free'd, or...
          b) Has a means of discovering when it is safe to free.  */
      if (haveFar()) {
        return _far->fromCounterparty(buf, len, mm);
      }
      else {
        return MEM_MGMT_RESPONSIBLE_BEARER;   // We take responsibility.
      }

    case MEM_MGMT_RESPONSIBLE_BEARER:
      /* We are now the bearer. That means that by returning non-failure, the
          caller will expect _us_ to manage this memory.  */
      if (haveFar()) {
        /* We are not the transport driver, and we do no transformation. */
        return _far->fromCounterparty(buf, len, mm);
      }
      else {
        return MEM_MGMT_RESPONSIBLE_BEARER;   // We take responsibility.
      }

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

int8_t STM32F7USB::connect() {
  // STM32F7USB, if instantiated, is ALWAYS connected.
  connected(true);
  return 0;
}

int8_t STM32F7USB::disconnect() {
  // Perhaps this makes sense in some context. But I cannot imagine it.
  //ManuvrXport::disconnect();
  return 0;
}


int8_t STM32F7USB::listen() {
  // STM32F7USB, if instantiated, is ALWAYS listening.
  listening(true);
  return 0;
}

// TODO: Perhaps reset the terminal?
int8_t STM32F7USB::reset() {
  #if defined(__MANUVR_DEBUG)
    if (getVerbosity() > 3) local_log.concatf("STM32F7USB initialized.\n");
  #endif
  initialized(true);
  listening(true);
  connected(true);

  if (local_log.length() > 0) Kernel::log(&local_log);
  return 0;
}

/**
* Read input from local keyboard.
*/
int8_t STM32F7USB::read_port() {
	char ch;
  int read_len = 0;

	/* Check if we are ready to use, if drivers are OK installed on computer */
	if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) {
		/* We are ready */
		/* Check if anything received */
		while (TM_USBD_CDC_Getc(TM_USB_FS, &ch)) {
      read_len++;

      if ((ch == '\r') || (ch == '\n') || (ch == '\0')) {
        TM_USBD_CDC_Putc(TM_USB_FS, '\n');  // Local echo
        fromCounterparty((uint8_t*)_cmd_buf, read_len, MEM_MGMT_RESPONSIBLE_CREATOR);
        _cmd_buf_ptr = 0;
        for (int i = 0; i < MANUVR_USB_BUF_SIZE; i++) *(_cmd_buf + i) = '\0';
      }
      else {
        _cmd_buf[_cmd_buf_ptr++] = ch;
      }
		}
	}
  else {
		/* We are not ready */
	}

  if (local_log.length() > 0) Kernel::log(&local_log);
  return read_len;
}


bool STM32F7USB::write_port(uint8_t* out, int out_len) {
  if (!_tx_in_progress) {
    if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) {
      TM_USBD_CDC_Puts(TM_USB_FS, (char*) out);
      _tx_in_progress = true;
    	TM_USBD_CDC_Process(TM_USB_FS);
      bytes_sent += out_len;
    }
  }
  return false;
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
const char* STM32F7USB::getReceiverName() {  return "USB";  }


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void STM32F7USB::printDebug(StringBuilder *temp) {
  if (temp == NULL) return;
  ManuvrXport::printDebug(temp);
  temp->concatf("-- Class size      %d\n",     sizeof(STM32F7USB));
}


/**
* There is a NULL-check performed upstream for the scheduler member. So no need
*   to do it again here.
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t STM32F7USB::bootComplete() {
  EventReceiver::bootComplete();

  // Tolerate 30ms of latency on the line before flushing the buffer.
  read_abort_event.alterScheduleRecurrence(0);
  read_abort_event.alterSchedulePeriod(30);
  read_abort_event.autoClear(false);
  read_abort_event.enableSchedule(false);
  read_abort_event.enableSchedule(false);

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
int8_t STM32F7USB::callback_proc(ManuvrRunnable *event) {
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


int8_t STM32F7USB::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;

  switch (active_event->event_code) {
    default:
      return_value += ManuvrXport::notify(active_event);
      break;
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}

#endif  // STM32F7XX | STM32F746xx
