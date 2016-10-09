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

#include "STM32F7USB.h"
#include "ManuvrConf.h"

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

extern "C" {
  StringBuilder _accumulator;  // TODO: Should be a class member. Tired...
  StringBuilder _usb_rx_buf;   // TODO: Should be a class member. Tired...

  static volatile bool _tx_in_progress = false;
  static volatile bool _rx_ready       = false;


  void VCP_Rx_Notify(uint8_t* _in_buf, int _len) {
    if (STM32F7USB::INSTANCE) {
      ((STM32F7USB*) STM32F7USB::INSTANCE)->rx_wakeup();
    }
    _rx_ready = true;
  }

  void VCP_Tx_Complete() {
    if (STM32F7USB::INSTANCE) {
      ((STM32F7USB*) STM32F7USB::INSTANCE)->tx_wakeup();
    }
    _tx_in_progress = false;
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
  INSTANCE = this;
  setReceiverName("USB");

  // /* Init USB peripheral as VCP */
  // TM_USBD_CDC_Init(TM_USB_FS);
  // TM_USBD_Start(TM_USB_FS);

  _bp_set_flag(BPIPE_FLAG_IS_BUFFERED, true);

  // We are the software nearest to the counterparty, and we do not
  //   allocate buffers we send.
  setNear(this);

  // Build some pre-formed Events.
  read_abort_event.repurpose(MANUVR_MSG_XPORT_QUEUE_RDY, (EventReceiver*) this);
  read_abort_event.isManaged(true);
  read_abort_event.specific_target = (EventReceiver*) this;
  read_abort_event.priority        = 1;
  read_abort_event.addArg(xport_id);  // Add our assigned transport ID to our pre-baked argument.

  _xport_mtu = MANUVR_USB_BUF_SIZE;
}

/**
* Destructor
*/
STM32F7USB::~STM32F7USB() {
  _accumulator.clear();
}



void STM32F7USB::tx_wakeup() {
  read_abort_event.fireNow();
}


void STM32F7USB::rx_wakeup() {
  read_abort_event.fireNow();
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
  if (connected() && !_tx_in_progress) {
    char* working_chunk = _accumulator.position(0);
    if (write_port((uint8_t*) working_chunk, strlen(working_chunk))) {
      // TODO: Fail-over timer? Disconnection signal?
      _accumulator.drop_position(0);
    }
  }
  return MEM_MGMT_RESPONSIBLE_BEARER;  // We took the buffer.
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
  //#if defined(__MANUVR_DEBUG)
  //  if (getVerbosity() > 3) local_log.concatf("STM32F7USB initialized.\n");
  //#endif
  initialized(true);
  listening(true);
  connected(true);

  //if (local_log.length() > 0) Kernel::log(&local_log);
  return 0;
}

/**
* Read input from local keyboard.
*/
int8_t STM32F7USB::read_port() {
  int read_len = 0;
	/* Check if we are ready to use, if drivers are OK installed on computer */
	if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) {
		/* We are ready */
		/* Check if anything received */
    uint8_t buf[256];
    uint16_t n = TM_USBD_CDC_GetArray(TM_USB_FS, buf, 255);
    //int n = _usb_rx_buf.length();
    if (n > 0) {
      buf[n] = '\0';
      bytes_received += n;
      //TM_USBD_CDC_Putc(TM_USB_FS, buf[0]);  // Local echo
      //const char* test_str = "Got a character.\n";
      //write_port((uint8_t*) test_str, strlen(test_str));
      BufferPipe::fromCounterparty(buf, n, MEM_MGMT_RESPONSIBLE_BEARER);
    }
	}
  else {
		/* We are not ready */
	}

  return read_len;
}


bool STM32F7USB::write_port(uint8_t* out, int out_len) {
  if (connected()) {
    if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) {
      TM_USBD_CDC_PutArray(TM_USB_FS, out, (uint16_t) out_len);
      //_tx_in_progress = true;
    	TM_USBD_CDC_Process(TM_USB_FS);
      bytes_sent += out_len;
      return true;
    }
  }
  return false;
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
int8_t STM32F7USB::attached() {
  EventReceiver::attached();

  read_abort_event.alterScheduleRecurrence(0);
  read_abort_event.alterSchedulePeriod(50);
  read_abort_event.autoClear(false);
  read_abort_event.enableSchedule(true);
  #if !defined (__BUILD_HAS_THREADS)
  platform.kernel()->addSchedule(&read_abort_event);
  #endif

  reset();
  if (_accumulator.count() > 0) {
    TM_USBD_CDC_Puts(TM_USB_FS, (const char*)_accumulator.string());
    //_tx_in_progress = true;
  	TM_USBD_CDC_Process(TM_USB_FS);
  }
  return 1;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void STM32F7USB::printDebug(StringBuilder *temp) {
  ManuvrXport::printDebug(temp);
  temp->concatf("-- Class size      %d\n",     sizeof(STM32F7USB));
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
  switch (event->eventCode()) {
    case MANUVR_MSG_XPORT_SEND:
      event->clearArgs();
      break;
    case MANUVR_MSG_XPORT_QUEUE_RDY:
      if (_accumulator.count()) {
        return_value = EVENT_CALLBACK_RETURN_RECYCLE;
      }
      break;
    default:
      break;
  }

  return return_value;
}


int8_t STM32F7USB::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;

  switch (active_event->eventCode()) {
    case MANUVR_MSG_XPORT_QUEUE_RDY:
      read_port();
      if (_accumulator.count()) {
        char* working_chunk = _accumulator.position(0);
        if (write_port((uint8_t*) working_chunk, strlen(working_chunk))) {
          // TODO: Fail-over timer? Disconnection signal?
          _accumulator.drop_position(0);
        }
      }
      return_value++;
      break;

    case MANUVR_MSG_SYS_BOOTLOADER:
    case MANUVR_MSG_SYS_REBOOT:
    case MANUVR_MSG_SYS_SHUTDOWN:
      connected(false);
      listening(false);
      TM_USBD_Stop(TM_USB_FS);    // DeInit() The USB device.
      return_value++;
      break;
    default:
      return_value += ManuvrXport::notify(active_event);
      break;
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}

#endif  // STM32F7XX | STM32F746xx
