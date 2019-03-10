/*
File:   Digitabulum.cpp
Author: J. Ian Lindsay
Date:   2019.02.10

Copyright 2019 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


This class is intended to be functional glue-logic abstracted from platform.
This way, we don't have to write application logic for every supported platform.

*/


#include "Digitabulum.h"
#include <Kernel.h>
#include <Platform/Platform.h>
#include <Transports/ManuvrSocket/ManuvrTCP.h>
#include "Digitabulum/ManuLegend/ManuLegendPipe.h"

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

Digitabulum* Digitabulum::INSTANCE = nullptr;
DigitabulumFrameFxnPtr Digitabulum::frame_cb = nullptr;


/* These options are fixed with respect to the application layer. */
const ATECC508Opts atecc_opts(
  (uint8_t) 255
);


ManuLegendPipe* PIPE = nullptr;
const uint8_t pipe_plan[] = {1, 0};

BufferPipe* _pipe_factory_1(BufferPipe* _n, BufferPipe* _f) {
  if (nullptr != PIPE) {
    PIPE->setNear(_n);
  }
  return (BufferPipe*) PIPE;
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/
/*
* Constructor.
*/
Digitabulum::Digitabulum(I2CAdapter* i2c_adapter, PMU* _pmu, const DigitabulumOpts* _o) :
    EventReceiver("Digitabulum"),
    leds(_o->adp_pins),
    cpld(_o->cpld_pins),
    manu(&cpld),
    atec(&atecc_opts),
    pmu(_pmu),
    _opts(_o) {
  if (nullptr == Digitabulum::INSTANCE) {
    Digitabulum::INSTANCE = this;
  }
  Kernel* kernel = platform.kernel();
  PIPE = &_def_pipe;

  if (0 != BufferPipe::registerPipe(1, _pipe_factory_1)) {
    Kernel::log("Failed to add ManuLegendPipe to the pipe registry.\n");
    exit(1);
  }

  #if defined(MANUVR_SUPPORT_TCPSOCKET)
    ManuvrTCP* tcp_srv = new ManuvrTCP((const char*) "0.0.0.0", 2319);
    tcp_srv->setPipeStrategy(pipe_plan);
    kernel->subscribe(tcp_srv);
    tcp_srv->listen();
  #endif

  kernel->subscribe(&leds);
  kernel->subscribe(&cpld);
  kernel->subscribe(&manu);
  i2c_adapter->addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  i2c_adapter->addSlaveDevice((I2CDevice*) &atec);

  _def_pipe.active(true);  // TODO: Should be in pipe override for connection.

  // Digit and metacarpals LEDs have a maximum current of 30mA.
  // The wrist unit RGB LED has a max current of 50/25/25.
  // We back off a bit from those values.
  leds.setMaxCurrents(30, 30, 30, 30, 30, 30, 25, 25, 25);
}


/*
* Destructor.
*/
Digitabulum::~Digitabulum() {
}


int8_t Digitabulum::init() {
  return 0;
}


/*
* Dump this item to the dev log.
*/
void Digitabulum::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
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
int8_t Digitabulum::attached() {
  if (EventReceiver::attached()) {
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
int8_t Digitabulum::callback_proc(ManuvrMsg* event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = (0 == event->refCount()) ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    default:
      break;
  }

  return return_value;
}


int8_t Digitabulum::notify(ManuvrMsg* active_event) {
  int8_t return_value = 0;

  switch (active_event->eventCode()) {
    case DIGITABULUM_MSG_CPLD_RESET_COMPLETE:
      led_wrist_color(0x0000000A);
      break;
    //case MANUVR_MSG_SYS_POWER_MODE:
    //  return_value++;
    //  break;
    case DIGITABULUM_MSG_IMU_MAP_STATE:
      // TODO: This idea might get brought back.
      //frame_cb(manu.getPipe());
      if (manu.hasFrame()) {
        SensorFrame* frame = manu.takeFrame();
        _def_pipe.offer(frame);
        manu.returnFrame(frame);
      }
      break;

    case DIGITABULUM_MSG_MANU_STATE_STABLE:  // ManuManager state machine is stable.
      local_log.concatf("ManuManager reached stable state:  %s\n", ManuManager::getManuStateString(manu.getState()));
      switch (manu.getState()) {
        case ManuState::READY_READING:
          indicate_reading();
          break;
        //case ManuState::READY_IDLE:
        case ManuState::READY_PAUSED:
          led_wrist_color(0x00050005);
          break;
        case ManuState::ASLEEP:
          indicate_sleep();
          break;
        case ManuState::FAULT:
          indicate_error(5, 200);
          break;
        default:
          break;
      }
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
/*******************************************************************************
* Console I/O
*******************************************************************************/

static const ConsoleCommand console_cmds[] = {
  { "L", "Host-facing ManuLegend" },
  { "r", "Reset" }
};


uint Digitabulum::consoleGetCmds(ConsoleCommand** ptr) {
  *ptr = (ConsoleCommand*) &console_cmds[0];
  return sizeof(console_cmds) / sizeof(ConsoleCommand);
}


void Digitabulum::consoleCmdProc(StringBuilder* input) {
  const char* str = (char *) input->position(0);
  char c    = *str;
  int temp_int = ((*(str) != 0) ? atoi((char*) str+1) : 0);

  if (input->count() > 1) {
    temp_int = input->position_as_int(1);
  }

  switch (c) {
    case 'i':   // Debug prints.
      switch (temp_int) {
        case 1:
          local_log.concatf("\nsizeof(Digitabulum):   %u\n", sizeof(Digitabulum));
          local_log.concatf("  sizeof(ATECC508):    %u\n", sizeof(ATECC508));
          local_log.concatf("  sizeof(CPLDDriver):  %u\n", sizeof(CPLDDriver));
          local_log.concatf("  sizeof(ManuManager): %u\n", sizeof(ManuManager));
          local_log.concatf("  sizeof(ADP8866):     %u\n", sizeof(ADP8866));
          break;
        case 9:
          _def_pipe.printDebug(&local_log);
          break;

        default:
          printDebug(&local_log);
          break;
      }
      break;

    case 'm':
      Kernel::raiseEvent(DIGITABULUM_MSG_IMU_MAP_STATE, nullptr);
      break;

    case 'r':
      reset();
      break;

    case 'l':   // LED tests
      switch (temp_int) {
        case 1:
          indicate_sleep();
          break;
        case 2:
          indicate_reading();
          break;
        case 3:
          led_wrist_color(0x00000500);
          break;
      }
      break;

    case 'L':   // Host-facing ManuLegend
      switch (temp_int) {
        case 1:
          {
            local_log.concat("_def_pipe LegendString:\t");
            StringBuilder shuttle;
            _def_pipe.getLegendString(&shuttle);
            shuttle.printDebug(&local_log);
          }
          break;
        case 2:
          _def_pipe.active(!_def_pipe.active());
          local_log.concatf("active() %c\n", _def_pipe.active() ? 'y' : 'n');
          break;
        case 3:
          _def_pipe.broadcast_legend();
          local_log.concat("Legend broadcast.\n");
          break;
        case 4:
          {
            SensorFrame frame;
            frame.stackLegend(&_def_pipe);
            _def_pipe.offer(&frame);
            local_log.concat("Cycled blank frame to host.\n");
          }
          break;
        case 5:
          _def_pipe.decoupleSeq(!_def_pipe.decoupleSeq());
          local_log.concatf("decoupleSeq() %c\n", _def_pipe.decoupleSeq() ? 'y' : 'n');
          break;
        case 7:
          _def_pipe.stackLegend(manu.getActiveLegend());
          local_log.concat("Moving _root_leg to _def_pipe.\n");
          break;
        case 8:
          manu.getActiveLegend()->stackLegend(&_def_pipe);
          local_log.concat("Moving _def_pipe to _root_leg.\n");
          break;
        case 9:   // Will cause a frame broadcast cycle.
          Kernel::raiseEvent(DIGITABULUM_MSG_IMU_MAP_STATE, nullptr);
          break;
        default:
          _def_pipe.printManuLegend(&local_log);
          break;
      }
      break;

    case 'E':
      switch (temp_int) {
        case 1:
        case 2:
        case 3:
        case 4:
          {
            ManuEncoding e = (ManuEncoding) (temp_int - 1);
            local_log.concatf("Switching to ManuEncoding::%s\n", ManuLegendPipe::encoding_label(e));
            _def_pipe.encoding(e);
          }
        default:
          local_log.concatf("Using ManuEncoding::%s\n", ManuLegendPipe::encoding_label(_def_pipe.encoding()));
      }
      break;

    case 'v':
      // The verbosity level given here will be propagated downward to all
      //   components involved with the sensor front-end board.
      cpld.setVerbosity(temp_int);
      leds.setVerbosity(temp_int);
      manu.setVerbosity(temp_int);
      break;

    default:
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT



/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

/*
* Perform a software reset.
*/
void Digitabulum::reset() {
  cpld.reset();
  leds.reset();
}



/*******************************************************************************
* LED indicator functions...                                                   *
*******************************************************************************/

int8_t Digitabulum::led_set_digit_brightness(DigitPort p, uint8_t brightness) {
  leds.set_brightness((uint8_t) p, brightness);
  return 0;
}


int8_t Digitabulum::led_wrist_color(uint8_t r, uint8_t g, uint8_t b) {
  leds.quell_all_timers();
  leds.set_brightness(6, g);
  leds.set_brightness(7, b);
  leds.set_brightness(8, r);
  return 0;
}


int8_t Digitabulum::led_wrist_color(uint32_t color) {
  return led_wrist_color(
    (uint8_t) ((color >> 16) & 0xFF),
    (uint8_t) ((color >> 8) & 0xFF),
    (uint8_t) (color & 0xFF)
  );
}


int8_t Digitabulum::indicate_sleep() {
  led_wrist_color(0x00000000);
  leds.set_fade(1750, 1750);
  leds.pulse_channel(6, 0x05, 250, 12500);
  return 0;
}


int8_t Digitabulum::indicate_reading() {
  led_wrist_color(0x00000000);
  leds.set_fade(250, 250);
  leds.pulse_channel(7, 0x10, 150, 500);
  leds.pulse_channel(8, 0x10, 150, 500);
  return 0;
}


int8_t Digitabulum::indicate_error(uint8_t p_count, uint16_t ms_period) {
  led_wrist_color(0x00000000);
  leds.set_fade(500, 1750);
  leds.pulse_channel(8, 0x15, 250, ms_period);
  return 0;
}
