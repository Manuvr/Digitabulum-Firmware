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
#include "IRQRouter.h"
#include "../ManuLegend/ManuManager.h"


/*******************************************************************************
* The cryptic debug commands and static formats are becoming a serious pain.
* This is an experiment to be migrated upstream into Manuvr if it works well
* enough.
*******************************************************************************/

typedef struct cli_cmd_def_t {
  // TODO: Might-should use lambdas for function pointer?
  const char* help_text;
} CLICmdDef;

/*******************************************************************************
* .-. .----..----.    .-.     .--.  .-. .-..----.
* | |{ {__  | {}  }   | |    / {} \ |  `| || {}  \
* | |.-._} }| .-. \   | `--./  /\  \| |\  ||     /
* `-'`----' `-' `-'   `----'`-'  `-'`-' `-'`----'
*
* Interrupt service routine support functions. Everything in this block
*   executes under an ISR. Keep it brief...
*******************************************************************************/
/* Access to CPLD driver from other classes. */
volatile CPLDDriver* cpld = nullptr;

/* Used to minimize software burden incurred by timeout support. */
volatile bool timeout_punch = false;

/*
* For the sake of speed, we handle IRQ data issues in a single memory field,
*   none of which is concurrency-safe.
* IRQs are double-buffered in the first 20 bytes, over which DMA runs continuously.
*   Upon DMA half-complete, _irq_data_ptr is set to the most-recent (stable) half
*   of the buffer, and any differences between them are computed and stored in...
* _irq_data[20-29] are diff. Diffs get clobbered each DMA half-cycle, so to
*   avoid losing track of IRQs, we then write them into...
* _irq_data[30-39] an action accumulator. The ISR writes...
*    action |= diff
*
* The ISR then fires the _irq_data_arrival message, with _irq_data[30-39]
*   attached as a reference. When it comes time to thread, this will be revised.
*/
volatile static uint8_t  _irq_data[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile static uint8_t* _irq_data_0   = &(_irq_data[0]);   // Convenience
volatile static uint8_t* _irq_data_1   = &(_irq_data[10]);  // Convenience
volatile static uint8_t* _irq_diff     = &(_irq_data[20]);  // Convenience
volatile static uint8_t* _irq_accum    = &(_irq_data[30]);  // Convenience
volatile static uint8_t* _irq_data_ptr = _irq_data_0;  // Used for block-wise access.
volatile static uint32_t _irq_frames_rxd   = 0;  // How many IRQ frames have arrived.
volatile static uint32_t _irq_latency_0    = 0;  // IRQ latency discovery.
volatile static uint32_t _irq_latency_1    = 0;  // IRQ latency discovery.
volatile static uint32_t _irq_latency_2    = 0;  // IRQ latency discovery.


uint8_t debug_buffer[64];

/* This message is dispatched when IRQ data changes. */
static ManuvrMsg _irq_data_arrival;

// These are debug. Cut them.
uint8_t active_imu_position = 0;
bool op_abuse_test = false;

/**
* ISR for CPLD GPIO.
*/
void cpld_gpio_isr() {
  Kernel::log("cpld_gpio_isr()\n");
}

/**
* ISR called by the selectable IRQ source.
*/
void cpld_wakeup_isr(){
  Kernel::log("cpld_wakeup_isr()\n");
}


// TODO: Textual inclusion of source files is super ugly...
#if defined(STM32F746xx)
  #include "target_stm32f746.cpp"
#elif defined(__MANUVR_LINUX)
  #include "target_linux.cpp"
#elif defined(__MANUVR_ESP32)
  #include "target_esp32.cpp"
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
/* Register representations. */
uint8_t  CPLDDriver::cpld_version       = 0;  // CPLD version byte.
uint8_t  CPLDDriver::cpld_conf_value    = 0;  // Configuration.
uint8_t  CPLDDriver::forsaken_digits    = 0;  // Forsaken digits.
uint8_t  CPLDDriver::cpld_wakeup_source = 0;  // WAKEUP mapping.
uint8_t  CPLDDriver::irq76_conf         = 0;  // Aggregated IRQ settings.



SPIBusOp  CPLDDriver::preallocated_bus_jobs[CPLD_SPI_PREALLOC_COUNT];

const unsigned char MSG_ARGS_IMU_READ[] = {
  (uint8_t)TCode::UINT8, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT, 0  // IMU id and a collection of readings.
};


const unsigned char MSG_ARGS_IMU_LEGEND[] = {
  (uint8_t)TCode::UINT8,  (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16,
  (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16,
  (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16,
  (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16,
  (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16, (uint8_t)TCode::UINT16,
  0     // 37 bytes: An IMU Legend broadcast.
};

/* There are only two grammatical forms represented here. A zero-length, and a giant block of Vectors. */
const unsigned char MSG_ARGS_IMU_MAP_STATE[] = {
  (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
    (uint8_t)TCode::VECT_4_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::VECT_3_FLOAT, (uint8_t)TCode::FLOAT,
  0   // (17 IMUs * 3 vectors + 1 float per IMU) * (4 bytes per float) = 680) bytes: Statement of debug map.
};    // 0 bytes: Request for present map.



const MessageTypeDef cpld_message_defs[] = {
  /* These are messages specific to Digitabulum. */
  {  DIGITABULUM_MSG_IMU_IRQ_RAISED       , 0x0000,               "IMU_IRQ_RAISED" , ManuvrMsg::MSG_ARGS_NONE }, // IRQ asserted by CPLD.
  {  DIGITABULUM_MSG_IMU_READ             , 0x0000,               "IMU_READ"       , MSG_ARGS_IMU_READ },  // IMU read request. Argument is the ID.
  {  DIGITABULUM_MSG_IMU_QUAT_CRUNCH      , 0x0000,               "IMU_QUAT_CRUNCH", ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_IMU_MAP_STATE        , MSG_FLAG_EXPORTABLE,  "IMU_MAP_STATE"  , MSG_ARGS_IMU_MAP_STATE }, //
  {  DIGITABULUM_MSG_IMU_INIT             , MSG_FLAG_EXPORTABLE,  "IMU_INIT"       , ManuvrMsg::MSG_ARGS_NONE }, // Signal to build the IMUs.
  {  DIGITABULUM_MSG_IMU_LEGEND           , MSG_FLAG_EXPORTABLE,  "IMU_LEGEND"     , MSG_ARGS_IMU_LEGEND }, // No args? Asking for this legend. Many args: Legend provided.
  {  DIGITABULUM_MSG_IMU_TAP              , MSG_FLAG_EXPORTABLE,  "IMU_TAP"        , ManuvrMsg::MSG_ARGS_NONE }, // IMU id and optional threshold.
  {  DIGITABULUM_MSG_IMU_DOUBLE_TAP       , MSG_FLAG_EXPORTABLE,  "IMU_DBL_TAP"    , ManuvrMsg::MSG_ARGS_NONE }, // IMU id and optional threshold.

  {  DIGITABULUM_MSG_CPLD_DIGIT_DROP      , 0x0000,               "DIGIT_DROP"     , ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_CPLD_RESET_COMPLETE  , 0x0000,               "CPLD_RST_CMPLTE", ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_CPLD_RESET_CALLBACK  , 0x0000,               "CPLD_RST_CB"    , ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_SPI_QUEUE_READY      , 0x0000,               "SPI_Q_RDY"      , ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_SPI_CB_QUEUE_READY   , 0x0000,               "SPICB_RDY"      , ManuvrMsg::MSG_ARGS_NONE }, //
  {  DIGITABULUM_MSG_SPI_TIMEOUT          , 0x0000,               "SPI_TO"         , ManuvrMsg::MSG_ARGS_NONE }, //
};


const char* CPLDDriver::digitStateToString(DigitState x) {
  switch (x) {
    case DigitState::ASLEEP:  return "ASLEEP";
    case DigitState::ABSENT:  return "ABSENT";
    case DigitState::AWAKE:   return "AWAKE";
    case DigitState::UNKNOWN:
    default:
      break;
  }
  return "UNKNOWN";
}
const char* CPLDDriver::getDigitPortString(DigitPort x) {
  switch (x) {
    case DigitPort::MC:  return "MC";
    case DigitPort::PORT_1:  return "PORT_1";
    case DigitPort::PORT_2:  return "PORT_2";
    case DigitPort::PORT_3:  return "PORT_3";
    case DigitPort::PORT_4:  return "PORT_4";
    case DigitPort::PORT_5:  return "PORT_5";
    case DigitPort::UNKNOWN:
    default:
      break;
  }
  return "UNKNOWN";
}



// NO ERROR CHECKING! Don't call this with an argument >79.
bool irq_is_presently_high(const uint8_t bit) {
  const uint8_t bit_offset  = bit & 0x07;  // Cheaper than modulus 8.
  const uint8_t byte_offset = bit >> 3;    // Cheaper than div by 8.
  return ((_irq_data_ptr[byte_offset] & (0x80 >> bit_offset)) != 0);
}

// NO ERROR CHECKING! Don't call this with an argument >79.
bool irq_demands_service(const uint8_t bit) {
  const uint8_t bit_offset  = bit & 0x07;  // Cheaper than modulus 8.
  const uint8_t byte_offset = bit >> 3;    // Cheaper than div by 8.
  return ((_irq_accum[byte_offset] & (0x80 >> bit_offset)) != 0);
}

/**
* Returns at most 12-bits of IRQ service data for the given digit.
*
* @return True if the IRQ data confirms a digit.
*/
uint16_t irq_digit_slice_service(DigitPort x) {
  uint16_t ret = 0;
  // TODO: There is a way to do this without branching.
  switch (x) {
    case DigitPort::MC:
      ret = _irq_accum[0];
      break;
    case DigitPort::PORT_1:
      ret = (uint16_t) (_irq_accum[1] & 0xFF) << 4;
      ret += (_irq_accum[2] >> 4);
      break;
    case DigitPort::PORT_2:
      ret = (uint16_t) (_irq_accum[2] & 0x0F) << 8;
      ret += _irq_accum[3];
      break;
    case DigitPort::PORT_3:
      ret = (uint16_t) (_irq_accum[4] & 0xFF) << 4;
      ret += (_irq_accum[5] >> 4);
      break;
    case DigitPort::PORT_4:
      ret = (uint16_t) (_irq_accum[5] & 0x0F) << 8;
      ret += _irq_accum[6];
      break;
    case DigitPort::PORT_5:
      ret = (uint16_t) (_irq_accum[7] & 0xFF) << 4;
      ret += (_irq_accum[8] >> 4);
      break;
    default:
      break;
  }
  return ret;
};


/**
* Returns at most 12-bits of IRQ data for the given digit.
*
* @return True if the IRQ data confirms a digit.
*/
uint16_t irq_digit_slice_current(DigitPort x) {
  uint16_t ret = 0;
  // TODO: There is a way to do this without branching.
  switch (x) {
    case DigitPort::MC:
      ret = _irq_data_ptr[0];
      break;
    case DigitPort::PORT_1:
      ret = (uint16_t) (_irq_data_ptr[1] & 0xFF) << 4;
      ret += (_irq_data_ptr[2] >> 4);
      break;
    case DigitPort::PORT_2:
      ret = (uint16_t) (_irq_data_ptr[2] & 0x0F) << 8;
      ret += _irq_data_ptr[3];
      break;
    case DigitPort::PORT_3:
      ret = (uint16_t) (_irq_data_ptr[4] & 0xFF) << 4;
      ret += (_irq_data_ptr[5] >> 4);
      break;
    case DigitPort::PORT_4:
      ret = (uint16_t) (_irq_data_ptr[5] & 0x0F) << 8;
      ret += _irq_data_ptr[6];
      break;
    case DigitPort::PORT_5:
      ret = (uint16_t) (_irq_data_ptr[7] & 0xFF) << 4;
      ret += (_irq_data_ptr[8] >> 4);
      break;
    default:
      break;
  }
  return ret;
};


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor. Also populates the global pointer reference.
*/
CPLDDriver::CPLDDriver(const CPLDPins* p) : EventReceiver("CPLDDriver"), BusAdapter(CPLD_SPI_MAX_QUEUE_DEPTH), _pins(p) {
  if (nullptr == cpld) {
    cpld = this;
    ManuvrMsg::registerMessages(cpld_message_defs, sizeof(cpld_message_defs) / sizeof(MessageTypeDef));
  }
  gpioSetup();

  // Build some pre-formed Events.
  event_spi_callback_ready.repurpose(DIGITABULUM_MSG_SPI_CB_QUEUE_READY, (EventReceiver*) this);
  event_spi_callback_ready.incRefs();
  event_spi_callback_ready.specific_target = (EventReceiver*) this;
  event_spi_callback_ready.priority(5);

  SPIBusOp::event_spi_queue_ready.repurpose(DIGITABULUM_MSG_SPI_QUEUE_READY, (EventReceiver*) this);
  SPIBusOp::event_spi_queue_ready.incRefs();
  SPIBusOp::event_spi_queue_ready.specific_target = (EventReceiver*) this;
  SPIBusOp::event_spi_queue_ready.priority(5);

  // Mark all of our preallocated SPI jobs as "No Reap" and pass them into the prealloc queue.
  for (uint8_t i = 0; i < CPLD_SPI_PREALLOC_COUNT; i++) {
    preallocated_bus_jobs[i].returnToPrealloc(true);     // Implies SHOuLD_REAP = false.
    preallocated.insert(&preallocated_bus_jobs[i]);
  }

  current_job = nullptr;
  _er_set_flag(CPLD_FLAG_QUEUE_IDLE);
}

/**
* Destructor. Should never be called.
* We don't bother tearing down any of the HAL-constructed interfaces. Since we
*   will never destroy this object, any code here will only take up flash space
*   needlessly. But we keep it here in commentary in case it is needed later.
*/
CPLDDriver::~CPLDDriver() {
}


/**
* Setup GPIO pins and their bindings to on-chip peripherals, if required.
*/
void CPLDDriver::gpioSetup() {
  // NOTE: The SPI pins are not handled here, since their efficient treatment
  //   is left to the platform-specific face of this driver.
  if (255 != _pins.reset) {
    gpioDefine(_pins.reset, GPIOMode::OUTPUT);
    setPin(_pins.reset, false);  // Hold the CPLD in reset.
  }
  if (255 != _pins.req) {
    gpioDefine(_pins.req, GPIOMode::OUTPUT);
    setPin(_pins.req, false);  // Xfer is triggered on the rising-edge.
  }
  if (255 != _pins.clk) {
    gpioDefine(_pins.clk, GPIOMode::OUTPUT);
    setPin(_pins.clk, true);
  }
  if (255 != _pins.oe) {
    gpioDefine(_pins.oe, GPIOMode::OUTPUT);
    setPin(_pins.oe, true);   // CPLD has internal pullup on this pin.
  }
  if (255 != _pins.gpio) {
    setPinFxn(_pins.gpio, CHANGE, cpld_gpio_isr);
  }
}


/**
* Resets the CPLD, and any registers within it.
* Any host-controlled pins affecting CPLD config are not altered by this call,
*   so it might be worth disabling some IRQs prior to doing something that might
*   make them seizure.
*/
void CPLDDriver::reset() {
  // Stop servicing IRQs and mark the CPLD as not ready.
  _er_clear_flag(CPLD_FLAG_SVC_IRQS | CPLD_FLAG_CPLD_READY);

  setPin(_pins.reset, false);  // Drive the reset pin low...
  setPin(_pins.req,   false);  // Reset the transfer pin.
  externalOscillator(true);    // Turn on the default oscillator...
  cpld_conf_value    = 0x00;   // Set our register representations to their
  cpld_version       = 0x00;   //   default values.
  cpld_wakeup_source = 0x00;
  forsaken_digits    = 0x00;
  irq76_conf         = 0x00;
  bus_timeout_millis = 5;   // TODO: Dynamically relate this to clock frequency.

  for (int z = 0; z < 30; z++) _irq_data[z] = 0;   // Wipe the IRQ data.
  _irq_data_ptr = _irq_data_0;  // Used for block-wise access.

  purge_queued_work();          // Purge the SPI queue...
  purge_stalled_job();
  hw_flush();

  // Fire the oneshot to bring us out of reset after several ms...
  ManuvrMsg* msg = Kernel::returnEvent(DIGITABULUM_MSG_CPLD_RESET_CALLBACK, this);
  msg->alterScheduleRecurrence(0);
  msg->alterSchedulePeriod(100);
  msg->autoClear(true);
  msg->enableSchedule(true);
  platform.kernel()->addSchedule(msg);
}


/**
* This function evaluates the last-known state against the new state of the
*   the version and configuration registers, and processes consequences.
* This function will be called as a result of every access (read or write) to
*   a CPLD internal register. The return code decides if the class will proceed.
*
* @param  nu  The updated value of the config register.
* @return 0 on nominal conditions, nonzero otherwise.
*/
int CPLDDriver::_process_cpld_base_return(uint8_t _version, uint8_t _conf) {
  if ((_version >= CPLD_MINIMUM_VERSION) && (0xFF != _version)) {
    // This block means the version code that cxame back was ok.
    if (_version != cpld_version) {
      // If the version changed, it can only mean we are getting our first
      //   confirmation of correct operation following reset.
      cpld_version = _version;
      Kernel::raiseEvent(DIGITABULUM_MSG_CPLD_RESET_COMPLETE, nullptr);
    }
    // Now deal with the config/status byte.
    uint8_t diff = cpld_conf_value ^ _conf;
    if (diff) {
      if (diff & CPLD_CONF_BIT_INT_CLK) {
        if (_conf & CPLD_CONF_BIT_INT_CLK) {
          // TODO: Optimize the branches out of this once it is shown to work.
          _er_set_flag(CPLD_FLAG_INT_OSC, true);
          // We needed to wait for the last write operation before doing this...
          externalOscillator(false);
        }
        else {
          _er_set_flag(CPLD_FLAG_INT_OSC, false);
        }
      }
      if (diff & CPLD_CONF_BIT_GPIO) {
        if (op_abuse_test) {
          // Causes endless bus traffic to toggle the GPIO pin.
          setCPLDConfig(CPLD_CONF_BIT_GPIO, !(_conf & CPLD_CONF_BIT_GPIO));
        }
      }
      if (diff & CPLD_CONF_BIT_DEN_AG_C) {
        _er_set_flag(CPLD_CONF_BIT_DEN_AG_C, (_conf & CPLD_CONF_BIT_DEN_AG_C));
      }
      if (diff & CPLD_CONF_BIT_DEN_AG_MC) {
        _er_set_flag(CPLD_CONF_BIT_DEN_AG_MC, (_conf & CPLD_CONF_BIT_DEN_AG_MC));
      }
      if (diff & CPLD_CONF_BIT_IRQ_SCAN) {
      }
      if (diff & CPLD_CONF_BIT_IRQ_74) {
        // We ought to be expecting a message from the IRQ subsystem.
      }
      if (diff & CPLD_CONF_BIT_PWR_CONSRV) {
      }
      if (diff & CPLD_CONF_BIT_ALIGN_XFER) {
      }
      cpld_conf_value = _conf;
    }
    return 0;
  }
  if (getVerbosity() > 1) {
    local_log.concatf("CPLD returned a bad version code: 0x%02x (Extant: 0x%02x), CONFIG: 0x%02x\n", _version, cpld_version, _conf);
  }
  return -1;
}




/*******************************************************************************
*  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄   Members related to the work queue
* ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌  and SPI bus I/O
* ▐░█▀▀▀▀▀▀▀▀▀ ▐░█▀▀▀▀▀▀▀█░▌ ▀▀▀▀█░█▀▀▀▀
* ▐░▌          ▐░▌       ▐░▌     ▐░▌       SPI transactions have two phases:
* ▐░█▄▄▄▄▄▄▄▄▄ ▐░█▄▄▄▄▄▄▄█░▌     ▐░▌       1) ADDR (Addressing) Max of 4 bytes.
* ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌     ▐░▌       2) IO_WAIT (Transfer)
*  ▀▀▀▀▀▀▀▀▀█░▌▐░█▀▀▀▀▀▀▀▀▀      ▐░▌
*           ▐░▌▐░▌               ▐░▌       CPLD-resident registers don't have a
*  ▄▄▄▄▄▄▄▄▄█░▌▐░▌           ▄▄▄▄█░█▄▄▄▄     transfer phase, as there is never
* ▐░░░░░░░░░░░▌▐░▌          ▐░░░░░░░░░░░▌    more than 4-bytes-worth of clock
*  ▀▀▀▀▀▀▀▀▀▀▀  ▀            ▀▀▀▀▀▀▀▀▀▀▀     required to complete a transaction.
*******************************************************************************/

/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/**
* Called prior to the given bus operation beginning.
* Returning 0 will allow the operation to continue.
* Returning anything else will fail the operation with IO_RECALL.
*   Operations failed this way will have their callbacks invoked as normal.
*
* @param  _op  The bus operation that was completed.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t CPLDDriver::io_op_callahead(BusOp* _op) {
  // Bus adapters don't typically do anything here, other
  //   than permit the transfer.
  return 0;
}

/**
* When a bus operation completes, it is passed back to its issuing class.
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

  switch (op->getTransferParam(0)) {
    case CPLD_REG_VERSION:
      if (0 == _process_cpld_base_return(op->getTransferParam(2), op->getTransferParam(3))) {
        if (getVerbosity() > 3) local_log.concatf("CPLD r%d\n\tCONFIG:  0x%02x\n", cpld_version, cpld_conf_value);
      }
      break;
    case CPLD_REG_CONFIG:
      // No special consequences on return, but we need to check rx/tx to decide
      //   which byte is most-current.
      _process_cpld_base_return(
        op->getTransferParam(2),
        op->getTransferParam(BusOpcode::TX == op->get_opcode() ? 1 : 3)
      );
      break;
    case CPLD_REG_WAKEUP_IRQ:
      if (0 == _process_cpld_base_return(op->getTransferParam(2), op->getTransferParam(3))) {
        cpld_wakeup_source = op->getTransferParam(1);
      }
      break;
    case CPLD_REG_DIGIT_FORSAKE:
      if (0 == _process_cpld_base_return(op->getTransferParam(2), op->getTransferParam(3))) {
        forsaken_digits = op->getTransferParam(1);
      }
      break;
    default:
      if (getVerbosity() > 2) local_log.concatf("An SPIBusOp called back with an unknown register: 0x%02x\n", op->getTransferParam(0));
      break;
  }

  flushLocalLog();
  return SPI_CALLBACK_NOMINAL;
}


/**
* This is what is called when the class wants to conduct a transaction on the bus.
* Note that this is different from other class implementations, in that it checks for
*   callback population before clobbering it. This is because this class is also the
*   SPI driver. This might end up being reworked later.
*
* @param  _op  The bus operation to execute.
* @return Zero on success, or appropriate error code.
*/
int8_t CPLDDriver::queue_io_job(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;

  if (op) {
    if (nullptr == op->callback) {
      op->callback = (BusOpCallback*) this;
    }

    if ((getVerbosity() > 6) && (op->callback == (BusOpCallback*) this)) {
      op->profile(true);
    }

    if (op->get_state() != XferState::IDLE) {
      if (getVerbosity() > 3) Kernel::log("Tried to fire a bus op that is not in IDLE state.\n");
      return -4;
    }
    op->csActiveHigh(true);   // We enfoce this here to prevent having to
    op->setCSPin(_pins.req);  //   enforce it in many places.

    if ((nullptr == current_job) && (work_queue.size() == 0)){
      // If the queue is empty, fire the operation now.
      current_job = op;
      advance_work_queue();
      if (bus_timeout_millis) event_spi_timeout.delaySchedule(bus_timeout_millis);  // Punch the timeout schedule.
    }
    else {    // If there is something already in progress, queue up.
      if (_er_flag(CPLD_FLAG_QUEUE_GUARD) && (MAX_Q_DEPTH <= work_queue.size())) {
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


/*******************************************************************************
* ___     _                                  This is a template class for
*  |   / / \ o    /\   _|  _. ._ _|_  _  ._  defining arbitrary I/O adapters.
* _|_ /  \_/ o   /--\ (_| (_| |_) |_ (/_ |   Adapters must be instanced with
*                             |              a BusOp as the template param.
*******************************************************************************/

/**
* Calling this function will advance the work queue after performing cleanup
*   operations on the present or pending operation.
*
* @return the number of bus operations proc'd.
*/
int8_t CPLDDriver::advance_work_queue() {
  int8_t return_value = 0;

  timeout_punch = false;
  if (current_job) {
    switch (current_job->get_state()) {
      case XferState::TX_WAIT:
      case XferState::RX_WAIT:
        if (current_job->hasFault()) {
          _failed_xfers++;
          if (getVerbosity() > 3) local_log.concat("CPLDDriver::advance_work_queue():\t Failed at IO_WAIT.\n");
        }
        else {
          current_job->markComplete();
        }
        // No break on purpose.
      case XferState::COMPLETE:
        _total_xfers++;
        callback_queue.insert(current_job);
        current_job = nullptr;
        if (callback_queue.size() == 1) Kernel::staticRaiseEvent(&event_spi_callback_ready);
        break;

      case XferState::IDLE:
      case XferState::INITIATE:
        switch (current_job->begin()) {
          case XferFault::NONE:     // Nominal outcome. Transfer started with no problens...
            break;
          case XferFault::BUS_BUSY:    // Bus appears to be in-use. State did not change.
            // Re-throw queue_ready event and try again later.
            if (getVerbosity() > 2) local_log.concat("  advance_work_queue() tried to clobber an existing transfer on chain.\n");
            //Kernel::staticRaiseEvent(&event_spi_queue_ready);  // Bypass our method. Jump right to the target.
            break;
          default:    // Began the transfer, and it barffed... was aborted.
            if (getVerbosity() > 3) local_log.concat("CPLDDriver::advance_work_queue():\t Failed to begin transfer after starting.\n");
            callback_queue.insert(current_job);
            current_job = nullptr;
            if (callback_queue.size() == 1) Kernel::staticRaiseEvent(&event_spi_callback_ready);
            break;
        }
        break;

      /* Cases below ought to be handled by ISR flow... */
      case XferState::ADDR:
        //Kernel::log("####################### advance_operation entry\n");
        current_job->advance_operation(0, 0);
        //local_log.concat("####################### advance_operation exit\n");
        current_job->printDebug(&local_log);
        flushLocalLog();
        break;
      case XferState::STOP:
        if (getVerbosity() > 5) local_log.concatf("State might be corrupted if we tried to advance_queue(). \n");
        break;
      default:
        if (getVerbosity() > 3) local_log.concatf("advance_work_queue() default state \n");
        break;
    }
  }

  if (current_job == nullptr) {
    current_job = work_queue.dequeue();
    // Begin the bus operation.
    if (current_job) {
      if (XferFault::NONE != current_job->begin()) {
        if (getVerbosity() > 2) local_log.concatf("advance_work_queue() tried to clobber an existing transfer on the pick-up.\n");
        Kernel::staticRaiseEvent(&SPIBusOp::event_spi_queue_ready);  // Bypass our method. Jump right to the target.
      }
      return_value++;
    }
    else {
      // No Queue! Relax...
      event_spi_timeout.enableSchedule(false);  // Punch the timeout schedule.
    }
  }

  flushLocalLog();
  return return_value;
}


/**
* Purges only the jobs belonging to the given device from the work_queue.
* Leaves the currently-executing job.
*
* @param  dev  The device pointer that owns jobs we wish purged.
*/
void CPLDDriver::purge_queued_work_by_dev(BusOpCallback *dev) {
  if (nullptr == dev) return;
  SPIBusOp* current = nullptr;

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
  SPIBusOp* current = nullptr;
  while (work_queue.hasNext()) {
    current = work_queue.dequeue();
    current->abort(XferFault::QUEUE_FLUSH);
    reclaim_queue_item(current);
  }

  // Check this last to head off any silliness with bus operations colliding with us.
  purge_stalled_job();
}


/**
* Purges a stalled job from the active slot.
*/
void CPLDDriver::purge_stalled_job() {
  if (current_job) {
    current_job->abort(XferFault::QUEUE_FLUSH);
    reclaim_queue_item(current_job);
    current_job = nullptr;
  }
}


/**
* Return a vacant SPIBusOp to the caller, allocating if necessary.
*
* @return an SPIBusOp to be used. Only NULL if out-of-mem.
*/
SPIBusOp* CPLDDriver::new_op() {
  return (SPIBusOp*) BusAdapter::new_op();
}


/**
* Return a vacant SPIBusOp to the caller, allocating if necessary.
*
* @param  _op   The device pointer that owns jobs we wish purged.
* @param  _req  The device pointer that owns jobs we wish purged.
* @return an SPIBusOp to be used. Only NULL if out-of-mem.
*/
SPIBusOp* CPLDDriver::new_op(BusOpcode _op, BusOpCallback* _req) {
  SPIBusOp* return_value = new_op();
  return_value->set_opcode(_op);
  return_value->callback = _req;
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
    //if (getVerbosity() > 6) local_log.concatf("CPLDDriver::reclaim_queue_item(): \t About to wipe.\n");
    BusAdapter::return_op_to_pool(op);
  }
  else if (op->shouldReap()) {
    //if (getVerbosity() > 6) local_log.concatf("CPLDDriver::reclaim_queue_item(): \t About to reap.\n");
    delete op;
    _heap_frees++;
  }
  else {
    /* If we are here, it must mean that some other class fed us a const SPIBusOp,
       and wants us to ignore the memory cleanup. But we should at least set it
       back to IDLE.*/
    //if (getVerbosity() > 6) local_log.concatf("CPLDDriver::reclaim_queue_item(): \t Dropping....\n");
    op->set_state(XferState::IDLE);
  }

  flushLocalLog();
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

  while ((nullptr != temp_op) && (return_value < spi_cb_per_event)) {
  //if (nullptr != temp_op) {
    if (getVerbosity() > 6) temp_op->printDebug(&local_log);
    if (nullptr != temp_op->callback) {
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

  flushLocalLog();
  return return_value;
}



/*******************************************************************************
* CPLD register manipulation and integral hardware functions.                  *
*******************************************************************************/
/**
* Read an arbitrary CPLD register.
*
* @param  reg_addr  The address.
* @return 0 on success. Nonzero on failure.
*/
int8_t CPLDDriver::readRegister(uint8_t reg_addr) {
  SPIBusOp* temp = new_op();
  temp->set_opcode(BusOpcode::RX);
  temp->setParams(reg_addr, 0);  // Set the READ bit...
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
  SPIBusOp* temp = new_op();
  temp->set_opcode(BusOpcode::TX);
  temp->setParams(reg_addr, val);
  queue_io_job(temp);
  return 0;
}

/**
* The CPLD has an internal oscillator that can continue running if we put the
*   CPU to sleep.
* If we are about to disable the internal oscillator, be sure to fire up the
*   external clock first. Otherwise, the transfer will never complete. When the
*   I/O callback arrives, disable the external oscillator if you want the CPLD
*   clock to be at DC.
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
* Calling this function will enable or disable a given digit's IRQ shift register.
*
* @param  mask   Combination of flags to change.
* @param  state  Should the flags be cleared or set?
*/
void CPLDDriver::_digit_irq_force(uint8_t d, bool state) {
  if (state) {
    writeRegister(CPLD_REG_DIGIT_FORSAKE, (forsaken_digits | (1 << d)));
  }
  else {
    writeRegister(CPLD_REG_DIGIT_FORSAKE, (forsaken_digits & ~(1 << d)));
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


/*******************************************************************************
* This is where IRQ-related functions live.                                    *
*******************************************************************************/

/**
* Used by the ManuManager to easily check for a digit's IRQ validity.
* Pulls directly from the most-recent valid IRQ buffer, so this check is fairly
*   direct, and not reliant on IMU access.
*
* @param  x  The DigitPort (Not Anatomical!) in question.
* @return DigitState
*/
DigitState CPLDDriver::digitState(DigitPort x) {
  if (digitExists(x)) {
    // TODO: Digit asleep or not?
    return DigitState::AWAKE;
  }
  return DigitState::ABSENT;
}


/**
* Used by the ManuManager to easily check for a digit's IRQ validity.
* Pulls directly from the most-recent valid IRQ buffer, so this check is fairly
*   direct, and not reliant on IMU access.
*
* @return True if the IRQ data confirms a digit.
*/
bool CPLDDriver::digitExists(DigitPort x) {
  switch (x) {
    case DigitPort::MC:
    case DigitPort::PORT_1:
    case DigitPort::PORT_2:
    case DigitPort::PORT_3:
    case DigitPort::PORT_4:
    case DigitPort::PORT_5:
      // MC_PRESENT is bit 68 in the IRQ stream, other digits follow in numerical order.
      return irq_is_presently_high(68 + (uint8_t) x);
    default:
      return false;
  }
};


/**
* Processes the IRQ message from the CPLD. If the driver is also servicing IRQs,
*   per-IMU signal groups will be passed to the ManuManager.
*
* @return non-zero on error.
*/
int8_t CPLDDriver::iiu_group_irq() {
  int8_t return_value = 0;
  uint32_t _irq_time = micros();
  const char* l_2_h = "L->H";
  const char* h_2_l = "H->L";
  // This class cares about these IRQs...
  // 68  Metacarpals present.
  // 69  Digit 1 present.
  // 70  Digit 2 present.
  // 71  Digit 3 present.
  // 72  Digit 4 present.
  // 73  Digit 5 present.
  // 74  CONFIG register, bit 2.
  // 75  CPLD_OE
  // 76  Aggregated IRQ
  // 77  Aggregated IRQ

  if (_irq_accum[8] & 0x0F) {  // We only care about the bottom-half here.
    if (_irq_accum[8] & 0x08) {
      if (getVerbosity() > 4) local_log.concatf("MC PRESENT %s\n", irq_is_presently_high(68) ? l_2_h : h_2_l);
    }
    if (_irq_accum[8] & 0x04) {
      if (getVerbosity() > 4) local_log.concatf("Digit_1 %s\n", irq_is_presently_high(69) ? l_2_h : h_2_l);
    }
    if (_irq_accum[8] & 0x02) {
      if (getVerbosity() > 4) local_log.concatf("Digit_2 %s\n", irq_is_presently_high(70) ? l_2_h : h_2_l);
    }
    if (_irq_accum[8] & 0x01) {
      if (getVerbosity() > 4) local_log.concatf("Digit_3 %s\n", irq_is_presently_high(71) ? l_2_h : h_2_l);
    }
  }

  if (_irq_accum[9] & 0xFC) {  // We only care about the top-6.
    if (_irq_accum[9] & 0x80) {
      if (getVerbosity() > 4) local_log.concatf("Digit_4 %s\n", irq_is_presently_high(72) ? l_2_h : h_2_l);
    }
    if (_irq_accum[9] & 0x40) {
      if (getVerbosity() > 4) local_log.concatf("Digit_5 %s\n", irq_is_presently_high(73) ? l_2_h : h_2_l);
    }
    if (_irq_accum[9] & 0x20) {
      if (0 == _irq_latency_2) {
        _irq_latency_2 = _irq_time;
        if (getVerbosity() > 4) local_log.concatf("IRQ latency is %u us.\n", _irq_latency_2);
      }
      if (!hardwareReady()) {
        // We have verified that we can talk to the hardware.
        _er_set_flag(CPLD_FLAG_CPLD_READY);
        irq_service_enabled(true);
      }
    }
    if (_irq_accum[9] & 0x10) {
      if (getVerbosity() > 4) local_log.concatf("CPLD_OE %s.\n", irq_is_presently_high(75) ? l_2_h : h_2_l);
    }
    if (_irq_accum[9] & 0x08) {
      if (getVerbosity() > 4) local_log.concatf("IRQ76 %s.\n", irq_is_presently_high(76) ? l_2_h : h_2_l);
    }
    if (_irq_accum[9] & 0x04) {
      if (getVerbosity() > 4) local_log.concatf("IRQ77 %s.\n", irq_is_presently_high(77) ? l_2_h : h_2_l);
    }
  }

  if ((nullptr != _manu) && irq_service_enabled()) {
    uint8_t imu_offset = 0;
    // Now we will scan across the IMU signals looking for the data ready signals.
    // If all the present digits have their signals raised, we fire the frame
    //   read for that sensor aspect.
    uint16_t irq_svc_slice = irq_digit_slice_service(DigitPort::MC);
    uint16_t irq_dat_slice = irq_digit_slice_current(DigitPort::MC);
    if (irq_svc_slice & 0x0F) {
      // The first 4-bits of data are not subject to guards, since the IMU is on
      //   the PCB with the CPLD and shift register.
      _manu->deliverIRQ(DigitPort::MC, imu_offset, (irq_dat_slice & 0x0F), (irq_svc_slice & 0x0F));
    }
    imu_offset++;

    if (digitExists(DigitPort::MC)) {
      // The top 4-bits are only observed if the MC unit is connected.
      if (irq_svc_slice & 0xF0) {
        // Ship IRQ to MC IMU.
        _manu->deliverIRQ(DigitPort::MC, imu_offset, (irq_dat_slice & 0xF0) >> 4, (irq_svc_slice & 0xF0));
      }
    }
    imu_offset++;

    for (uint8_t i = 1; i < 6; i++) {
      // The other digits are loopable.
      const DigitPort port = (DigitPort) i;
      if (digitExists(port)) {
        irq_svc_slice = irq_digit_slice_service(port);
        irq_dat_slice = irq_digit_slice_current(port);

        if (irq_svc_slice) {
          const uint8_t p_svc = irq_svc_slice & 0x000F;
          const uint8_t i_svc = (irq_svc_slice >> 4) & 0x000F;
          const uint8_t d_svc = (irq_svc_slice >> 8) & 0x000F;
          if (d_svc) {  // Distal
            _manu->deliverIRQ(port, imu_offset+2, (irq_dat_slice >> 8) & 0x000F, d_svc);
          }
          if (i_svc) {  // Intermediate
            _manu->deliverIRQ(port, imu_offset+1, (irq_dat_slice >> 4) & 0x000F, i_svc);
          }
          if (p_svc) {  // Proximal
            _manu->deliverIRQ(port, imu_offset+0, irq_dat_slice & 0x000F, p_svc);
          }
        }
      }
      imu_offset += 3;
    }
  }

  // Mark everything as serviced.
  for (int i = 0; i < 10; i++) {
    _irq_accum[i] = 0;
  }
  flushLocalLog();
  return return_value;
}


/**
* Toggle the INT74 bit in the config register.
* This bit is off by default, and setting it will cause (in this order):
* 1) The IRQ aggregation machinary in the CPLD to send a frame, thus
*    allowing us to test for its proper operation when the message
*    arrives in the near-future.
* 2) The return bytes in the message carry version information and
*    initial configuration data. This allows us to check compatibility
*    and gives us proof of proper operation.
*/
void CPLDDriver::measure_irq_latency() {
  _irq_latency_0 = micros();
  _irq_latency_1 = 0;
  _irq_latency_2 = 0;
  setIRQ74(!getIRQ74());
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
int8_t CPLDDriver::attached() {
  if (EventReceiver::attached()) {
    init_ext_clk();

    _irq_data_arrival.repurpose(DIGITABULUM_MSG_IMU_IRQ_RAISED, (EventReceiver*) this);
    _irq_data_arrival.incRefs();
    _irq_data_arrival.specific_target = (EventReceiver*) this;
    _irq_data_arrival.priority(2);
    _irq_data_arrival.addArg((void*) _irq_accum, 10);

    _periodic_debug.repurpose(0x5080, (EventReceiver*) this);
    _periodic_debug.incRefs();
    _periodic_debug.specific_target = (EventReceiver*) this;
    _periodic_debug.priority(1);
    _periodic_debug.alterSchedulePeriod(100);
    _periodic_debug.alterScheduleRecurrence(-1);
    _periodic_debug.autoClear(false);
    _periodic_debug.enableSchedule(false);
    platform.kernel()->addSchedule(&_periodic_debug);

    bus_init();
    init_spi2(0, 0);  // CPOL=0, CPHA=0, HW-driven

    // An SPI transfer might hang (very unlikely). This will un-hang it.
    event_spi_timeout.repurpose(DIGITABULUM_MSG_SPI_TIMEOUT, (EventReceiver*) this);
    event_spi_timeout.incRefs();
    event_spi_timeout.specific_target = (EventReceiver*) this;
    event_spi_timeout.priority(6);
    event_spi_timeout.alterSchedulePeriod(bus_timeout_millis);
    event_spi_timeout.alterScheduleRecurrence(-1);
    event_spi_timeout.autoClear(false);
    event_spi_timeout.enableSchedule(false);
    //platform.kernel()->addSchedule(&event_spi_timeout);

    reset();

    if (255 != _pins.irq) {
      gpioDefine(_pins.irq, GPIOMode::INPUT_PULLUP);
      setPinFxn(_pins.irq, FALLING, cpld_wakeup_isr);
      //gpio_wakeup_enable((gpio_num_t) _pins.irq, GPIO_INTR_POSEDGE);
    }
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
int8_t CPLDDriver::callback_proc(ManuvrMsg* event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = (0 == event->refCount()) ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    case DIGITABULUM_MSG_IMU_IRQ_RAISED:
      break;
    case DIGITABULUM_MSG_SPI_QUEUE_READY:
      return_value = ((work_queue.size() > 0) || (nullptr != current_job)) ? EVENT_CALLBACK_RETURN_RECYCLE : return_value;
      break;
    case DIGITABULUM_MSG_SPI_CB_QUEUE_READY:
      return_value = (callback_queue.size() > 0) ? EVENT_CALLBACK_RETURN_RECYCLE : return_value;
      break;
    default:
      break;
  }

  return return_value;
}


int8_t CPLDDriver::notify(ManuvrMsg* active_event) {
  int8_t return_value = 0;
  switch (active_event->eventCode()) {
    /* General system events */
    case MANUVR_MSG_SYS_REBOOT:
    case MANUVR_MSG_SYS_BOOTLOADER:
      // If we are to go down for reboot, reset the CPLD.
      _deinit();
      return_value = 1;
      break;

    /* Things that only this class is likely to care about. */
    case DIGITABULUM_MSG_IMU_IRQ_RAISED:
      // We scan the IRQ list for signals we care about and let the ManuManager
      //   do the same.
      iiu_group_irq();
      return_value++;
      break;
    case DIGITABULUM_MSG_SPI_QUEUE_READY:
      advance_work_queue();
      return_value = 1;
      break;
    case DIGITABULUM_MSG_SPI_CB_QUEUE_READY:
      service_callback_queue();
      return_value = 1;
      break;
    case DIGITABULUM_MSG_SPI_TIMEOUT:
      if (timeout_punch) {
        if (current_job) {
          Kernel::log("callback_spi_timeout()\n");
          if (current_job->isComplete()) {
            advance_work_queue();
          }
          else {
            current_job->abort(XferFault::TIMEOUT);
          }
        }
      }
      else {
        timeout_punch = true;
      }
      return_value = 1;
      break;

    case DIGITABULUM_MSG_CPLD_RESET_CALLBACK:
      return_value = 1;
      if (getVerbosity() > 4) local_log.concat("CPLD reset. Testing IRQs...\n");
      // Release the reset pin and measure the IRQ latency and get CPLD version.
      setPin(_pins.reset, true);
      measure_irq_latency();
      break;
    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  flushLocalLog();
  return return_value;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void CPLDDriver::printDebug(StringBuilder* output) {
  EventReceiver::printDebug(output);
  //if (getVerbosity() > 6) output->concatf("-- volatile *cpld      0x%08x\n--\n", cpld);
  output->concatf("-- Conf                0x%02x\n",      cpld_conf_value);
  output->concatf("-- Osc (Int/Ext)       %s / %s\n",       (_er_flag(CPLD_FLAG_INT_OSC) ? "on":"off"), (_er_flag(CPLD_FLAG_EXT_OSC) ? "on":"off"));
  if (_er_flag(CPLD_FLAG_EXT_OSC)) {
  //  output->concatf("-- Base GetState       0x%02x\n", HAL_TIM_Base_GetState(&htim1));
    output->concatf("-- Ext freq:           %u\n", _ext_clk_freq);
  //  output->concatf("-- PWM GetState        0x%02x\n", HAL_TIM_PWM_GetState(&htim1));
  }

  output->concatf("-- DEN_AG (C/MC)       %s / %s\n", (_conf_bits_set(CPLD_CONF_BIT_DEN_AG_C) ? "on":"off"), (_conf_bits_set(CPLD_CONF_BIT_DEN_AG_MC) ? "on":"off"));
  output->concatf("-- CPLD_GPIO           %s\n", (_conf_bits_set(CPLD_CONF_BIT_GPIO) ? "hi":"lo"));
  output->concatf("-- Bus power conserve  %s\n", ((cpld_conf_value & CPLD_CONF_BIT_PWR_CONSRV) ? "on":"off"));
  output->concatf("-- Forsaken:           0x%02x\n--\n", forsaken_digits);
  output->concatf("-- IRQ76_reg:          0x%02x\n--\n", irq76_conf);

  if (getVerbosity() > 2) {
    output->concatf("-- Guarding queue      %s\n",   (_er_flag(CPLD_FLAG_QUEUE_GUARD)?"yes":"no"));
    output->concatf("-- spi_cb_per_event    %d\n\n", spi_cb_per_event);
  }
  printAdapter(output);
  output->concatf("-- callback q depth    %d\n--\n", callback_queue.size());

  if (getVerbosity() > 3) {
    printWorkQueue(output, CPLD_SPI_MAX_QUEUE_PRINT);
  }
  output->concat("\n\n");
}

/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void CPLDDriver::printIRQs(StringBuilder* output) {
  output->concat("\n---< IRQ Aggregator >--------------------\n");
  output->concatf("-- IRQ service         %sabled\n", (_er_flag(CPLD_FLAG_SVC_IRQS)?"en":"dis"));
  output->concatf("-- Constant scan       %s\n", (_conf_bits_set(CPLD_CONF_BIT_IRQ_SCAN) ? "on":"off"));
  if (cpld_wakeup_source & 0x80) {
    output->concatf("-- WAKEUP Signal       %d\n", (cpld_wakeup_source & 0x7F));
  }
  output->concatf("-- IRQ74 (conf/agg)    %c / %c\n",
    (_conf_bits_set(CPLD_CONF_BIT_IRQ_74) ? '1':'0'),
    (irq_is_presently_high(74) ? '1':'0')
  );
  output->concat("-- IRQ latency\n\tDispatch     0\n");
  output->concatf("\tArrival:     %u\n", wrap_accounted_delta(_irq_latency_0, _irq_latency_1));
  output->concatf("\tApplication  %u\n", wrap_accounted_delta(_irq_latency_0, _irq_latency_2));

  output->concatf("-- OE Pin              %c\n", (irq_is_presently_high(75) ? '1':'0'));
  output->concatf("-- Frames              %u\n", _irq_frames_rxd);

  output->concatf("-- Valid IRQ buffer    %d", _irq_data_ptr == _irq_data_0 ? 0 : 1);

  output->concat("\n--    _irq_data_0:     ");
  for (int i = 0; i < 10; i++) { output->concatf("%02x", _irq_data_0[i]); }
  output->concat("\n--    _irq_data_1:     ");
  for (int i = 0; i < 10; i++) { output->concatf("%02x", _irq_data_1[i]); }
  output->concat("\n--    _irq_diff:       ");
  for (int i = 0; i < 10; i++) { output->concatf("%02x", _irq_diff[i]); }
  output->concat("\n--    _irq_accum:      ");
  for (int i = 0; i < 10; i++) { output->concatf("%02x", _irq_accum[i]); }
  output->concat("\n\n");
}


#if defined(MANUVR_CONSOLE_SUPPORT)
/*******************************************************************************
* Console I/O
*******************************************************************************/

static const ConsoleCommand console_cmds[] = {
  { "o", "Enable or disable internal oscillator." },
  { "O", "Enable or disable external oscillator." }
};


uint CPLDDriver::consoleGetCmds(ConsoleCommand** ptr) {
  *ptr = (ConsoleCommand*) &console_cmds[0];
  return sizeof(console_cmds) / sizeof(ConsoleCommand);
}


void CPLDDriver::consoleCmdProc(StringBuilder* input) {
  char* str = input->position(0);
  int temp_int = ((*(str) != 0) ? atoi((char*) str+1) : 0);

  switch (*(str)) {
    case 'i':        // Info print
      switch (temp_int) {
        case 0:
          printIRQs(&local_log);
          break;
        case 1:
          getCPLDVersion();
          break;
        case 2:
          local_log.concatf("---< CPLD Pin assignments >--------------\n");
          local_log.concatf("\tspi1_cs    %d\tspi2_cs    %d\n", _pins.s1_cs, _pins.s2_cs);
          local_log.concatf("\tspi1_clk   %d\tspi2_clk   %d\n", _pins.s1_clk, _pins.s2_clk);
          local_log.concatf("\tspi1_mosi  %d\tspi2_mosi  %d\n", _pins.s1_mosi, _pins.s2_mosi);
          local_log.concatf("\tspi1_miso  %d\n\n", _pins.s1_miso);
          local_log.concatf("\treset  %d\tclk    %d\n", _pins.reset, _pins.clk);
          local_log.concatf("\treq    %d\toe     %d\n", _pins.req, _pins.oe);
          local_log.concatf("\tirq    %d\tgpio   %d\n\n", _pins.irq, _pins.gpio);
          break;
        case 3:
          local_log.concatf("---< Digit states >----------------------\n");
          local_log.concatf("\tmc:  %s\n",   digitStateToString(digitState(DigitPort::MC)));
          local_log.concatf("\t1:   %s\n",   digitStateToString(digitState(DigitPort::PORT_1)));
          local_log.concatf("\t2:   %s\n",   digitStateToString(digitState(DigitPort::PORT_2)));
          local_log.concatf("\t3:   %s\n",   digitStateToString(digitState(DigitPort::PORT_3)));
          local_log.concatf("\t4:   %s\n",   digitStateToString(digitState(DigitPort::PORT_4)));
          local_log.concatf("\t5:   %s\n\n", digitStateToString(digitState(DigitPort::PORT_5)));
          break;
        case 4:
          measure_irq_latency();
          break;
        case 5:
          printHardwareState(&local_log);
          break;
        case 6:
          local_log.concat("\n--    debug_buffer:\n");
          for (int i = 0; i < 32; i++) { local_log.concatf("%02x ", debug_buffer[i]); }
          break;
        default:
          printDebug(&local_log);
          break;
      }
      break;

    case 'b':
      timeout_punch = false;
      bus_timeout_millis = temp_int;
      if (0 == temp_int) {
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
      if (temp_int) SPIBusOp::spi_wait_timeout = temp_int * 10;
      local_log.concatf("SPIBusOp::spi_wait_timeout is %uuS...\n", SPIBusOp::spi_wait_timeout);
      break;
    case 'g':     // SPI1 queue-guard (overflow protection).
      _er_flip_flag(CPLD_FLAG_QUEUE_GUARD);
      local_log.concatf("CPLD guarding SPI queue from overflow?  %s\n", _er_flag(CPLD_FLAG_QUEUE_GUARD)?"yes":"no");
      break;
    case 'm':     // Set the number of callbacks per event.
      if (temp_int) spi_cb_per_event = temp_int;
      local_log.concatf("CPLD spi_cb_per_event:  %d\n", spi_cb_per_event);
      break;

    case 'o':        // CPLD internal oscillator.
    case 'O':        // CPLD external oscillator.
      local_log.concatf(
        "%sabling CPLD %sternal oscillator...\n",
        (temp_int ? "En" : "Dis"),
        ('O' == *(str) ? "ex" : "in")
      );
      if ('O' == *(str)) {
        externalOscillator(temp_int > 0);
      }
      else {
        internalOscillator(temp_int > 0);
      }
      break;

    case 'p':     // Purge the SPI1 work queue.
      purge_queued_work();
      local_log.concat("SPI1 queue purged.\n");
      break;

    case 'd':
      switch (temp_int) {
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

    case '%':   // Ext clock rate.
      if (setCPLDClkFreq(strict_max(temp_int, 1)*1000)) {
        local_log.concatf("Set ext clock period to %d kHz.\n", strict_max(temp_int, 1));
      }
      else {
        local_log.concat("Failed to set ext clock period.\n");
      }
      break;

    case '^':
      local_log.concatf("WAKEUP ISR bound to IRQ signal %d.\n", temp_int);
      setWakeupSignal(temp_int);
      break;
    case 'X':
    case 'x':
      local_log.concatf("_digit_irq_force(%u, %c)\n", temp_int, (*(str) == 'X' ? '1' : '0'));
      _digit_irq_force(temp_int, (*(str) == 'X'));
      break;
    //case 'E':
    //case 'e':
    //  local_log.concatf("%s IRQ 74.\n", (*(str) == 'E' ? "Setting" : "Clearing"));
    //  setIRQ74(*(str) == 'E');
    //  break;
    case 'A':
    case 'a':
      local_log.concatf("IRQ scanning %sabled.\n", (*(str) == 'A' ? "Dis" : "En"));
      disableIRQScan(*(str) == 'A');
      break;
    case ':':
    case ';':
      local_log.concatf("%sabling bus power conservation.\n", (*(str) == ':' ? "En" : "Dis"));
      setCPLDConfig(CPLD_CONF_BIT_PWR_CONSRV, (*(str) == ':'));
      break;
    case '+':
    case '=':
      local_log.concatf("%s CPLD_GPIO.\n", (*(str) == '+' ? "Setting" : "Clearing"));
      setGPIO(*(str) == '+');
      break;
    case '[':
    case '{':
      local_log.concatf("DEN_AG Carpals(%s)\n", (*(str) == '{' ? "true" : "false"));
      enableCarpalAG(*(str) == '{');
      break;
    case ']':
    case '}':
      local_log.concatf("DEN_AG Metacarpals(%s)\n", (*(str) == '}' ? "true" : "false"));
      enableMetacarpalAG(*(str) == '}');
      break;

    case 'r':  /* Reset the CPLD. */
      reset();
      break;


    case 'W':  // TODO: Cut once system is fully validated.
    case 'w':
      local_log.concatf("%sabling transfer alignment.\n", (*(str) == 'W' ? "En" : "Dis"));
      setCPLDConfig(CPLD_CONF_BIT_ALIGN_XFER, (*(str) == 'W'));
      break;
    case '_':  // TODO: Cut once system is fully validated.
    case '-':
      local_log.concatf("op_abuse_test <--- (%s)\n", (*(str) == '_' ? "true" : "false"));
      op_abuse_test = (*(str) == '_');
      break;
    case 'Z':  // TODO: Cut once system is fully validated.
    case 'z':
      if (temp_int) {
        _periodic_debug.alterSchedulePeriod(temp_int * 10);
      }
      _periodic_debug.enableSchedule(*(str) == 'Z');
      local_log.concatf("%s periodic reader.\n", (*(str) == 'z' ? "Stopping" : "Starting"));
      break;
    case '&':  // TODO: Cut once system is fully validated.
      local_log.concatf("Advanced CPLD SPI work queue.\n");
      Kernel::raiseEvent(DIGITABULUM_MSG_SPI_QUEUE_READY, nullptr);   // Raise an event
      break;

    case 'K':  // TODO: Cut once system is fully validated.
    case 'k':  // TODO: Cut once system is fully validated.
      irq_service_enabled(*(str) == 'K');
      local_log.concatf("CPLD servicing IRQs?  %s\n", irq_service_enabled()?"yes":"no");
      break;

    //case 's':  // TODO: Cut once system is fully validated.
    //  switch (temp_int) {
    //    case 1:     // SPI1 initialization...
    //      init_spi(1, 0);  // CPOL=1, CPHA=0, HW-driven
    //      local_log.concat("Re-initialized SPI1.\n");
    //      break;
    //    case 2:     // SPI2 initialization...
    //      init_spi2(1, 0);  // CPOL=1, CPHA=0, HW-driven
    //      local_log.concat("Re-initialized SPI2 into Mode-2.\n");
    //      break;
    //  }
    //  break;
    case '~':   // CPLD debug
      {
        bzero(&debug_buffer[0], 64);
        SPIBusOp* op = new_op(BusOpcode::RX, this);
        uint8_t imu_num = ((*(str) == '~') ? CPLD_REG_IMU_DM_P_M : CPLD_REG_IMU_DM_P_I) | 0x80;
        op->setParams(imu_num, 1, 1, 0x8F);
        op->setBuffer(&debug_buffer[0], 2);
        queue_io_job(op);
      }
      break;
    case '`':   // CPLD debug
      {
        bzero(&debug_buffer[0], 64);
        SPIBusOp* op = new_op(BusOpcode::RX, this);
        uint8_t imu_num = ((*(str) == '~') ? CPLD_REG_IMU_DM_P_M : CPLD_REG_IMU_DM_P_I) | 0x80;
        op->setParams(imu_num, 1, 28, 0x8F);
        op->setBuffer(&debug_buffer[0], 2);
        queue_io_job(op);
      }
      break;

    default:
      //EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT
