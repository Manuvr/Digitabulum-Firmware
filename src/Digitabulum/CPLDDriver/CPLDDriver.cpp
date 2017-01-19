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

/* This message is dispatched when IRQ data changes. */
ManuvrMsg _irq_data_arrival;

// These are debug. Cut them.
uint8_t active_imu_position = 0;


/**
* ISR for CPLD GPIO.
*/
void cpld_gpio_isr_0() {
  Kernel::log((readPin(75) ? "CPLD_GPIO_0 HIGH\n":"CPLD_GPIO_0 LOw\n"));
}

/**
* ISR for CPLD GPIO.
*/
void cpld_gpio_isr_1() {
  Kernel::log((readPin(78) ? "CPLD_GPIO_1 HIGH\n":"CPLD_GPIO_1 LOw\n"));
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
uint8_t CPLDDriver::cpld_version       = 0;  // CPLD version byte.
uint8_t CPLDDriver::cpld_conf_value    = 0;  // Configuration.
uint8_t CPLDDriver::forsaken_digits    = 0;  // Forsaken digits.
uint8_t CPLDDriver::cpld_wakeup_source = 0;  // WAKEUP mapping.

SPIBusOp  CPLDDriver::preallocated_bus_jobs[CPLD_SPI_PREALLOC_COUNT];

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
CPLDDriver::CPLDDriver(const CPLDPins* p) : EventReceiver("CPLDDriver"), BusAdapter(CPLD_SPI_MAX_QUEUE_DEPTH) {
  memcpy(&_pins, p, sizeof(CPLDPins));

  if (nullptr == cpld) {
    cpld = this;
    ManuvrMsg::registerMessages(cpld_message_defs, sizeof(cpld_message_defs) / sizeof(MessageTypeDef));
  }

  // Build some pre-formed Events.
  event_spi_callback_ready.repurpose(DIGITABULUM_MSG_SPI_CB_QUEUE_READY, (EventReceiver*) this);
  event_spi_callback_ready.incRefs();
  event_spi_callback_ready.specific_target = (EventReceiver*) this;
  event_spi_callback_ready.priority(5);

  SPIBusOp::event_spi_queue_ready.repurpose(DIGITABULUM_MSG_SPI_QUEUE_READY, (EventReceiver*) this);
  SPIBusOp::event_spi_queue_ready.incRefs();
  SPIBusOp::event_spi_queue_ready.specific_target    = (EventReceiver*) this;
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
  if (255 != _pins.reset) {
    gpioDefine(_pins.reset, OUTPUT);
    setPin(_pins.reset, false);  // Hold the CPLD in reset.
  }
  if (255 != _pins.tx_rdy) {
    gpioDefine(_pins.tx_rdy, OUTPUT);
    setPin(_pins.tx_rdy, false);
  }
  if (255 != _pins.irq) {
    setPinFxn(_pins.irq, FALLING, cpld_wakeup_isr);
  }
  if (255 != _pins.gpio0) {
    //setPinFxn(_pins.gpio0, CHANGE, cpld_gpio_isr_0);
  }
  if (255 != _pins.gpio1) {
    setPinFxn(_pins.gpio1, CHANGE, cpld_gpio_isr_1);
  }
  if (255 != _pins.den) {
    gpioDefine(_pins.den, OUTPUT);
    setPin(_pins.den, true);
  }
}


/**
* Resets the CPLD, and any registers within it.
* Any host-controlled pins affecting CPLD config are not altered by this call,
*   so it might be worth disabling some IRQs prior to doing something that might
*   make them seizure.
*/
void CPLDDriver::reset() {
  setPin(_pins.reset, false);  // Drive the reset pin low...
  externalOscillator(true);    // Turn on the default oscillator...
  cpld_conf_value    = 0x00;   // Set our register representations to their
  cpld_version       = 0x00;   //   default values.
  cpld_wakeup_source = 0x00;
  forsaken_digits    = 0x00;
  bus_timeout_millis = 5;   // TODO: Dynamically relate this to clock frequency.

  for (int z = 0; z < 30; z++) _irq_data[z] = 0;   // Wipe the IRQ data.
  _irq_data_ptr = _irq_data_0;  // Used for block-wise access.

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
      {
        uint8_t _version = op->getTransferParam(3);
        if (getVerbosity() > 3) local_log.concatf("CPLD r%d.\n", _version);
        if ((0 != _version) && (0xFF != _version)) {
          if (_version != cpld_version) {
            Kernel::raiseEvent(DIGITABULUM_MSG_CPLD_RESET_COMPLETE, nullptr);
          }
        }
        else {
          if (getVerbosity() > 1) local_log.concatf("CPLD returned a bad version code: 0x%02x\n", cpld_version);
        }
      }
      break;
    case CPLD_REG_CONFIG:
      _process_conf_update(op->getTransferParam(1));
      break;
    case CPLD_REG_WAKEUP_IRQ:
      cpld_wakeup_source = op->getTransferParam(1);
      break;
    case CPLD_REG_DIGIT_FORSAKE:
      forsaken_digits = op->getTransferParam(1);
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
    op->setCSPin(_pins.tx_rdy);

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
        Kernel::log("####################### advance_operation entry\n");
        current_job->advance_operation(0, 0);
        local_log.concat("####################### advance_operation exit\n");
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
  SPIBusOp* return_value = BusAdapter::new_op();
  return_value->setCSPin(_pins.tx_rdy);
  return_value->csActiveHigh(true);
  return return_value;
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
* Change the frequency of the timer-generated external CPLD clock.
*
* @param  _freq  The desired frequency, in Hz.
* @return 0 on success. Nonzero on failure.
*/
int CPLDDriver::setCPLDClkFreq(int _period) {
  return (_set_timer_base((uint16_t) _period) ? 1 : 0);
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


DigitState CPLDDriver::digitState(DigitPort x) {
  if (digitExists(x)) {
    // Digit asleep or not?
    return DigitState::AWAKE;
  }
  return DigitState::ABSENT;
}


/*******************************************************************************
* This is where IMU-related functions live.                                    *
*******************************************************************************/

// NO ERROR CHECKING! Don't call this with an argument >79.
bool irq_is_presently_high(uint8_t bit) {
  const uint8_t bit_offset  = bit & 0x07;  // Cheaper than modulus 8.
  const uint8_t byte_offset = bit >> 3;    // Cheaper than div by 8.
  return ((_irq_data_ptr[byte_offset] & (0x01 << bit_offset)) != 0);
}

// NO ERROR CHECKING! Don't call this with an argument >79.
bool irq_demands_service(uint8_t bit) {
  const uint8_t bit_offset  = bit & 0x07;  // Cheaper than modulus 8.
  const uint8_t byte_offset = bit >> 3;    // Cheaper than div by 8.
  return ((_irq_accum[byte_offset] & (0x01 << bit_offset)) != 0);
}


/**
*
*
* @return The index of the IMU with an outstanding IRQ.
*/
int8_t CPLDDriver::iiu_group_irq() {
  int8_t return_value = 0;

  // This class cares about these IRQs...
  // 68  Metacarpals present.
  // 69  Digit 1 present.
  // 70  Digit 2 present.
  // 71  Digit 3 present.
  // 72  Digit 4 present.
  // 73  Digit 5 present.
  // 74  CONFIG register, bit 2.
  // 75  CPLD_OE

  // TODO: Next CPLD revision should align these bits better.
  if (_irq_accum[8] & 0xF0) {  // We only care about the upper-half here.
    uint8_t reset_bits = 0xFF;   // These are bit we wish to preserve.
    if (_irq_accum[8] & 0x10) {
      reset_bits &= ~0x10;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): MC PRESENT %s\n", irq_is_presently_high(68) ? "L->H":"H->L");
    }
    if (_irq_accum[8] & 0x20) {
      reset_bits &= ~0x20;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): Digit_1 %s\n", irq_is_presently_high(69) ? "L->H":"H->L");
    }
    if (_irq_accum[8] & 0x40) {
      reset_bits &= ~0x40;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): Digit_2 %s\n", irq_is_presently_high(70) ? "L->H":"H->L");
    }
    if (_irq_accum[8] & 0x80) {
      reset_bits &= ~0x80;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): Digit_3 %s\n", irq_is_presently_high(71) ? "L->H":"H->L");
    }
    _irq_accum[8] &= reset_bits;
  }

  if (_irq_accum[9] & 0x0F) {  // We only care about the lower-half here.
    uint8_t reset_bits = 0xFF;   // These are bit we wish to preserve.
    if (_irq_accum[9] & 0x01) {
      reset_bits &= ~0x01;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): Digit_4 %s\n", irq_is_presently_high(72) ? "L->H":"H->L");
    }
    if (_irq_accum[9] & 0x02) {
      reset_bits &= ~0x02;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): Digit_5 %s\n", irq_is_presently_high(73) ? "L->H":"H->L");
    }
    if (_irq_accum[9] & 0x04) {
      reset_bits &= ~0x04;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): CONFIG[2] %s\n", irq_is_presently_high(74) ? "L->H":"H->L");
    }
    if (_irq_accum[9] & 0x08) {
      reset_bits &= ~0x08;
      if (getVerbosity() > 4) local_log.concatf("iiu_group_irq(): CPLD_OE %s.\n", irq_is_presently_high(75) ? "L->H":"H->L");
    }
    _irq_accum[9] &= reset_bits;  // Clear serviced bits.
  }
  flushLocalLog();
  return return_value;
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
      // MC_PRESENT is bit 68 in the IRQ stream.
      return irq_is_presently_high(68 + (uint8_t) x);
    default:
      return false;
  }
};



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

    gpioSetup();
    bus_init();

    init_spi2(1, 0);  // CPOL=1, CPHA=0, HW-driven

    init_ext_clk();

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
      // Wipe the service array. TODO: This is not concurrency-safe.
      for (int i = 0; i < 10; i++) { _irq_accum[i] = 0; }
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
    case MANUVR_MSG_INTERRUPTS_MASKED:
      break;
    case MANUVR_MSG_SYS_REBOOT:
    case MANUVR_MSG_SYS_BOOTLOADER:
      // If we are to go down for reboot, reset the CPLD.
      _deinit();
      return_value = 1;
      break;

    case 0x5050:
      //{
      //  SPIBusOp* op = new_op(BusOpcode::RX, this);
      //  op->setParams((active_imu_position | 0x80), 0x01, 0x02, 0x8F);
      //  op->setBuffer(__hack_buffer, 2);
      //  queue_io_job((BusOp*) op);
      //}
      return_value = 1;
      break;

    /* Things that only this class is likely to care about. */
    case DIGITABULUM_MSG_IMU_IRQ_RAISED:
      // We scan the IRQ list for signals we care about and let the ManuManager
      //   do the same.
      iiu_group_irq();
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
      setPin(_pins.reset, true);
      //if (getVerbosity() > 4) local_log.concat("CPLD reset.\n");
      return_value = 1;
      //getCPLDVersion();
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
void CPLDDriver::printDebug(StringBuilder *output) {
  EventReceiver::printDebug(output);
  //if (getVerbosity() > 6) output->concatf("-- volatile *cpld      0x%08x\n--\n", cpld);
  output->concatf("-- Conf                0x%02x\n",      cpld_conf_value);
  output->concatf("-- Osc (Int/Ext)       %s / %s\n",       (_er_flag(CPLD_FLAG_INT_OSC) ? "on":"off"), (_er_flag(CPLD_FLAG_EXT_OSC) ? "on":"off"));
  //if (_er_flag(CPLD_FLAG_EXT_OSC)) {
  //  output->concatf("-- Base GetState       0x%02x\n", HAL_TIM_Base_GetState(&htim1));
  //  output->concatf("-- PWM GetState        0x%02x\n", HAL_TIM_PWM_GetState(&htim1));
  //}
  output->concatf("-- DEN_AG Main         %s\n", (_er_flag(CPLD_FLAG_DEN_AG_STATE) ? "on":"off"));
  output->concatf("-- Bus power conserve  %s\n", ((cpld_conf_value & CPLD_CONF_BIT_PWR_CONSRV) ? "on":"off"));
  if (cpld_wakeup_source & 0x80) {
    output->concatf("-- WAKEUP Signal       %d\n", (cpld_wakeup_source & 0x7F));
  }

  output->concatf("--\n-- CPLD_GPIO (0/1)     %s / %s\n--\n",       (readPin(75) ? "hi":"lo"), (readPin(78) ? "hi":"lo"));

  printHardwareState(output);
  if (getVerbosity() > 2) {
    output->concatf("-- Guarding queue      %s\n",       (_er_flag(CPLD_FLAG_QUEUE_GUARD)?"yes":"no"));
    output->concatf("-- spi_cb_per_event    %d\n--\n",   spi_cb_per_event);
  }
  printAdapter(output);
  output->concatf("-- callback q depth    %d\n--\n", callback_queue.size());

  if (getVerbosity() > 3) {
    printWorkQueue(output, CPLD_SPI_MAX_QUEUE_PRINT);
  }

  output->concatf("--\n-- Valid IRQ buffer:   %d\n", _irq_data_ptr == _irq_data_0 ? 0 : 1);
  output->concatf("-- IRQ service:        %sabled", (_er_flag(CPLD_FLAG_SVC_IRQS)?"en":"dis"));
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
void CPLDDriver::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = ((*(str) != 0) ? atoi((char*) str+1) : 0);

  switch (*(str)) {
    case 'i':        // Readback test
      switch (temp_byte) {
        case 1:
          getCPLDVersion();
          break;
        case 2:
          local_log.concatf("---< CPLD Pin assignments >--------------\n");
          local_log.concatf("-- reset        %d\n", _pins.reset);
          local_log.concatf("-- tx_rdy       %d\n", _pins.tx_rdy);
          local_log.concatf("-- irq          %d\n", _pins.irq);
          local_log.concatf("-- gpio0        %d\n", _pins.gpio0);
          local_log.concatf("-- gpio1        %d\n", _pins.gpio1);
          local_log.concatf("-- den          %d\n\n", _pins.den);
          break;
        case 3:
          local_log.concatf("---< Digit states >----------------------\n");
          local_log.concatf("-- mc:  %s\n",   digitStateToString(digitState(DigitPort::MC)));
          local_log.concatf("-- 1:   %s\n",   digitStateToString(digitState(DigitPort::PORT_1)));
          local_log.concatf("-- 2:   %s\n",   digitStateToString(digitState(DigitPort::PORT_2)));
          local_log.concatf("-- 3:   %s\n",   digitStateToString(digitState(DigitPort::PORT_3)));
          local_log.concatf("-- 4:   %s\n",   digitStateToString(digitState(DigitPort::PORT_4)));
          local_log.concatf("-- 5:   %s\n\n", digitStateToString(digitState(DigitPort::PORT_5)));
          break;
        default:
          printDebug(&local_log);
          break;
      }
      break;

    case 's':     // SPI1 initialization...
      switch (temp_byte) {
        case 1:
          init_spi(1, 0);  // CPOL=1, CPHA=0, HW-driven
          local_log.concat("Re-initialized SPI1.\n");
          break;
        case 2:
          init_spi2(1, 0);  // CPOL=1, CPHA=0, HW-driven
          local_log.concat("Re-initialized SPI2 into Mode-2.\n");
          break;
      }
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

    case 'o':        // CPLD internal oscillator.
    case 'O':        // CPLD external oscillator.
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

    case '&':
      local_log.concatf("Advanced CPLD SPI work queue.\n");
      Kernel::raiseEvent(DIGITABULUM_MSG_SPI_QUEUE_READY, nullptr);   // Raise an event
      break;

    case '%':   // Ext clock rate.
      temp_byte = temp_byte % 16;
      if (setCPLDClkFreq(1 << temp_byte)) {
        local_log.concatf("Set ext clock period to %d.\n", 1 << temp_byte);
      }
      else {
        local_log.concat("Failed to set ext clock period.\n");
      }
      break;

    case '^':
      local_log.concatf("WAKEUP ISR bound to IRQ signal %d.\n", temp_byte);
      setWakeupSignal(temp_byte);
      break;
    case 'E':
    case 'e':
      local_log.concatf("%s IRQ 74.\n", (*(str) == '_' ? "Clearing" : "Setting"));
      setCPLDConfig(CPLD_CONF_BIT_IRQ_74, (*(str) == 'E'));
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

    case '_':
    case '-':
      local_log.concatf("enableCarpalAG(%s)\n", (*(str) == '-' ? "true" : "false"));
      enableCarpalAG(*(str) == '-');
      break;

    case '+':
    case '=':
      local_log.concatf("enableMetacarpalAG(%s)\n", (*(str) == '+' ? "true" : "false"));
      enableMetacarpalAG(*(str) == '+');
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

    case 'r':
      reset();
      break;

    case 'Z':
    case 'z':
      if (temp_byte) {
        _periodic_debug.alterSchedulePeriod(temp_byte * 10);
      }
      _periodic_debug.enableSchedule(*(str) == 'Z');
      local_log.concatf("%s periodic reader.\n", (*(str) == 'z' ? "Stopping" : "Starting"));
      break;

    //case 'C':    // Individual IMU access tests...
    //case 'c':    // Individual IMU access tests...
    //  if (temp_byte < 0x22) {
    //    active_imu_position = temp_byte;
    //    for (int z = 0; z < 34; z++) __hack_buffer[z] = 0;
    //    SPIBusOp* op = new_op(BusOpcode::RX, this);
    //    op->setParams((temp_byte | 0x80), 0x01, 0x01, 0x8F);
    //    op->setBuffer(__hack_buffer, (*(str) == 'C' ? 5 : 1));
    //    queue_io_job((BusOp*) op);
    //  }
    //  else {
    //    local_log.concat("IMU out of bounds.\n");
    //  }
    //  break;

    //case 'n':    // Many bytes for a given address...
    //  if (temp_byte < 35) {
    //    for (int z = 0; z < 34; z++) __hack_buffer[z] = 0;
    //    SPIBusOp* op = new_op(BusOpcode::RX, this);
    //    op->setParams((active_imu_position | 0x80), temp_byte, 0x01, 0x8F);
    //    op->setBuffer(__hack_buffer, temp_byte);
    //    queue_io_job((BusOp*) op);
    //  }
    //  else {
    //    local_log.concat("Length out of bounds.\n");
    //  }
    //  break;

    //case 'N':    // Single byte for a multiple access...
    //  if (temp_byte < 35) {
    //    for (int z = 0; z < 34; z++) __hack_buffer[z] = 0;
    //    SPIBusOp* op = new_op(BusOpcode::RX, this);
    //    op->setParams((active_imu_position | 0x80), 0x01, temp_byte, 0x8F);
    //    op->setBuffer(__hack_buffer, temp_byte);
    //    queue_io_job((BusOp*) op);
    //  }
    //  else {
    //    local_log.concat("Length out of bounds.\n");
    //  }
    //  break;

    default:
      EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT
