/*
File:   ManuManager.cpp
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

#include "ManuManager.h"


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

SPIBusOp ManuManager::_preformed_read_i;
SPIBusOp ManuManager::_preformed_read_m;
SPIBusOp ManuManager::_preformed_read_temp;
SPIBusOp ManuManager::_preformed_fifo_read;

Vector3<int16_t> ManuManager::reflection_mag;
Vector3<int16_t> ManuManager::reflection_acc;
Vector3<int16_t> ManuManager::reflection_gyr;

ManuManager* ManuManager::INSTANCE = nullptr;

ManuManager* ManuManager::getInstance() {
  return INSTANCE;
}

/* ---------------------- */
/*    Register memory     */
/* ---------------------- */
/* These are giant strips of DMA-capable memory that are used for raw frame
     reads from the sensor package. */

/* Inertial data,
    x2 because Acc+Gyr
    x2 because double-buffered
    x3 because 3-space vectors (of 16-bit ints)
    X17 because that many sensors
    = 204 int16's
    = 408 bytes
*/
int16_t _frame_buf_i[2 * 2 * 3 * LEGEND_DATASET_IIU_COUNT];


/* Temperature data. Single buffered. */
// TODO: Might consolidate temp into inertial. Sensor and CPLD allow for it.
int16_t __temperatures[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer data registers. Single buffered. */
int16_t _reg_block_m_data[3 * LEGEND_DATASET_IIU_COUNT];

/* More large stretches of DMA memory. These are for IIU register definitions.
     Registers laid out this way cannot be multiply-accessed as more than single bytes
     by their respective IMU classes because the memory is not contiguous. */

/* Identity registers for both sensor aspects. */
uint8_t _reg_block_ident[2 * LEGEND_DATASET_IIU_COUNT];


/* Ranked-access registers below this line.*/
  // TODO: Until the DMA apparatus is smart enough to know what we're doing,
  //   we actually need to define all 6 bytes. We should be able to loop the
  //   memory-side of the transaction if we want all bytes to be the same.

/*
  AG_INT1_CTRL and AG_INT2_CTRL (Rank-access)
  3 ranks times 2 byte-wide registers.
  INT1 will service re-scaling, and INT2 will service the FIFO.
  TODO: Might merge this into a single register 16-bits wide?
*/
uint8_t _reg_block_ag_interrupt_conf[6] = {0xC0, 0x08, 0xC0, 0x08, 0xC0, 0x08};

/*
  G_ORIENT_CFG(Rank-access)
  3 ranks times 1 byte-wide register.
  Great pains were taken to enforce a common orientation in hardware.
*/
uint8_t _reg_block_orient_cfg[3] = {0x00, 0x00, 0x00};

/*
  Deals with IRQ latching, axis enablement, and 4D. (Rank-access)
  3 ranks times 2 byte-wide registers.
  ctrl4: All gyro axis enabled, IRQ not latched, no 4D.
  ctrl5: No acc decimation, all axis enabled.
  TODO: Might merge this into a single register 16-bits wide?
*/
uint8_t _reg_block_ag_ctrl4_5[6] = {0x38, 0x38, 0x38, 0x38, 0x38, 0x38};

/*
  IRQ pin control, FIFO and bus config, self-test. (Rank-access)
  3 ranks times 3 byte-wide registers.
  ctrl8:  BDU=1, active-hi push-pull IRQ pins, 4-wire SPI w/auto-inc, LE.
  ctrl9:  No temp in FIFO, i2c, or DRDY. FIFO on and unrestricted.
  ctrl10: Self-tests disabled.
*/
uint8_t _reg_block_ag_ctrl8_9_10[9] = {
  0x44, 0x06, 0x00,
  0x44, 0x06, 0x00,
  0x44, 0x06, 0x00
};

/*
  Inertial aspect FIFO control registers. (Rank-access)
  3 ranks times 1 byte-wide register.
  FIFO in continuous mode, with a threshold of 4.
*/
uint8_t _reg_block_fifo_ctrl[3] = {0xC4, 0xC4, 0xC4};

/*
  Magnetometer interrupt config registers. (Rank-access)
  3 ranks times 1 byte-wide register.
  Non-latched, active-high IRQ enabled for all axes.
*/
uint8_t _reg_block_m_irq_cfg[3] = {0xE5, 0xE5, 0xE5};

/*
  Magnetometer ctrl1 registers. (Rank-access)
  3 ranks times 1 byte-wide register.
  ctrl1: Temp-compensated, med performance, 5hz, no self-test
*/
uint8_t _reg_block_m_ctrl1[3] = {0xAC, 0xAC, 0xAC};

/*
  Magnetometer ctrl3-5 registers. (Rank-access)
  3 ranks times 3 byte-wide registers.
  ctrl3:  No i2c, no LP, SPI I/O, continuous conversion.
  ctrl4:  Little-endian, med performance
  ctrl5:  Block-update.
*/
uint8_t _reg_block_m_ctrl3_5[9] = {
  0x84, 0x04, 0x40,
  0x84, 0x04, 0x40,
  0x84, 0x04, 0x40
};



/* Individually-packed registers below this line. */

/* Activity thresholds and durations. */
uint8_t _reg_block_ag_activity[2 * LEGEND_DATASET_IIU_COUNT];

/* Accelerometer interrupt settings, thresholds, G_REFERENCE */
uint8_t _reg_block_ag_0[6 * LEGEND_DATASET_IIU_COUNT];

/* Inertial aspect control registers. */
uint8_t _reg_block_ag_ctrl1_3[3 * LEGEND_DATASET_IIU_COUNT];
uint8_t _reg_block_ag_ctrl6_7[2 * LEGEND_DATASET_IIU_COUNT];


/* Inertial aspect status registers. */
uint8_t _reg_block_ag_status[LEGEND_DATASET_IIU_COUNT];

/* Accelerometer IRQ status registers. */
uint8_t _reg_block_a_irq_src[LEGEND_DATASET_IIU_COUNT];



/* Inertial aspect FIFO status registers. */
uint8_t __fifo_levels[LEGEND_DATASET_IIU_COUNT];

/* Gyroscope interrupt config, source, and threshold registers. */
uint8_t _reg_block_g_irq_src[LEGEND_DATASET_IIU_COUNT];
uint8_t _reg_block_g_irq_cfg[LEGEND_DATASET_IIU_COUNT];
int16_t _reg_block_g_thresholds[3 * LEGEND_DATASET_IIU_COUNT];
uint8_t _reg_block_g_irq_dur[LEGEND_DATASET_IIU_COUNT];


/* Magnetometer offset registers. */
int16_t _reg_block_m_offsets[3 * LEGEND_DATASET_IIU_COUNT];

/* Magnetometer control registers. */
uint8_t _reg_block_m_ctrl2[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer status registers. */
uint8_t  _reg_block_m_status[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer interrupt source registers. */
uint8_t _reg_block_m_irq_src[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer threshold registers. Will be interpreted as 15-bit unsigned. */
uint16_t _reg_block_m_thresholds[LEGEND_DATASET_IIU_COUNT];
/* ---------------------- */
/* End of register memory */
/* ---------------------- */

/* These are the beginnings of a ring-buffer for whole inertial frames. */
const int16_t* _frames_i[] = {
  &_frame_buf_i[0],
  &_frame_buf_i[102]
};


const RegPtrMap _reg_ptrs[] = {
  RegPtrMap(0 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(1 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(2 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(3 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(4 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(5 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(6 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(7 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(8 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(9 , &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(10, &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(11, &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(12, &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(13, &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(14, &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(15, &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0]),
  RegPtrMap(16, &_reg_block_ag_activity[0], &_reg_block_ag_0[0], &_reg_block_ag_ctrl1_3[0], &_reg_block_ag_ctrl6_7[0], &_reg_block_ag_status[0])
};


LSM9DS1 imus[LEGEND_DATASET_IIU_COUNT] = {
  LSM9DS1(&_reg_ptrs[0 ]),
  LSM9DS1(&_reg_ptrs[1 ]),
  LSM9DS1(&_reg_ptrs[2 ]),
  LSM9DS1(&_reg_ptrs[3 ]),
  LSM9DS1(&_reg_ptrs[4 ]),
  LSM9DS1(&_reg_ptrs[5 ]),
  LSM9DS1(&_reg_ptrs[6 ]),
  LSM9DS1(&_reg_ptrs[7 ]),
  LSM9DS1(&_reg_ptrs[8 ]),
  LSM9DS1(&_reg_ptrs[9 ]),
  LSM9DS1(&_reg_ptrs[10]),
  LSM9DS1(&_reg_ptrs[11]),
  LSM9DS1(&_reg_ptrs[12]),
  LSM9DS1(&_reg_ptrs[13]),
  LSM9DS1(&_reg_ptrs[14]),
  LSM9DS1(&_reg_ptrs[15]),
  LSM9DS1(&_reg_ptrs[16])
};


// This is used to define the noise floors for the data.
//int16_t noise_floor_mag[3*LEGEND_DATASET_IIU_COUNT];
//int16_t noise_floor_acc[3*LEGEND_DATASET_IIU_COUNT];
//int16_t noise_floor_gyr[3*LEGEND_DATASET_IIU_COUNT];


/*
* The following sensor registers are managed entirely within ManuManager. For
*   those registers that we treat as write-only, and homogenous, we will use the
*   ranked-access mode in the CPLD.

  A_DATA_X,           // 16-bit accelerometer data registers
  A_DATA_Y,           // 16-bit accelerometer data registers
  A_DATA_Z,           // 16-bit accelerometer data registers
  AG_FIFO_CTRL,
  AG_FIFO_SRC,
  G_INT_GEN_CFG,
  G_INT_GEN_THS_X,    // 16-bit threshold registers
  G_INT_GEN_THS_Y,    // 16-bit threshold registers
  G_INT_GEN_THS_Z,    // 16-bit threshold registers
  G_INT_GEN_DURATION
*/


const char* ManuManager::chiralityString(Chirality x) {
  switch (x) {
    case Chirality::RIGHT:  return "RIGHT";
    case Chirality::LEFT:   return "LEFT";
    case Chirality::UNKNOWN:
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
ManuManager::ManuManager(BusAdapter<SPIBusOp>* bus) : EventReceiver("ManuMgmt") {
  _bus = (CPLDDriver*) bus;  // TODO: Make this cast unnecessary.
  INSTANCE = this;

  reflection_mag.x = 1;
  reflection_mag.y = 1;
  reflection_mag.z = -1;

  reflection_acc.x = -1;
  reflection_acc.y = 1;
  reflection_acc.z = -1;

  reflection_gyr.x = -1;
  reflection_gyr.y = 1;
  reflection_gyr.z = -1;

  _preformed_read_i.shouldReap(false);
  _preformed_read_i.devRegisterAdvance(true);
  _preformed_read_i.set_opcode(BusOpcode::RX);
  _preformed_read_i.callback = (BusOpCallback*) this;
  // Starting from the first accelerometer...
  // Read 12 bytes...  (A and G vectors)
  // ...across 17 sensors...
  // ...from this base address...
  _preformed_read_i.setParams(CPLD_REG_IMU_DM_P_I|0x80, 12, 17, LSM9DS1::regAddr(RegID::A_DATA_X)|0x80);
  // ...and drop the results here.
  _preformed_read_i.buf      = (uint8_t*) _frame_buf_i;
  _preformed_read_i.buf_len  = 204;

  _preformed_read_m.shouldReap(false);
  _preformed_read_m.devRegisterAdvance(true);
  _preformed_read_m.set_opcode(BusOpcode::RX);
  _preformed_read_m.callback = (BusOpCallback*) this;
  // Starting from the first magnetometer...
  // Read 6 bytes...
  // ...across 17 sensors...
  // ...from this base address...
  _preformed_read_m.setParams(CPLD_REG_IMU_DM_P_M|0x80, 6, 17, LSM9DS1::regAddr(RegID::M_DATA_X)|0xC0);
  // ...and drop the results here.
  _preformed_read_m.buf      = (uint8_t*) _reg_block_m_data;
  _preformed_read_m.buf_len  = 102;

  _preformed_fifo_read.shouldReap(false);
  _preformed_fifo_read.devRegisterAdvance(false);
  _preformed_fifo_read.set_opcode(BusOpcode::RX);
  _preformed_fifo_read.callback = (BusOpCallback*) this;
  // Starting from the first inertial...
  // Read 1 byte...
  // ...across 17 sensors...
  // ...from this base address...
  _preformed_fifo_read.setParams(CPLD_REG_IMU_DM_P_I|0x80, 1, 17, LSM9DS1::regAddr(RegID::AG_FIFO_SRC)|0x80);
  // ...and drop the results here.
  _preformed_fifo_read.buf      = (uint8_t*) __fifo_levels;
  _preformed_fifo_read.buf_len  = 17;

  _preformed_read_temp.shouldReap(false);
  _preformed_read_temp.devRegisterAdvance(true);
  _preformed_read_temp.set_opcode(BusOpcode::RX);
  _preformed_read_temp.callback = (BusOpCallback*) this;
  // Starting from the first inertial...
  // Read 2 bytes...
  // ...across 17 sensors...
  // ...from this base address...
  _preformed_read_temp.setParams(CPLD_REG_IMU_DM_P_I|0x80, 2, 17, LSM9DS1::regAddr(RegID::AG_DATA_TEMP)|0x80);
  // ...and drop the results here.
  _preformed_read_temp.buf      = (uint8_t*) __temperatures;
  _preformed_read_temp.buf_len  = 34;

  // Zero the ManuLegend.
  for (int i = 0; i < LEGEND_MGR_MAX_DATASET_SIZE; i++) {
    *(__dataset + i) = 0;
  }

  // Global frame data for MAP_STATE.
  _ptr_sequence = (uint32_t*) (__dataset + LEGEND_DATASET_OFFSET_SEQUENCE/4);
  _ptr_delta_t  = (float*)    (__dataset + LEGEND_DATASET_OFFSET_DELTA_T/4);

  *(_ptr_sequence) = 0;

  _er_set_flag(LEGEND_MGR_FLAGS_LEGEND_STABLE, true);  // Ick....
  operating_legend = new ManuLegend();
  operating_legend->sensorEnabled(true);
  //operating_legend->accNullGravity(true);
  operating_legend->accRaw(true);
  operating_legend->gyro(true);
  operating_legend->mag(true);
  operating_legend->orientation(true);
  operating_legend->temperature(true);
  if (operating_legend->finallize()) {
    local_log.concat("ManuLegend failed to finallize().\n");
  }
  reconfigure_data_map();
  setLegend(operating_legend);
}



/* This should probably never be called. */
ManuManager::~ManuManager() {
  while (active_legends.hasNext()) active_legends.remove();
}


/*
* Return a ref to the indicated IIU, instantiating and validating it, if necessary.
* Assuming the caller passes an unsigned int in the range [0, 17), this fxn has
*   absolutely no excuse for returning NULL, since we built these objects when CPLDDriver
*   was constructed.
*/
LSM9DS1* ManuManager::fetchIMU(uint8_t idx) {
  // TODO: We have no excuse for needing a modulus here. Too expensive. Never occured.
  return &imus[idx % LEGEND_DATASET_IIU_COUNT];
}


/*
* Digitabulum places the following constraints on IMU operation:
*   1) The entire sensor package must be operating at the same sample-rate.
*   2) Sensors must be configured for multiple-access.
*   3) AG FIFO enabled, and interrupt at high-water mark.
*   4) Interrupt on magnetometer data ready.
*
* Constraints 1 might be lifted in the future at the cost of software complexity.
*
*
*/
int8_t ManuManager::init_iius() {
  return 0;
}


/**
* Calling this method will cause the class to send the event that represents the
*   current state of the IMU data. This is the means by which we send IMU data to
*   a counterparty.
*
* @return non-zero on error.
*/
int8_t ManuManager::send_map_event() {
  event_legend_frame_ready.specific_target = nullptr;
  if (operating_legend && _er_flag(LEGEND_MGR_FLAGS_LEGEND_SENT)) {
    operating_legend->copy_frame();
    Kernel::staticRaiseEvent(&event_legend_frame_ready);
    return 0;
  }
  return -1;
}


/**
* Calling this fxn will cause the initial address assignment and dataset to be fed to the IIUs.
*
* @return non-zero on error.
*/
int8_t ManuManager::reconfigure_data_map() {
  uint16_t accumulated_offset = LEGEND_DATASET_GLOBAL_SIZE;
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    /* Assign the ManuLegend specification to the IIU class, thereby giving the IIU class its pointers. */
    // TODO: This is broken. Integrator should emit the data?
    integrator.assign_legend_pointers(
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_ACC      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_GYR      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_MAG      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_VEL      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_NULL_GRAV),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_POSITION ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_QUAT     ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_TEMP     ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_ACC   ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_GYR   ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_MAG   ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_TMEP  )
    );
    accumulated_offset += LEGEND_DATASET_PER_IMU_SIZE;
  }
  integrator.nullGyroError(true);   // No reason to not do this...
  integrator.nullifyGravity(false);
  return 0;
}



// Calling causes a pointer dance that reconfigures the data we send to the host.
// Don't do anything unless the legend is stable. This is concurrency-control.
int8_t ManuManager::setLegend(ManuLegend* nu_legend) {
  if (_er_flag(LEGEND_MGR_FLAGS_LEGEND_STABLE)) {
    // Only reconfigure if stable.
    _er_clear_flag(LEGEND_MGR_FLAGS_LEGEND_STABLE);   // Mark as unstable.
    _er_clear_flag(LEGEND_MGR_FLAGS_LEGEND_SENT);

    if (!nu_legend->finallized()) {
      // Finallize the ManuLegend prior to installing it.
      nu_legend->finallize();
    }

    bool should_enable_pid = false;
    if (event_legend_frame_ready.isScheduled()) {
      event_legend_frame_ready.enableSchedule(false);
      should_enable_pid = true;
    }

    // Store a pointer to our dataset in the event that is to carry them.
    // It is important that this argument NOT be reaped.
    event_legend_frame_ready.abort();      // We don't want this to proc while the dataset is in flux.
    event_legend_frame_ready.clearArgs();
    event_legend_frame_ready.addArg((void*) nu_legend->dataset_local, nu_legend->datasetSize());

    // Now we need to declare the new IMU Legend to the rest of the system. We will not re-enable
    //   the frame broadcast until the callback for this event happens. This assures that the message
    //   order to anyone listening is what we intend.
    StringBuilder* legend_string = new StringBuilder();
    nu_legend->formLegendString(legend_string);
    ManuvrMsg* legend_broadcast     = Kernel::returnEvent(DIGITABULUM_MSG_IMU_LEGEND);
    legend_broadcast->specific_target = nu_legend->owner;
    legend_broadcast->priority(EVENT_PRIORITY_LOWEST + 1);
    legend_broadcast->setOriginator((EventReceiver*) this);
    legend_broadcast->addArg(legend_string)->reapValue(true);

    if (should_enable_pid) {
      event_legend_frame_ready.enableSchedule(true);
    }
    return 0;
  }
  return -1;
}



uint32_t ManuManager::totalSamples() {
  uint32_t return_value = 0;
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    return_value += integrator.totalSamples();
  }
  return return_value;
}


void ManuManager::enableAutoscale(SampleType s_type, bool enabled) {
  switch (s_type) {
    case SampleType::ACCEL:
      for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        imus[i].autoscale_acc(enabled);
      }
      break;
    case SampleType::GYRO:
      for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        imus[i].autoscale_gyr(enabled);
      }
      break;
    case SampleType::MAG:
      for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        imus[i].autoscale_mag(enabled);
      }
      break;
    case SampleType::ALL:
      for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        imus[i].autoscale_acc(enabled);
        imus[i].autoscale_gyr(enabled);
        imus[i].autoscale_mag(enabled);
      }
      break;
    default:
      break;
  }
}


/*******************************************************************************
* Functions related to abstracting chirality away from the hardware.
*******************************************************************************/

/*
* Called internally as a result of either (a conclusive chirality test) or
*   (loaded configuration).
*/
int ManuManager::set_chirality(Chirality c) {
  uint8_t x = LEGEND_MGR_FLAGS_CHIRALITY_KNOWN;
  switch (c) {   // I won't even apologize for this. Suffer.
    default:
    case Chirality::UNKNOWN: return -1;
    case Chirality::LEFT:    x |= LEGEND_MGR_FLAGS_CHIRALITY_LEFT;
    case Chirality::RIGHT:   break;
  }
  _er_set_flag(x);
  return 0;
}


DigitPort ManuManager::get_port_given_digit(Anatomical digit) {
  // TODO: This is sloppy. Might-should rework it to use no conditionals.
  switch (digit) {
    case Anatomical::DIGIT_1:
    case Anatomical::DIGIT_2:
    case Anatomical::DIGIT_4:
    case Anatomical::DIGIT_5:
      if (_er_flag(LEGEND_MGR_FLAGS_CHIRALITY_KNOWN)) {
        // If left-handed, we mirror around the 3rd digit using XOR..
        return (DigitPort)(
          _er_flag(LEGEND_MGR_FLAGS_CHIRALITY_LEFT) ?
          ((uint8_t)digit)^0x04 : (uint8_t) digit
        );
      }
    case Anatomical::UNKNOWN:
      break;
    // The two invarient cases:
    case Anatomical::METACARPALS:  return DigitPort::MC;
    case Anatomical::DIGIT_3:      return DigitPort::PORT_3;
  }
  return DigitPort::UNKNOWN;
}


Anatomical ManuManager::get_digit_given_port(DigitPort port) {
  // TODO: This is sloppy. Might-should rework it to use no conditionals.
  switch (port) {
    case DigitPort::PORT_1:
    case DigitPort::PORT_2:
    case DigitPort::PORT_4:
    case DigitPort::PORT_5:
      if (_er_flag(LEGEND_MGR_FLAGS_CHIRALITY_KNOWN)) {
        // If left-handed, we mirror around the 3rd digit using XOR..
        return (Anatomical)(
          _er_flag(LEGEND_MGR_FLAGS_CHIRALITY_LEFT) ?
          ((uint8_t)port)^0x04 : (uint8_t) port
        );
      }
    case DigitPort::UNKNOWN:
      break;
    // The two invarient cases:
    case DigitPort::MC:      return Anatomical::METACARPALS;
    case DigitPort::PORT_3:  return Anatomical::DIGIT_3;
  }
  return Anatomical::UNKNOWN;
}



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
* Here we have the chance to ammend or cancel a bus operation prior to it
*   consuming bus time.
*
* @param  _op  The bus operation that was completed.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t ManuManager::io_op_callahead(BusOp* _op) {
  Kernel::log("ManuManager::io_op_callahead\n");
  return 0;
}


/**
* When a bus operation completes, it is passed back to its issuing class.
* Unlike r0, all IMU traffic calls back to this function, and not the individual
*   IMU drivers.
*
* @param  _op  The bus operation that was completed.
* @return 0 on success, or appropriate error code.
*/
int8_t ManuManager::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  int8_t return_value = SPI_CALLBACK_NOMINAL;
  if (op->hasFault()) {
    if (getVerbosity() > 3) {
      local_log.concat("io_op_callback() rejected a callback because the bus op failed.\n");
      Kernel::log(&local_log);
    }
    return SPI_CALLBACK_ERROR;
  }

  uint8_t cpld_addr = op->getTransferParam(0);
  uint8_t imu_count = op->getTransferParam(2);
  uint8_t reg_addr  = op->getTransferParam(3);
  RegID   idx       = RegPtrMap::regIdFromAddr(cpld_addr, reg_addr);

  // Alright... we'll do the selection based on register address first
  // These checks we can do regardless of target sensor aspect.
  switch (idx) {
    case RegID::M_WHO_AM_I:
    case RegID::AG_WHO_AM_I:  // This is a bulk identity check.
      for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        if ((0x3d == _reg_block_ident[i]) && (0x68 == _reg_block_ident[i+LEGEND_DATASET_IIU_COUNT])) {
          // If the identity bytes match, set the IMU state appropriately...
          fetchIMU(i)->setDesiredState(IMUState::STAGE_1);
        }
      }
      //printIMURollCall(&local_log);
      break;

    case RegID::A_DATA_X:  // Data must be copied out of the register buffer,
    case RegID::A_DATA_Y:  //   adjusted while it's still cheap (integer), and
    case RegID::A_DATA_Z:  //   scaled into floats.
    case RegID::G_DATA_X:  // Then, the data is sent to the integrator.
    case RegID::G_DATA_Y:  //
    case RegID::G_DATA_Z:  //
      {
        // First, note the pointer relation.
        // Scale the data
        SensorFrame* nu_msrmnt = Integrator::fetchMeasurement();
        for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        }
        // Send to integrator.
      }
      break;


    case RegID::M_CTRL_REG1:
      break;
    case RegID::M_CTRL_REG3:
      break;
    case RegID::M_CTRL_REG4:
      break;
    case RegID::M_CTRL_REG5:
      break;
    case RegID::M_INT_CFG:
      break;
    case RegID::AG_INT1_CTRL:
      break;
    case RegID::AG_INT2_CTRL:
      break;
    case RegID::G_ORIENT_CFG:
      break;
    case RegID::AG_CTRL_REG4:
      break;
    case RegID::A_CTRL_REG5:
      break;
    case RegID::AG_CTRL_REG8:
      break;
    case RegID::AG_CTRL_REG9:
      break;
    case RegID::AG_CTRL_REG10:
      break;
    case RegID::AG_FIFO_CTRL:
      break;


    /* These are exported to the IMU class. */
    case RegID::M_OFFSET_X:
      break;
    case RegID::M_OFFSET_Y:
      break;
    case RegID::M_OFFSET_Z:
      break;
    case RegID::M_CTRL_REG2:
      break;
    case RegID::M_STATUS_REG:
      break;
    case RegID::M_DATA_X:
      break;
    case RegID::M_DATA_Y:
      break;
    case RegID::M_DATA_Z:
      break;
    case RegID::M_INT_SRC:
      break;
    case RegID::M_INT_TSH:
      break;
    case RegID::AG_ACT_THS:
      break;
    case RegID::AG_ACT_DUR:
      break;
    case RegID::A_INT_GEN_CFG:
      break;
    case RegID::A_INT_GEN_THS_X:
      break;
    case RegID::A_INT_GEN_THS_Y:
      break;
    case RegID::A_INT_GEN_THS_Z:
      break;
    case RegID::A_INT_GEN_DURATION:
      break;
    case RegID::G_REFERENCE:
      break;
    case RegID::G_CTRL_REG1:
      break;
    case RegID::G_CTRL_REG2:
      break;
    case RegID::G_CTRL_REG3:
      break;
    case RegID::G_INT_GEN_SRC:
      break;
    case RegID::AG_DATA_TEMP:
      break;
    case RegID::AG_STATUS_REG:
      break;
    case RegID::A_CTRL_REG6:
      break;
    case RegID::A_CTRL_REG7:
      break;
    case RegID::A_INT_GEN_SRC:
      break;
    case RegID::AG_STATUS_REG_ALT:
      break;
    case RegID::AG_FIFO_SRC:
      break;
    case RegID::G_INT_GEN_CFG:
      break;
    case RegID::G_INT_GEN_THS_X:
      break;
    case RegID::G_INT_GEN_THS_Y:
      break;
    case RegID::G_INT_GEN_THS_Z:
      break;
    case RegID::G_INT_GEN_DURATION:
      break;

    default:
      // For registers that are still handled via individual IMU classes, pass
      //   control to them.
      for (uint8_t i = (cpld_addr % LEGEND_DATASET_IIU_COUNT); i < imu_count; i++) {
        // For each sensor involved in the transfer, notify it of new data.
        LSM9DS1* imu = fetchIMU(i);
        IMUFault error_condition = (BusOpcode::RX == op->get_opcode()) ? imu->proc_register_read(idx) : imu->proc_register_write(idx);
      }
      break;
  }

  if (op == &_preformed_read_i) {
    Kernel::staticRaiseEvent(&quat_crunch_event);
    // TODO: If the FIFO watermark IRQ signal is still asserted, read another batch.
    return_value = SPI_CALLBACK_RECYCLE;
  }

  flushLocalLog();
  return return_value;
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
int8_t ManuManager::queue_io_job(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  if (nullptr == op->callback) {
    op->callback = (BusOpCallback*) this;
  }
  return _bus->queue_io_job(op);
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
int8_t ManuManager::attached() {
  if (EventReceiver::attached()) {
    /* Get ready for a silly pointer dance....
    *  This is an argument-heavy event, and we will be using it ALOT. So we build the Event arguments
    *    once, and then change the data at the location being pointed at, and not the pointers in the
    *    arguments. Technically, we could make this even faster by addingg a new type for Vector3<float>**,
    *    but this will be a serious undertaking. We should ultimately implement ** types with a flag, not
    *    a type_code.
    *    Bassnectar - 04. You &amp; Me ft. W. Darling.mp3
    *    Knife Party - 04. EDM Trend Machine.mp3
    *    Bassnectar - 04 - Boomerang.mp3
    *    Bassnectar - 01. F.U.N..mp3
    *    ---J. Ian Lindsay   Thu Apr 09 04:04:41 MST 2015
    */
    event_iiu_read.repurpose(DIGITABULUM_MSG_IMU_READ, (EventReceiver*) this);
    event_iiu_read.incRefs();
    event_iiu_read.specific_target = (EventReceiver*) this;
    event_iiu_read.priority(EVENT_PRIORITY_LOWEST);
    event_iiu_read.alterSchedulePeriod(20);
    event_iiu_read.alterScheduleRecurrence(-1);
    event_iiu_read.autoClear(false);
    event_iiu_read.enableSchedule(false);

    // Build some pre-formed Events.
    event_legend_frame_ready.repurpose(DIGITABULUM_MSG_IMU_MAP_STATE, (EventReceiver*) this);
    event_legend_frame_ready.incRefs();
    event_legend_frame_ready.specific_target = nullptr; //(EventReceiver*) this;
    event_legend_frame_ready.priority(EVENT_PRIORITY_LOWEST);
    event_legend_frame_ready.alterSchedulePeriod(25);
    event_legend_frame_ready.alterScheduleRecurrence(-1);
    event_legend_frame_ready.autoClear(false);
    event_legend_frame_ready.enableSchedule(false);

    /* Setup our pre-formed quat crunch event. */
    quat_crunch_event.repurpose(DIGITABULUM_MSG_IMU_QUAT_CRUNCH, this);
    quat_crunch_event.incRefs();
    quat_crunch_event.specific_target = (EventReceiver*) this;
    //quat_crunch_event.priority(4);

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
int8_t ManuManager::callback_proc(ManuvrMsg* event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = (0 == event->refCount()) ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->eventCode()) {
    case DIGITABULUM_MSG_IMU_READ:
      if (false) {  // TODO: Check the minimum FIFO level.
        return EVENT_CALLBACK_RETURN_RECYCLE;
      }
      break;

    case DIGITABULUM_MSG_IMU_INIT:
      // At this point we should have all IMUs initialized. We probably had interrupts
      //   happen that are hanging. Clear them, as they will be meaningless.
      break;

    case DIGITABULUM_MSG_IMU_LEGEND:
      // We take this as an indication that our notice of altered Legend was sent.
      _er_set_flag(LEGEND_MGR_FLAGS_LEGEND_SENT);
      break;

    case DIGITABULUM_MSG_IMU_MAP_STATE:
      *(_ptr_sequence) = *(_ptr_sequence) + 1;
      if (operating_legend && _er_flag(LEGEND_MGR_FLAGS_LEGEND_SENT)) {
        operating_legend->copy_frame();
        Kernel::staticRaiseEvent(&event_legend_frame_ready);
        return 0;
      }
      break;

    case DIGITABULUM_MSG_IMU_QUAT_CRUNCH:
      if (integrator.has_quats_left()) {
        return_value = EVENT_CALLBACK_RETURN_RECYCLE;
      }
      break;

    default:
      if (getVerbosity() > 5) {
        local_log.concat("ManuManager::callback_proc(): Default case.\n");
        #if defined(__MANUVR_DEBUG)
          event->printDebug(&local_log);
        #endif
        Kernel::log(&local_log);
      }
      break;
  }

  flushLocalLog();
  return return_value;
}




int8_t ManuManager::notify(ManuvrMsg* active_event) {
  int8_t return_value = 0;
  uint8_t temp_uint_8 = 0;

  /* Some class-specific set of conditionals below this line. */
  switch (active_event->eventCode()) {
    case DIGITABULUM_MSG_IMU_READ:
      return_value++;
      break;

    case MANUVR_MSG_SESS_ESTABLISHED:
      event_legend_frame_ready.delaySchedule(1100);     // Enable the periodic frame broadcast.
      {
        ManuvrMsg *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT);
        event->addArg((uint8_t) 4);  // Set the desired init stage.
        event->priority(0);
        raiseEvent(event);
      }
      event_iiu_read.delaySchedule(1000);  // Enable the periodic read after letting the dust settle.
      return_value++;
      break;

    case MANUVR_MSG_SESS_HANGUP:
      event_legend_frame_ready.enableSchedule(false);
      for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        ManuvrMsg *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT);
        event->addArg((uint8_t) 4);  // Set the desired init stage.
        event->priority(0);
        raiseEvent(event);
      }
      event_iiu_read.enableSchedule(false);    // Disable the periodic read.
      return_value++;
      break;

    case DIGITABULUM_MSG_IMU_INIT:
      for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        imus[i].init();
      }
      return_value++;
      break;

    case DIGITABULUM_MSG_IMU_MAP_STATE:
      //if (0 == active_event->argCount()) {
        // No args means a request. Send it.
      //  send_map_event();
      //  return_value++;
      //}
      break;


    case DIGITABULUM_MSG_CPLD_RESET_COMPLETE:
      if (getVerbosity() > 3) local_log.concatf("Initializing IMUs...\n");
      // Range-bind everything....
      integrator.rangeBind(true);

      // Fire the event to put the IMUs into INIT-1.
      //raiseEvent(Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT));
      return_value++;
      break;

    case DIGITABULUM_MSG_IMU_TAP:
      if (0 == active_event->argCount()) {
        // Somthing wants the thresholds for all configured taps.
      }
      else {
        // Otherwise, it means we've emitted the event. No need to respond.
      }
      break;

    case DIGITABULUM_MSG_IMU_QUAT_CRUNCH:
      if (0 == active_event->getArgAs(&temp_uint_8)) {
        if (temp_uint_8 > 16) {
          if (getVerbosity() > 1) local_log.concat("QUAT_CRUNCH had an IMU idx > 16.\n");
        }
        else {
          integrator.MadgwickQuaternionUpdate();
        }
        return_value++;
      }
      else {
        if (getVerbosity() > 2) local_log.concatf("QUAT_CRUNCH handler (IIU %u) got a bad return from an Arg..\n", temp_uint_8);
      }
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
* // TODO: This is a lie. Audit __MANUVR_DEBUG usage and elaborate it by class to reduce build sizes.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void ManuManager::printIMURollCall(StringBuilder *output) {
  EventReceiver::printDebug(output);
  output->concat("-- Intertial integration units: id(M/I)\n--\n-- Dgt      Prx        Imt        Dst        Reports\n");
  // TODO: Audit usage of length-specified integers as iterators. Cut where not
  //   important and check effects on optimization, as some arch's take a
  //   runtime hit for access in any length less than thier ALU widths.
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    switch (i) {
      case 1:   // Skip output for the IMU that doesn't exist at digit0.
        output->concat("   <N/A>   ");
        break;
      case 0:
        output->concat("-- 0(MC)    ");
        break;
      case 2:   // digit1 begins
        output->concatf("%c\n-- 1        ", _bus->digitExists(DigitPort::MC) ? 'Y' : ' ');
        break;
      case 5:   // digit2 begins
        output->concatf("%c\n-- 2        ", _bus->digitExists(DigitPort::PORT_1) ? 'Y' : ' ');
        break;
      case 8:   // digit3 begins
        output->concatf("%c\n-- 3        ", _bus->digitExists(DigitPort::PORT_2) ? 'Y' : ' ');
        break;
      case 11:  // digit4 begins
        output->concatf("%c\n-- 4        ", _bus->digitExists(DigitPort::PORT_3) ? 'Y' : ' ');
        break;
      case 14:  // digit5 begins
        output->concatf("%c\n-- 5        ", _bus->digitExists(DigitPort::PORT_4) ? 'Y' : ' ');
        break;
      default:
        break;
    }
    output->concatf("%02u(%02x/%02x)  ", i, _reg_block_ident[i], _reg_block_ident[i+LEGEND_DATASET_IIU_COUNT]);
  }
  output->concatf("%c\n\n", _bus->digitExists(DigitPort::PORT_5) ? 'Y' : ' ');
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void ManuManager::printTemperatures(StringBuilder *output) {
  EventReceiver::printDebug(output);
  output->concat("-- Intertial integration units: id(deg-C)\n--\n-- Dgt      Prx        Imt        Dst\n");
  // TODO: Audit usage of length-specified integers as iterators. Cut where not
  //   important and check effects on optimization, as some arch's take a
  //   runtime hit for access in any length less than thier ALU widths.
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    switch (i) {
      case 1:   // Skip output for the IMU that doesn't exist at digit0.
        output->concat("   <N/A>   ");
        break;
      case 0:
        output->concat("-- 0(MC)    ");
        break;
      case 2:   // digit1 begins
        output->concat("\n-- 1        ");
        break;
      case 5:   // digit2 begins
        output->concat("\n-- 2        ");
        break;
      case 8:   // digit3 begins
        output->concat("\n-- 3        ");
        break;
      case 11:  // digit4 begins
        output->concat("\n-- 4        ");
        break;
      case 14:  // digit5 begins
        output->concat("\n-- 5        ");
        break;
      default:
        break;
    }
    output->concatf("%02u(%03d)  ", i, *((int16_t*) &__temperatures[i << 2]));
  }
  output->concat("\n\n");
}


#if defined(__MANUVR_DEBUG)
void ManuManager::dumpPreformedElements(StringBuilder* output) {
  output->concat("--- Quat-crunch event\n");
  quat_crunch_event.printDebug(output);
  output->concat("\n");
}
#endif



/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void ManuManager::printDebug(StringBuilder *output) {
  EventReceiver::printDebug(output);
  if (getVerbosity() > 0) {
    // Print just the aggregate sample count and return.
  }
  output->concatf("-- Chirality           %s\n", chiralityString(getChirality()));

  if (getVerbosity() > 3) {
    output->concatf("-- __dataset location  %p\n", (uintptr_t) __dataset);
    output->concatf("-- __IIU location      %p\n", (uintptr_t) imus);
    output->concatf("-- INSTANCE location   %p\n--\n", (uintptr_t) INSTANCE);
  }

  float grav_consensus = 0.0;
  output->concat("-- Intertial integration units:\n");
  for (uint8_t i = 0; i < 17; i++) {
    //grav_consensus += imus[i].grav_scalar;
  }
  grav_consensus /= 17;
  output->concatf("-- Gravity consensus:  %.4fg\n",  (double) grav_consensus);

  output->concatf("-- Sequence number     %u\n",    (unsigned long) *(_ptr_sequence));
  output->concatf("-- Max quat proc       %u\n",    Integrator::max_quats_per_event);
  output->concatf("-- Sequence number     %u\n",    (unsigned long) *(_ptr_sequence));
  output->concatf("-- Delta-t             %2.5f\n--\n", (double) *(_ptr_delta_t));

  if (getVerbosity() > 3) {
    output->concatf("-- MAX_DATASET_SIZE    %u\n",    (unsigned long) LEGEND_MGR_MAX_DATASET_SIZE);
    #if defined(__MANUVR_DEBUG)
      if (getVerbosity() > 5) {
        event_legend_frame_ready.printDebug(output);
        event_iiu_read.printDebug(output);
      }
    #endif
  }

  #if defined(__MANUVR_DEBUG)
    dumpPreformedElements(output);
  #endif

  output->concat("-- Intertial integration units:\n");
  for (uint8_t i = 0; i < 17; i++) {
    output->concatf("\tIIU %d\t ", i);
    imus[i].dumpDevRegs(output);
  }
  output->concat("\n");
}



#if defined(MANUVR_CONSOLE_SUPPORT)
void ManuManager::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = 0;
  if (*(str) != 0) {
    temp_byte = atoi((char*) str+1);
  }

  StringBuilder parse_mule;

  switch (*(str)) {
    case 'v':
      parse_mule.concat(str);
      local_log.concatf("parse_mule split (%s) into %d positions.\n", str, parse_mule.split(","));
      parse_mule.drop_position(0);
      if (temp_byte < 17) {
        if (parse_mule.count() > 0) {
          int temp_int = parse_mule.position_as_int(0);
          imus[temp_byte].setVerbosity(temp_int);
        }
        local_log.concatf("Verbosity on IMU %d is %d.\n", temp_byte, imus[temp_byte].getVerbosity());
      }
      break;

    case 'i':
      switch (temp_byte) {
        case 1:
          dumpPreformedElements(&local_log);
          break;

        case 2:
          if (operating_legend) {
            operating_legend->printDebug(&local_log);
          }
          else {
            local_log.concat("No operating ManuLegend.\n");
          }
          break;
        case 3:
          read_identities();  // Read the sensor's identity registers.
          break;
        case 4:
          printIMURollCall(&local_log);   // Show us the results, JIC
          break;
        case 5:
          integrator.dumpPointers(&local_log);   // Show us the results, JIC
          break;
        case 9:
          printTemperatures(&local_log);   // Show us the temperatures.
          break;
        default:
          {
            int8_t old_verbosity = getVerbosity();
            if (temp_byte && (temp_byte < 7)) setVerbosity(temp_byte);
            printDebug(&local_log);
            if (temp_byte && (temp_byte < 7)) setVerbosity(old_verbosity);
          }
          break;
      }
      break;

    case '-':
      if (operating_legend) {
        operating_legend->copy_frame();
        local_log.concat("Frame copied.\n");
        operating_legend->printDataset(&local_log);
      }
      break;

    // IMU DEBUG //////////////////////////////////////////////////////////////////
    case 'c':
      if (temp_byte < 17) {
        imus[temp_byte].dumpDevRegs(&local_log);
      }
      break;

    // IMU STATE CONTROL //////////////////////////////////////////////////////////
    case 's':
      parse_mule.concat(str);
      parse_mule.split(",");
      parse_mule.drop_position(0);
      if (parse_mule.count() > 0) {
        int temp_int = parse_mule.position_as_int(0);

        if (255 == temp_byte) {
          for (uint8_t i = 0; i < 17; i++) {
            imus[i].setDesiredState((IMUState) temp_int);
          }
        }
        else if (temp_byte < 17) {
          local_log.concatf("Setting the state of IMU %d to %d\n", temp_byte, temp_int);
          imus[temp_byte].setDesiredState((IMUState) temp_int);
        }

      }
      break;

    case 'k':
      if ((temp_byte < 6) && (temp_byte >= 0)) {
        ManuvrMsg *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT);
        event->addArg((uint8_t) temp_byte);  // Set the desired init stage.
        event->priority(0);
        raiseEvent(event);
        local_log.concatf("Broadcasting IMU_INIT for stage %u...\n", temp_byte);
      }
      else {
        local_log.concatf("Illegal INIT stage: %u\n", temp_byte);
      }
      break;

    case 'r':
      if (255 == temp_byte) {
        local_log.concat("Reseting all IIUs...\n");
        for (uint8_t i = 0; i < 17; i++) {
          imus[i].reset();
        }
      }
      else if (temp_byte < 17) {
        local_log.concatf("Resetting IIU %d.\n", temp_byte);
        imus[temp_byte].reset();
      }
      break;


    case 'T':
    case 't':
      if (temp_byte < 17) {
        ManuvrMsg *event = Kernel::returnEvent((*(str) == 'T') ? DIGITABULUM_MSG_IMU_DOUBLE_TAP : DIGITABULUM_MSG_IMU_TAP);
        event->setOriginator((EventReceiver*) this);
        event->addArg((uint8_t) temp_byte);
        Kernel::staticRaiseEvent(event);
        local_log.concatf("Sent %stap event for IMU %d.\n", ((*(str) == 'T') ? "double ":""), temp_byte);
      }
      break;

    case 'q':
      if (temp_byte < 17) {
        ManuvrMsg *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_QUAT_CRUNCH);
        event->specific_target = (EventReceiver*) this;
        event->addArg((uint8_t) temp_byte);
        Kernel::staticRaiseEvent(event);
        local_log.concatf("Running quat on IIU %d.\n", temp_byte);
      }
      break;



    // IMU DATA ///////////////////////////////////////////////////////////////////
    case 'j':
      switch (*(str+1)) {
        case '0':
          reflection_acc.x = -1;
          reflection_acc.y = 1;
          reflection_acc.z = -1;
          reflection_gyr.set(-1, 1, -1);
          reflection_mag.x = 1;
          reflection_mag.y = 1;
          reflection_mag.z = -1;
          break;
        case 'm':
          if (*(str+2) == 'x')      reflection_mag.x *= -1;
          else if (*(str+2) == 'y') reflection_mag.y *= -1;
          else if (*(str+2) == 'z') reflection_mag.z *= -1;
          break;

        case 'a':
          if (*(str+2) == 'x')      reflection_acc.x *= -1;
          else if (*(str+2) == 'y') reflection_acc.y *= -1;
          else if (*(str+2) == 'z') reflection_acc.z *= -1;
          break;

        case 'g':
          if (*(str+2) == 'x')      reflection_gyr.x *= -1;
          else if (*(str+2) == 'y') reflection_gyr.y *= -1;
          else if (*(str+2) == 'z') reflection_gyr.z *= -1;
          break;
      }
      local_log.concatf("Reflection vectors\n\tMag (%d, %d, %d)\n\tAcc (%d, %d, %d)\n\tGyr (%d, %d, %d)\n", reflection_mag.x, reflection_mag.y, reflection_mag.z, reflection_acc.x, reflection_acc.y, reflection_acc.z, reflection_gyr.x, reflection_gyr.y, reflection_gyr.z);
      break;

    case '[':
    case ']':
      local_log.concatf("%sabling spherical abberation correction on all IIUs.\n", ((*(str) == ']') ? "En":"Dis"));
      integrator.correctSphericalAbberation((*(str) == ']'));
      break;

    case 'u':
    case 'U':
      local_log.concatf("%sabling (clean-mag-is-zero) on all IIUs.\n", ((*(str) == 'U') ? "En":"Dis"));
      integrator.cleanMagZero((*(str) == 'U'));
      break;

    case 'w':
    case 'W':
      local_log.concatf("%sabling mag data scrutiny on all IIUs.\n", ((*(str) == 'Z') ? "En":"Dis"));
      integrator.dropObviousBadMag((*(str) == 'Z'));
      break;

    case 'z':
    case 'Z':
      local_log.concatf("%sabling autoscale on all IMUs.\n", ((*(str) == 'Z') ? "En":"Dis"));
      enableAutoscale(SampleType::ALL, (*(str) == 'Z'));
      break;

    case 'n':
    case 'N':
      local_log.concatf("%sabling range-binding on all IIUs.\n", ((*(str) == 'N') ? "En":"Dis"));
      integrator.rangeBind((*(str) == 'N'));
      break;

    case 'h':
    case 'H':
      local_log.concatf("%sabling quats on all IIUs.\n", ((*(str) == 'H') ? "En":"Dis"));
      integrator.processQuats((*(str) == 'H'));
      break;

    case 'x':
    case 'X':
      local_log.concatf("%sabling gyro error compensation on all IIUs.\n", ((*(str) == 'X') ? "En":"Dis"));
      integrator.nullGyroError((*(str) == 'X'));
      break;

    case 'm':
    case 'M':
      local_log.concatf("%sabling gravity nullification on all IIUs.\n", ((*(str) == 'M') ? "En":"Dis"));
      integrator.nullifyGravity((*(str) == 'M'));
      break;

    case 'y':
    case 'Y':
      local_log.concatf("%sabling bearing nullification on all IIUs.\n", ((*(str) == 'Y') ? "En":"Dis"));
      integrator.nullifyBearing((*(str) == 'Y'));
      break;

    case 'Q':
      local_log.concatf("Madgwick iterations to %d on all IIUs.\n", temp_byte);
      integrator.madgwickIterations(temp_byte);
      break;



    case ',':
      Integrator::max_quats_per_event = temp_byte;
      local_log.concatf("IIU class now runs a maximum of %u quats per event.\n", Integrator::max_quats_per_event);
      break;

    case 'b':
      integrator.beta = (float)temp_byte * 0.1;
      local_log.concatf("Beta value is now %f.\n", (double) integrator.beta);
      break;

    case 'L':
      for (uint8_t i = 0; i < 17; i++) {
        imus[i].setSampleRateProfile(temp_byte);
      }
      local_log.concatf("Moving to sample rate profile %d.\n", temp_byte);
      break;

    case 'o':
      for (uint8_t i = 0; i < 17; i++) {
        imus[i].set_base_filter_param_gyr(temp_byte);
      }
      local_log.concatf("Setting GYR base filter to %d.\n", temp_byte);
      break;

    case 'O':
      for (uint8_t i = 0; i < 17; i++) {
        imus[i].set_base_filter_param_acc(temp_byte);
      }
      local_log.concatf("Setting ACC base filter to %d.\n", temp_byte);
      break;

    case 'd':
      switch (temp_byte) {
        case 255:
          event_legend_frame_ready.fireNow();  // Fire a single frame transmission.
          local_log.concat("We are manually firing the IMU frame broadcasts schedule.\n");
          break;
        case 254:
          event_legend_frame_ready.enableSchedule(true);  // Enable the periodic read.
          local_log.concat("Enabled frame broadcasts.\n");
          break;
        #if defined(__MANUVR_DEBUG)
          case 253:
            event_legend_frame_ready.printDebug(&local_log);
            break;
        #endif
        case 252:
          send_map_event();
          local_log.concat("We are manually firing the IMU frame broadcasts schedule.\n");
          break;
        default:
          if (temp_byte) {
            event_legend_frame_ready.alterSchedulePeriod(temp_byte*10);
            local_log.concatf("Set periodic frame broadcast to once every %dms.\n", temp_byte*10);
          }
          else {
            event_legend_frame_ready.enableSchedule(false);  // Disable the periodic read.
            local_log.concat("Disabled frame broadcasts.\n");
          }
          break;
      }
      break;

    case 'f':
      switch (temp_byte) {
        case 255:
          event_iiu_read.fireNow();
          local_log.concat("We are manually firing the IMU read schedule.\n");
          break;
        case 254:
          event_iiu_read.enableSchedule(true);
          local_log.concat("Enabled periodic readback.\n");
          break;
        default:
          if (temp_byte) {
            event_iiu_read.alterSchedulePeriod(temp_byte*10);
            local_log.concatf("Set periodic read schedule to once every %dms.\n", temp_byte*10);
          }
          else {
            event_iiu_read.enableSchedule(false);  // Disable the periodic read.
            local_log.concat("Disabled periodic readback.\n");
          }
          break;
      }
      break;

    /* Single frame readback tests. */
    case '!':   // Temperature
      queue_io_job(&_preformed_read_temp);
      break;
    case '@':   // Mag
      queue_io_job(&_preformed_read_m);
      break;
    case '#':   // Intertial
      queue_io_job(&_preformed_read_i);
      break;
    case '%':   // FIFO
      queue_io_job(&_preformed_fifo_read);
      break;

    default:
      EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  flushLocalLog();
}
#endif  //MANUVR_CONSOLE_SUPPORT





/*******************************************************************************
* ManuManager is doing too much.
*******************************************************************************/

int8_t ManuManager::read_identities() {
  // Zero the space so we ensure no false positives.
  bzero(&_reg_block_ident[0], (2 * LEGEND_DATASET_IIU_COUNT));

  // Because the identity address is the same for both aspects, and their addresses
  //   are continuous, we just read 1 byte from 34 sensors.
  SPIBusOp* op = _bus->new_op(BusOpcode::RX, this);
  op->setParams((CPLD_REG_IMU_DM_P_M | 0x80), 0x01, (2 * LEGEND_DATASET_IIU_COUNT), 0x8F);
  op->setBuffer(&_reg_block_ident[0], (2 * LEGEND_DATASET_IIU_COUNT));
  return queue_io_job(op);
}



int8_t ManuManager::read_fifo_depth() {
  SPIBusOp* op = _bus->new_op(BusOpcode::RX, this);
  op->setParams((CPLD_REG_IMU_DM_P_I | 0x80), 0x01, LEGEND_DATASET_IIU_COUNT, 0x8F);
  op->setBuffer(&_reg_block_ident[0], LEGEND_DATASET_IIU_COUNT);
  return queue_io_job(op);
}
