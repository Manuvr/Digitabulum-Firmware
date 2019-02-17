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

Vector3<int16_t> reflection_mag;
Vector3<int16_t> reflection_acc;
Vector3<int16_t> reflection_gyr;

SensorFrame _frame_pool_mem[PREALLOCD_IMU_FRAMES];


/*------------------------------------------------------------------------------
  Register memory
  These are giant strips of DMA-capable memory that are used for raw frame
    reads from the sensor package. */

/****** First, the special-cases.... ******************************************/

/* Inertial data,
    x2 because Acc+Gyr
    x2 because double-buffered
    x3 because 3-space vectors (of 16-bit ints)
    X17 because that many sensors
    = 204 int16's
    = 408 bytes
*/
int16_t __attribute__ ((aligned (4))) _frame_buf_i[2 * 2 * 3 * LEGEND_DATASET_IIU_COUNT];

/* Temperature data. Single buffered. */
// TODO: Might consolidate temp into inertial. Sensor and CPLD allow for it.
int16_t __attribute__ ((aligned (4))) __temperatures[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer data registers. Single buffered. */
int16_t __attribute__ ((aligned (4))) _reg_block_m_data[3 * LEGEND_DATASET_IIU_COUNT];

/* Identity registers for both sensor aspects. */
uint8_t __attribute__ ((aligned (4))) _reg_block_ident[2 * LEGEND_DATASET_IIU_COUNT];

/****** Ranked-access registers below this line. ******************************/
/*
  The following sensor registers are managed entirely within ManuManager. For
    those registers that we treat as write-only, and homogenous, we will use the
    ranked-access mode in the CPLD.
  NOTE: There are three ranked access addresses (prox, int, distl).
*/

/*
  AG_INT1_CTRL and AG_INT2_CTRL (Rank-access)
  3 ranks times 2 byte-wide registers.
  INT1 will service re-scaling, and INT2 will service the FIFO.
  TODO: Might merge this into a single register 16-bits wide?
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_interrupt_conf[6] = {0xC0, 0x08, 0xC0, 0x08, 0xC0, 0x08};

/*
  G_ORIENT_CFG(Rank-access)
  3 ranks times 1 byte-wide register.
  Great pains were taken to enforce a common orientation in hardware.
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_orient_cfg[3] = {0x00, 0x00, 0x00};

/*
  Deals with IRQ latching, axis enablement, and 4D. (Rank-access)
  3 ranks times 2 byte-wide registers.
  ctrl4: All gyro axis enabled, IRQ not latched, no 4D.
  ctrl5: No acc decimation, all axis enabled.
  TODO: Might merge this into a single register 16-bits wide?
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_ctrl4_5[6] = {0x38, 0x38, 0x38, 0x38, 0x38, 0x38};

/*
  IRQ pin control, FIFO and bus config, self-test. (Rank-access)
  3 ranks times 3 byte-wide registers.
  ctrl8:  BDU=1, active-hi push-pull IRQ pins, 4-wire SPI w/auto-inc, LE.
  ctrl9:  No temp in FIFO, i2c, or DRDY. FIFO on and unrestricted.
  ctrl10: Self-tests disabled.
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_ctrl8_9_10[9] = {
  0x44, 0x06, 0x00,
  0x44, 0x06, 0x00,
  0x44, 0x06, 0x00
};

/*
  Inertial aspect FIFO control registers. (Rank-access)
  3 ranks times 1 byte-wide register.
  FIFO in continuous mode, with a threshold of 4.
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_fifo_ctrl[3] = {0xC4, 0xC4, 0xC4};

/*
  Magnetometer interrupt config registers. (Rank-access)
  3 ranks times 1 byte-wide register.
  Non-latched, active-high IRQ enabled for all axes.
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_m_irq_cfg[3] = {0xE5, 0xE5, 0xE5};

/*
  Magnetometer ctrl1 registers. (Rank-access)
  3 ranks times 1 byte-wide register.
  ctrl1: Temp-compensated, med performance, 5hz, no self-test
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_m_ctrl1[3] = {0xAC, 0xAC, 0xAC};

/*
  Magnetometer ctrl3-5 registers. (Rank-access)
  3 ranks times 3 byte-wide registers.
  ctrl3:  No i2c, no LP, SPI I/O, continuous conversion.
  ctrl4:  Little-endian, med performance
  ctrl5:  Block-update.
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_m_ctrl3_5[9] = {
  0x84, 0x04, 0x40,
  0x84, 0x04, 0x40,
  0x84, 0x04, 0x40
};

/*
  Accelerometer IRQ status registers. (Discrete-access)
  Handled by: ManuManager
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_a_irq_src[LEGEND_DATASET_IIU_COUNT];

/*
  Gyroscope interrupt config, source, and threshold registers. (Discrete-access)
  Handled by: ManuManager
*/
uint8_t __attribute__ ((aligned (4))) _reg_block_g_irq_src[LEGEND_DATASET_IIU_COUNT];
uint8_t __attribute__ ((aligned (4))) _reg_block_g_irq_cfg[LEGEND_DATASET_IIU_COUNT];



/****** Individually-packed registers below this line. ************************/

/* Activity thresholds and durations. */
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_activity[2 * LEGEND_DATASET_IIU_COUNT];

/* Accelerometer interrupt settings, thresholds, G_REFERENCE */
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_0[6 * LEGEND_DATASET_IIU_COUNT];

/* Inertial aspect control registers. */
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_ctrl1_3[3 * LEGEND_DATASET_IIU_COUNT];
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_ctrl6_7[2 * LEGEND_DATASET_IIU_COUNT];


/* Inertial aspect status registers. */
uint8_t __attribute__ ((aligned (4))) _reg_block_ag_status[LEGEND_DATASET_IIU_COUNT];

/* Inertial aspect FIFO status registers. */
uint8_t __attribute__ ((aligned (4))) __fifo_levels[LEGEND_DATASET_IIU_COUNT];


int16_t __attribute__ ((aligned (4))) _reg_block_g_thresholds[3 * LEGEND_DATASET_IIU_COUNT];
uint8_t __attribute__ ((aligned (4))) _reg_block_g_irq_dur[LEGEND_DATASET_IIU_COUNT];
uint8_t __attribute__ ((aligned (4))) _reg_fifo_src[LEGEND_DATASET_IIU_COUNT];


/* Magnetometer offset registers. */
int16_t __attribute__ ((aligned (4))) _reg_block_m_offsets[3 * LEGEND_DATASET_IIU_COUNT];

/* Magnetometer control registers. */
uint8_t __attribute__ ((aligned (4))) _reg_block_m_ctrl2[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer status registers. */
uint8_t  __attribute__ ((aligned (4))) _reg_block_m_status[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer interrupt source registers. */
uint8_t __attribute__ ((aligned (4))) _reg_block_m_irq_src[LEGEND_DATASET_IIU_COUNT];

/* Magnetometer threshold registers. Will be interpreted as 15-bit unsigned. */
uint16_t __attribute__ ((aligned (4))) _reg_block_m_thresholds[LEGEND_DATASET_IIU_COUNT];

/* End of register memory
------------------------------------------------------------------------------*/


/* These are the beginnings of a ring-buffer for whole inertial frames. */
const int16_t* _frames_i[] = {
  &_frame_buf_i[0],
  &_frame_buf_i[102]
};

/* This is used to define the noise floors for the data. */
Vector3<int16_t> noise_floor_mag[LEGEND_DATASET_IIU_COUNT];
Vector3<int16_t> noise_floor_acc[LEGEND_DATASET_IIU_COUNT];
Vector3<int16_t> noise_floor_gyr[LEGEND_DATASET_IIU_COUNT];


// TODO: ThereMustBeABetterWay.jpg
const RegPtrMap _reg_ptrs[] = {
  RegPtrMap(0 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(1 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(2 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(3 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(4 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(5 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(6 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(7 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(8 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(9 , _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(10, _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(11, _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(12, _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(13, _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(14, _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(15, _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds),
  RegPtrMap(16, _reg_block_ag_activity, _reg_block_ident, _reg_block_ag_0, _reg_block_ag_ctrl1_3, _reg_block_ag_ctrl6_7, _reg_block_ag_status, _reg_fifo_src, _reg_block_g_thresholds, _reg_block_g_irq_dur, _reg_block_m_offsets, _reg_block_m_ctrl2, _reg_block_m_status, _reg_block_m_irq_src, _reg_block_m_thresholds)
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

const char* ManuManager::getManuStateString(ManuState x) {
  switch (x) {
    case ManuState::UNKNOWN:         return "UNKNOWN";
    case ManuState::PREINIT:         return "PREINIT";
    case ManuState::IMU_INIT:        return "IMU_INIT";
    case ManuState::CHIRALITY_TEST:  return "CHIRALITY_TEST";
    case ManuState::READY_IDLE:      return "READY_IDLE";
    case ManuState::READY_READING:   return "READY_READING";
    case ManuState::READY_PAUSED:    return "READY_PAUSED";
    case ManuState::READY_FLUSHING:  return "READY_FLUSHING";
    case ManuState::ASLEEP:          return "ASLEEP";
    case ManuState::FAULT:           return "FAULT";
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
ManuManager::ManuManager(BusAdapter<SPIBusOp>* bus) : EventReceiver("ManuMgmt"), _frame_pool(PREALLOCD_IMU_FRAMES, &_frame_pool_mem[0]) {
  _bus = (CPLDDriver*) bus;  // TODO: Make this cast unnecessary.
  _bus->setManuManager(this);  // Introduce ourselves immediately.

  reflection_gyr(1, 1, 1);
  reflection_acc(1, 1, 1);
  reflection_mag(1, 1, 1);

  /* Populate all the static preallocation slots for measurements. */
  for (uint16_t i = 0; i < PREALLOCD_IMU_FRAMES; i++) {
    _frame_pool_mem[i].wipe();
  }

  _preformed_read_i.shouldReap(false);
  _preformed_read_i.devRegisterAdvance(true);
  _preformed_read_i.set_opcode(BusOpcode::RX);
  _preformed_read_i.callback = (BusOpCallback*) this;
  // Starting from the first accelerometer...
  // Read 12 bytes...  (A and G vectors)
  // ...across 17 sensors...
  // ...from this base address...
  _preformed_read_i.setParams(CPLD_REG_IMU_DM_P_I|0x80, 12, 17, RegPtrMap::regAddr(RegID::A_DATA_X)|0x80);
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
  _preformed_read_m.setParams(CPLD_REG_IMU_DM_P_M|0x80, 6, 17, RegPtrMap::regAddr(RegID::M_DATA_X)|0xC0);
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
  _preformed_fifo_read.setParams(CPLD_REG_IMU_DM_P_I|0x80, 1, 17, RegPtrMap::regAddr(RegID::AG_FIFO_SRC)|0x80);
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
  _preformed_read_temp.setParams(CPLD_REG_IMU_DM_P_I|0x80, 2, 17, RegPtrMap::regAddr(RegID::AG_DATA_TEMP)|0x80);
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

  _def_pipe.active(true);
  _root_leg.sequence(true);
  _root_leg.deltaT(true);
  //_root_leg.accNullGravity(true);
  _root_leg.accRaw(true);
  _root_leg.gyro(true);
  _root_leg.mag(true);
  _root_leg.orientation(true);
  _root_leg.temperature(true);
}



/* This should probably never be called. */
ManuManager::~ManuManager() {
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


// Calling causes a pointer dance that reconfigures the data we send to the host.
// Don't do anything unless the legend is stable. This is concurrency-control.
int8_t ManuManager::setLegend(ManuLegend* nu_legend) {
  if (nullptr == nu_legend) {
    return -1;
  }

  // Now we need to declare the new IMU Legend to the rest of the system. We will not re-enable
  //   the frame broadcast until the callback for this event happens. This assures that the message
  //   order to anyone listening is what we intend.
  if (_root_leg.stackLegend(nu_legend)) {
    // TODO: If our root legend changed because of this addition, do we need to take action?
  }
  return 0;
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


int8_t ManuManager::deliverIRQ(DigitPort port, uint8_t imu_idx, uint8_t svc, uint8_t data) {
  if (getVerbosity() > 4) {
    char* pos_str = "UNKNOWN";
    switch (imu_idx) {
      case 0:
      case 2:
      case 5:
      case 8:
      case 11:
      case 14:
        pos_str = "PROX";
        break;
      case 3:
      case 6:
      case 9:
      case 12:
      case 15:
        pos_str = "INTR";
        break;
      case 1:
      case 4:
      case 7:
      case 10:
      case 13:
      case 16:
        pos_str = "DIST";
        break;
      default:
        break;
    }
    local_log.concatf(
      "deliverIRQ(%s, %u, %s): 0x%x 0x%x\n",
      CPLDDriver::getDigitPortString(port),
      imu_idx,
      pos_str,
      svc,
      data
    );
  }
  flushLocalLog();
  return 0;
}



int8_t ManuManager::_set_target_state(ManuState nu_state) {
  if (_target_state != nu_state) {
    // If there is a change to be made.
    switch (nu_state) {
      case ManuState::IMU_INIT:
      case ManuState::CHIRALITY_TEST:
      case ManuState::READY_IDLE:
      case ManuState::READY_READING:
      case ManuState::READY_PAUSED:
      case ManuState::ASLEEP:
        _target_state = nu_state;
        return _advance_state_machine();
      // States below cannot be requested.
      case ManuState::UNKNOWN:
      case ManuState::PREINIT:
      case ManuState::READY_FLUSHING:
      case ManuState::FAULT:
      default:
        return -1;
    }
  }
  return 1;
}


int8_t ManuManager::_advance_state_machine() {
  bool reloop = true;
  while (reloop && (_target_state != _current_state)) {
    reloop = false;
    local_log.concatf("_advance_state_machine(): %s --> %s\n", getManuStateString(_current_state), getManuStateString(_target_state));
    // If we are not yet in the target state...
    switch (_current_state) {
      case ManuState::UNKNOWN:
        _last_state = _current_state;
        _current_state = ManuState::PREINIT;
        // NOTE: No break;
      case ManuState::PREINIT:
        switch (_target_state) {
          case ManuState::IMU_INIT:
          case ManuState::CHIRALITY_TEST:
          case ManuState::READY_IDLE:
          case ManuState::READY_READING:
          case ManuState::READY_PAUSED:
          case ManuState::READY_FLUSHING:
          case ManuState::ASLEEP:
            if (!imuIdentitiesRead()) {
              if (_bus->hardwareReady()) {
                // If the hardware is ready, we can read the IMU identities.
                read_identities();
              }
            }
            else {
              _last_state = _current_state;
              _current_state = ManuState::IMU_INIT;
              reloop = true;
            }
            break;
          case ManuState::FAULT:
            break;
          case ManuState::UNKNOWN:
          case ManuState::PREINIT:
          default:
            break;
        }
        break;

      case ManuState::IMU_INIT:
        switch (_target_state) {
          case ManuState::CHIRALITY_TEST:
          case ManuState::READY_IDLE:
          case ManuState::READY_READING:
          case ManuState::READY_PAUSED:
          case ManuState::READY_FLUSHING:
          case ManuState::ASLEEP:
            {
              uint8_t imus_found         = 0;
              uint8_t imus_init_complete = 0;
              for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
                if (imus[i].present()) {
                  imus_found++;
                  if (imus[i].initComplete()) {
                    imus_init_complete++;
                  }
                  else if (!imus[i].initPending()) {
                    imus[i].init();
                  }
                }
              }
              if ((imus_found > 0) && (imus_found == imus_init_complete)) {
                _last_state = _current_state;
                _current_state = ManuState::READY_IDLE;
                reloop = true;
              }
            }
            break;

          case ManuState::FAULT:
            break;
          case ManuState::UNKNOWN:
          case ManuState::PREINIT:
          case ManuState::IMU_INIT:
          default:
            break;
        }
        break;
      case ManuState::CHIRALITY_TEST:
        // TODO
        _last_state = _current_state;
        _current_state = ManuState::READY_IDLE;
        break;

      case ManuState::READY_IDLE:
        switch (_target_state) {
          case ManuState::IMU_INIT:
            break;
          case ManuState::CHIRALITY_TEST:
          case ManuState::READY_READING:
          case ManuState::READY_PAUSED:
          case ManuState::READY_FLUSHING:
            {
              uint8_t imus_found   = 0;
              uint8_t imus_reading = 0;
              for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
                if (imus[i].present()) {
                  imus_found++;
                  if (imus[i].reading()) {
                    imus_reading++;
                  }
                  else if (imus[i].desired_state_attained()) {
                    imus[i].setDesiredState(IMUState::STAGE_5);
                  }
                }
              }
              if ((imus_found > 0) && (imus_found == imus_reading)) {
                _last_state = _current_state;
                _current_state = ManuState::READY_READING;
                reloop = true;
              }
            }
            break;
          case ManuState::ASLEEP:
            break;
          case ManuState::FAULT:
            break;
          case ManuState::UNKNOWN:
          case ManuState::PREINIT:
          case ManuState::READY_IDLE:
          default:
            break;
        }
        break;

      case ManuState::READY_READING:
        switch (_target_state) {
          case ManuState::IMU_INIT:
            break;
          case ManuState::CHIRALITY_TEST:
            break;
          case ManuState::READY_IDLE:
            break;
          case ManuState::READY_PAUSED:
            break;
          case ManuState::READY_FLUSHING:
            break;
          case ManuState::ASLEEP:
            break;
          case ManuState::FAULT:
            break;
          case ManuState::UNKNOWN:
          case ManuState::PREINIT:
          case ManuState::READY_READING:
          default:
            break;
        }
        break;

      case ManuState::READY_PAUSED:
        switch (_target_state) {
          case ManuState::IMU_INIT:
            break;
          case ManuState::CHIRALITY_TEST:
            break;
          case ManuState::READY_IDLE:
            break;
          case ManuState::READY_READING:
            break;
          case ManuState::READY_FLUSHING:
            break;
          case ManuState::ASLEEP:
            break;
          case ManuState::FAULT:
            break;
          case ManuState::UNKNOWN:
          case ManuState::PREINIT:
          case ManuState::READY_PAUSED:
          default:
            break;
        }
        break;
      case ManuState::READY_FLUSHING:
        switch (_target_state) {
          case ManuState::IMU_INIT:
            break;
          case ManuState::CHIRALITY_TEST:
            break;
          case ManuState::READY_IDLE:
            break;
          case ManuState::READY_READING:
            break;
          case ManuState::READY_PAUSED:
            break;
          case ManuState::ASLEEP:
            break;
          case ManuState::FAULT:
            break;
          case ManuState::UNKNOWN:
          case ManuState::PREINIT:
          case ManuState::READY_FLUSHING:
          default:
            break;
        }
        break;
      case ManuState::ASLEEP:
        switch (_target_state) {
          case ManuState::IMU_INIT:
          case ManuState::CHIRALITY_TEST:
          case ManuState::READY_IDLE:
          case ManuState::READY_READING:
          case ManuState::READY_PAUSED:
          case ManuState::READY_FLUSHING:
            break;
          case ManuState::FAULT:
            break;
          case ManuState::UNKNOWN:
          case ManuState::PREINIT:
          case ManuState::ASLEEP:
          default:
            break;
        }
        break;
      case ManuState::FAULT:
        local_log.concatf("_advance_state_machine(): Faulted while on state %s\n", getManuStateString(_current_state));
        break;
      default:
        break;
    }
  }
  flushLocalLog();
  return 0;
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
      imuIdentitiesRead(true);
      for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        if ((0x68 == _reg_block_ident[i]) && (0x3d == _reg_block_ident[i+LEGEND_DATASET_IIU_COUNT])) {
          // If the identity bytes match, set the IMU state appropriately...
          fetchIMU(i)->setDesiredState(IMUState::STAGE_1);
        }
      }
      //printIMURollCall(&local_log);
      _advance_state_machine();
      break;

    case RegID::A_DATA_X:  // Data must be copied out of the register buffer,
    case RegID::A_DATA_Y:  //   adjusted while it's still cheap (integer), and
    case RegID::A_DATA_Z:  //   scaled into floats.
    case RegID::G_DATA_X:  // Then, the data is sent to the integrator.
    case RegID::G_DATA_Y:  //
    case RegID::G_DATA_Z:  //
      {
        // First, note the pointer relation.
        float scalar_a;
        float scalar_g;
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
        int16_t* offset = (int16_t*) op->buf;

        // Scale the data
        uint32_t this_frame_time = millis();
        SensorFrame* nu_msrmnt = _frame_pool.take();
        nu_msrmnt->time((this_frame_time - _frame_time_last)/1000.0f);
        _frame_time_last = this_frame_time;
        for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
          scalar_a = imus[i].scaleA();
          scalar_g = imus[i].scaleG();
          if (imus[i].cancel_error()) {
            ax = ((((int16_t) *(offset +  0)) - noise_floor_acc[i].x) * reflection_acc.x * scalar_a);
            ay = ((((int16_t) *(offset +  2)) - noise_floor_acc[i].y) * reflection_acc.y * scalar_a);
            az = ((((int16_t) *(offset +  4)) - noise_floor_acc[i].z) * reflection_acc.z * scalar_a);
            gx = ((((int16_t) *(offset +  6)) - noise_floor_gyr[i].x) * reflection_gyr.x * scalar_g);
            gy = ((((int16_t) *(offset +  8)) - noise_floor_gyr[i].y) * reflection_gyr.y * scalar_g);
            gz = ((((int16_t) *(offset + 10)) - noise_floor_gyr[i].z) * reflection_gyr.z * scalar_g);
          }
          else {
            ax = (((int16_t) *(offset +  0)) * reflection_acc.x * scalar_a);
            ay = (((int16_t) *(offset +  2)) * reflection_acc.y * scalar_a);
            az = (((int16_t) *(offset +  4)) * reflection_acc.z * scalar_a);
            gx = (((int16_t) *(offset +  6)) * reflection_gyr.x * scalar_g);
            gy = (((int16_t) *(offset +  8)) * reflection_gyr.y * scalar_g);
            gz = (((int16_t) *(offset + 10)) * reflection_gyr.z * scalar_g);
          }
          nu_msrmnt->setI(i, ax, ay, az, gx, gy, gz);

          if (true) {  // TODO
            // If there is magnetometer data waiting, include it with the frame.
            float scalar_m = imus[i].scaleM();
            //float x = ((((int16_t)regValue(RegID::AG_DATA_X_M) - noise_floor_mag_mag.x) * reflection_vector_mag.x) * scaler);
            //float y = ((((int16_t)regValue(RegID::AG_DATA_Y_M) - noise_floor_mag_mag.y) * reflection_vector_mag.y) * scaler);
            //float z = ((((int16_t)regValue(RegID::AG_DATA_Z_M) - noise_floor_mag_mag.z) * reflection_vector_mag.z) * scaler);
            nu_msrmnt->setM(
              i,
              (_reg_block_m_data[i*3 + 0] * reflection_mag.x * scalar_m),
              (_reg_block_m_data[i*3 + 1] * reflection_mag.y * scalar_m),
              (_reg_block_m_data[i*3 + 2] * reflection_mag.z * scalar_m)
            );
          }
          offset += 12;
        }
        // Send softened and scaled frame to the integrator.
        sample_count++;
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
      // TODO: Is this data being captured elsewhere?
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
    _event_integrator.fireNow();
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
    for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
      noise_floor_mag[i].set(0, 0, 0);
      noise_floor_acc[i].set(0, 0, 0);
      noise_floor_gyr[i].set(0, 0, 0);
    }

    /* Setup our pre-formed quat crunch event. */
    _event_integrator.repurpose(DIGITABULUM_MSG_IMU_QUAT_CRUNCH, this);
    _event_integrator.incRefs();
    _event_integrator.alterScheduleRecurrence(-1);
    _event_integrator.alterSchedulePeriod(100);
    _event_integrator.autoClear(false);
    _event_integrator.enableSchedule(false);
    _event_integrator.specific_target = (EventReceiver*) this;

    /* Setup our pre-formed quat crunch event. */
    event_iiu_read.repurpose(DIGITABULUM_MSG_IMU_READ, this);
    event_iiu_read.incRefs();
    event_iiu_read.alterScheduleRecurrence(-1);
    event_iiu_read.alterSchedulePeriod(1000);
    event_iiu_read.autoClear(false);
    event_iiu_read.enableSchedule(false);
    event_iiu_read.specific_target = (EventReceiver*) this;

    platform.kernel()->addSchedule(&event_iiu_read);
    platform.kernel()->addSchedule(&_event_integrator);

    // When the hardware reports readiness, converge on this state.
    _set_target_state(ManuState::READY_IDLE);

    integrator.rangeBind(true);   // Range-bind everything....
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

    case DIGITABULUM_MSG_IMU_LEGEND:
      // We take this as an indication that our notice of altered Legend was sent.
      //legendSent(true);
      break;

    case DIGITABULUM_MSG_IMU_QUAT_CRUNCH:
      if (integrator.resultsWaiting()) {
        // TODO: This does not belong here.
        SensorFrame* frame = integrator.takeResult();
        _def_pipe.offer(frame);
        frame->wipe();
        _frame_pool.give(frame);
      }
      if (!debugFrameCycle()) {
        if (integrator.has_quats_left()) {
          return_value = EVENT_CALLBACK_RETURN_RECYCLE;
        }
      }
      break;

    default:
      if (getVerbosity() > 5) {
        local_log.concat("ManuManager::callback_proc(): Default case.\n");
        #if defined(MANUVR_DEBUG)
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

    case DIGITABULUM_MSG_CPLD_RESET_COMPLETE:
      // Instruct the state machine to put the IMUs into INIT-1.
      if (getVerbosity() > 3) local_log.concatf("Initializing IMUs...\n");
      _advance_state_machine();
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
      if (!integrator.has_quats_left()) {
        // Debug to allow cycling frames without hardware.
        uint32_t this_frame_time = millis();
        SensorFrame* nu_msrmnt = _frame_pool.take();
        nu_msrmnt->time((this_frame_time - _frame_time_last)/1000.0f);
        _frame_time_last = this_frame_time;
        integrator.pushFrame(nu_msrmnt);
      }
      integrator.churn();
      return_value++;
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
void ManuManager::printTemperatures(StringBuilder *output) {
  EventReceiver::printDebug(output);
  output->concat("-- Intertial integration units: id(deg-C)\n--\n-- Dgt      Prx      Imt      Dst\n");
  // TODO: Audit usage of length-specified integers as iterators. Cut where not
  //   important and check effects on optimization, as some arch's take a
  //   runtime hit for access in any length less than thier ALU widths.
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    switch (i) {
      case 1:   // Skip output for the IMU that doesn't exist at digit0.
        output->concat("<N/A>    ");
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


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void ManuManager::printFIFOLevels(StringBuilder *output) {
  EventReceiver::printDebug(output);
  output->concat("-- Intertial integration units: fifo_lvl(hex)\n--\n-- Dgt      Prx     Imt     Dst\n");
  // TODO: Audit usage of length-specified integers as iterators. Cut where not
  //   important and check effects on optimization, as some arch's take a
  //   runtime hit for access in any length less than thier ALU widths.
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    switch (i) {
      case 1:   // Skip output for the IMU that doesn't exist at digit0.
        output->concat("<N/A>   ");
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
    output->concatf("%02u(%02x)  ", i, __fifo_levels[i]);
  }
  output->concat("\n\n");
}


#if defined(MANUVR_DEBUG)
void ManuManager::dumpPreformedElements(StringBuilder* output) {
  output->concat("--- Predefined events:\n");
  event_iiu_read.printDebug(output);
  _event_integrator.printDebug(output);
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
  printStateMachine(output);
  output->concatf("-- Chirality           %s\n", chiralityString(getChirality()));

  if (getVerbosity() > 5) {
    output->concatf("-- __dataset location  %p\n", (uintptr_t) __dataset);
    output->concatf("-- __IIU location      %p\n--\n", (uintptr_t) imus);
  }

  float grav_consensus = 0.0;
  for (uint8_t i = 0; i < 17; i++) {
    //grav_consensus += imus[i].grav_scalar;
  }
  grav_consensus /= 17;
  output->concatf("-- Gravity consensus:  %.4fg\n", (double) grav_consensus);
  output->concatf("-- Max quat proc       %u\n",    max_quats_per_event);
  output->concatf("-- Identities read     %c\n",    imuIdentitiesRead() ? 'y':'n');
  output->concatf("-- sample_count        %d\n",    sample_count);

  if (getVerbosity() > 3) {
    output->concatf("-- MAX_DATASET_SIZE    %u\n",    (unsigned long) LEGEND_MGR_MAX_DATASET_SIZE);
  }

  //for (uint8_t i = 0; i < 17; i++) {
    //output->concatf("\tIIU %d\t ", i);
    //imus[i].dumpDevRegs(output);
    //output->concatf("--- noise_floor_mag     (%d, %d, %d)\n", noise_floor_mag[i].x, noise_floor_mag[i].y, noise_floor_mag[i].z);
    //output->concatf("--- noise_floor_acc     (%d, %d, %d)\n", noise_floor_acc[i].x, noise_floor_acc[i].y, noise_floor_acc[i].z);
    //output->concatf("--- noise_floor_gyr     (%d, %d, %d)\n", noise_floor_gyr[i].x, noise_floor_gyr[i].y, noise_floor_gyr[i].z);
  //}
  output->concat("\n");
}



/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void ManuManager::printStateMachine(StringBuilder* output) {
  output->concat("-- ManuState:\n");
  output->concatf("     Last      %s\n", getManuStateString(_last_state));
  output->concatf("     Current   %s\n", getManuStateString(_current_state));
  output->concatf("     Target    %s\n", getManuStateString(_target_state));
}



#if defined(MANUVR_CONSOLE_SUPPORT)
/*******************************************************************************
* Console I/O
*******************************************************************************/

static const ConsoleCommand console_cmds[] = {
  { "V", "IMU verbosity" },
  { "i", "General info" },
  { "i1", "Integrator info" },
  { "i2", "Ping sensors" },
  { "i3", "Print sensor roll-call" },
  { "i4", "Frame pool info" },
  { "i5", "Type sizes" },
  { "i6", "FIFO levels" },

  { "E", "Set data encoding" },

  { "j", "Per-datum reflection parameters" },
  { "[", "Enable spherical abberation correction" },
  { "]", "Disable spherical abberation correction" },
  { "Z", "Enable autoscale" },
  { "z", "Disable autoscale" },
  { "N", "Enable range-binding" },
  { "n", "Disable range-binding" },
  { "X", "Enable gyro error compensation" },
  { "x", "Disable gyro error compensation" },
  { "Y", "Enable bearing nullification" },
  { "y", "Disable bearing nullification" },

  { "Q", "Set Madjwick iterations" },
  { ",", "Quats per event" },
  { "b", "Set Madjwick beta" },
  { "L", "Set sample rate profile" },
  { "o", "Set GYR base filter" },
  { "O", "Set ACC base filter" }
};


uint ManuManager::consoleGetCmds(ConsoleCommand** ptr) {
  *ptr = (ConsoleCommand*) &console_cmds[0];
  return sizeof(console_cmds) / sizeof(ConsoleCommand);
}


void ManuManager::consoleCmdProc(StringBuilder* input) {
  char* str = input->position(0);

  uint8_t temp_byte = 0;
  if (*(str) != 0) {
    temp_byte = atoi((char*) str+1);
  }

  StringBuilder parse_mule;
  if (input->count() > 1) {
    parse_mule.concat(input->position(1));
    parse_mule.split(",");
  }

  switch (*(str)) {
    case 'h':
    case 'H':
    case 'm':

    case 'g':    // Sync registers
      switch (temp_byte) {
        case 1:
          {
            SPIBusOp* op = _bus->new_op(BusOpcode::RX, this);
            op->setParams((CPLD_REG_IMU_DM_P_M | 0x80), 1, LEGEND_DATASET_IIU_COUNT, 0x27 | 0x80 | 0x40);
            op->setBuffer(_reg_block_m_status, 17);
            queue_io_job(op);
          }
          break;
        default:
          _advance_state_machine();
          break;
      }
      break;

    case 'M':
      switch (temp_byte) {
        case 1:
          _set_target_state(ManuState::IMU_INIT);
          break;
        case 2:
          _set_target_state(ManuState::READY_IDLE);
          break;
        case 3:
          _set_target_state(ManuState::READY_READING);
          break;
        case 4:
          _set_target_state(ManuState::READY_PAUSED);
          break;
        case 5:
          _set_target_state(ManuState::ASLEEP);
          break;
        default:
          _advance_state_machine();
          break;
      }
      printStateMachine(&local_log);
      break;

    case 'V':
      temp_byte = parse_mule.position_as_int(0);
      if (temp_byte < 17) {
        if (parse_mule.count() > 1) {
          int temp_int = parse_mule.position_as_int(1);
          imus[temp_byte].setVerbosity(temp_int);
        }
        local_log.concatf("Verbosity on IMU %d is %d.\n", temp_byte, imus[temp_byte].getVerbosity());
      }
      break;

    case 'v':
      temp_byte = parse_mule.position_as_int(0);
      integrator.setVerbosity(temp_byte);
      break;

    case 'i':
      switch (temp_byte) {
        case 1:
          integrator.printDebug(&local_log);
          break;
        case 2:
          local_log.concat("Reading sensor identities...\n");
          read_identities();  // Read the sensor's identity registers.
          break;
        case 3:
          printIMURollCall(&local_log);   // Show us the results, JIC
          break;
        case 4:
          _frame_pool.printDebug(&local_log);
          break;
        case 5:
          local_log.concatf("sizeof(ManuLegendPipe)\t%u\n", sizeof(ManuLegendPipe));
          local_log.concatf("sizeof(ManuLegend)  \t%u\n", sizeof(ManuLegend));
          local_log.concatf("sizeof(Integrator)  \t%u\n", sizeof(Integrator));
          local_log.concatf("sizeof(SensorFrame) \t%u\n", sizeof(SensorFrame));
          local_log.concatf("sizeof(LSM9DS1)     \t%u\n", sizeof(LSM9DS1));
          local_log.concatf("sizeof(RegPtrMap)   \t%u\n", sizeof(RegPtrMap));
          local_log.concatf("sizeof(_frame_buf_i)\t%u\n", sizeof(_frame_buf_i));
          break;
        case 6:
          dumpPreformedElements(&local_log);
          break;

        case 0:
        default:
          printDebug(&local_log);
          break;
      }
      break;

    // Legend DEBUG //////////////////////////////////////////////////////////////////
    case 'l':
      switch (temp_byte) {
        case 1:
          //_def_pipe.broadcast_legend();
          //local_log.concat("Legend broadcast.\n");
          break;
        case 2:
          //_def_pipe.offer(_frame_pool.take());
          //local_log.concat("Cycled blank frame.\n");
          break;
        case 3:
          _def_pipe.decoupleSeq(!_def_pipe.decoupleSeq());
          local_log.concatf("decoupleSeq() %c\n", _def_pipe.decoupleSeq() ? 'y' : 'n');
          break;
        case 4:
          {
            uint8_t test_array[37];
            random_fill(&test_array[1], 36);
            test_array[0] = 17;  // IMU count.
            StringBuilder shuttle(&test_array[0], 37);
            int ret = _root_leg.setLegendString(&shuttle);
            local_log.concat("setLegendString(");
            shuttle.printDebug(&local_log);
            local_log.concat(")\n");
            local_log.concatf("setLegendString(buf, %u) returns %d\n", shuttle.length(), ret);
          }
        case 5:
          {
            local_log.concat("LegendString:\t");
            StringBuilder shuttle;
            _root_leg.getLegendString(&shuttle);
            shuttle.printDebug(&local_log);
          }
          break;
        case 6:
          _def_pipe.active(!_def_pipe.active());
          local_log.concatf("active() %c\n", _def_pipe.active() ? 'y' : 'n');
          break;

        case 10:
        case 11:
        case 12:
        case 13:
          {
            ManuEncoding e = (ManuEncoding) (temp_byte - 10);
            _def_pipe.encoding(e);
            local_log.concatf("Switched to ManuEncoding::%s\n", ManuLegendPipe::encoding_label(e));
          }
          break;
        default:
          _root_leg.printManuLegend(&local_log);
          break;
      }
      break;

    // Integrator DEBUG //////////////////////////////////////////////////////////////////
    case 'I':
      switch (temp_byte) {
        case 1:
          local_log.concat("Pushing SensorFrame into integrator...\n");
          integrator.pushFrame(_frame_pool.take());
          break;
        case 2:
          local_log.concat("Cycling the integrator...\n");
          _event_integrator.fireNow();
          break;
        case 3:
          if (integrator.resultsWaiting()) {
            SensorFrame* frame = integrator.takeResult();
            _def_pipe.offer(frame);
            frame->wipe();
            _frame_pool.give(frame);
          }
          local_log.concatf("Integrator has %u frames available.\n", integrator.resultsWaiting());
          break;
        case 4:
          debugFrameCycle(true);
          local_log.concat("Frame cycle started.\n");
          integrator.pushFrame(_frame_pool.take());
          _event_integrator.enableSchedule(true);
          break;
        case 5:
          debugFrameCycle(false);
          local_log.concat("Frame cycle stopped.\n");
          _event_integrator.enableSchedule(false);
          break;
        default:
          break;
      }
      break;

    case 'E':
      switch (temp_byte) {
        case 1:
        case 2:
        case 3:
        case 4:
          {
            ManuEncoding e = (ManuEncoding) (temp_byte - 1);
            local_log.concatf("Switching to ManuEncoding::%s\n", ManuLegendPipe::encoding_label(e));
            _def_pipe.encoding(e);
          }
        default:
          local_log.concatf("Using ManuEncoding::%s\n", ManuLegendPipe::encoding_label(_def_pipe.encoding()));
      }
      break;



    // IMU DEBUG //////////////////////////////////////////////////////////////////
    case 's':
      switch (temp_byte) {
        case 0:
          break;
        case 1:
          printFIFOLevels(&local_log);
          break;
        case 2:
          printTemperatures(&local_log);   // Show us the temperatures.
          break;
        default:
          break;
      }
      break;

    case 'c':
      if (temp_byte < 17) {
        imus[temp_byte].dumpDevRegs(&local_log);
      }
      break;

    // IMU STATE CONTROL //////////////////////////////////////////////////////////
    case 'd':
      temp_byte = parse_mule.position_as_int(0);
      if (parse_mule.count() > 1) {
        int temp_int = parse_mule.position_as_int(1);

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
      if (temp_byte < 6) {
        init_iius();
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
      local_log.concatf("%sabling spherical abberation correction on all IIUs.\n", ((*(str) == '[') ? "En":"Dis"));
      integrator.correctSphericalAbberation((*(str) == '['));
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

    case 'x':
    case 'X':
      local_log.concatf("%sabling gyro error compensation on all IIUs.\n", ((*(str) == 'X') ? "En":"Dis"));
      integrator.nullGyroError((*(str) == 'X'));
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
      max_quats_per_event = temp_byte;
      local_log.concatf("IIU class now runs a maximum of %u quats per event.\n", max_quats_per_event);
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

    case 'f':
      switch (temp_byte) {
        case 255:
          event_iiu_read.fireNow();
          local_log.concat("IMU read schedule fired.\n");
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

    case 'e':
      if (temp_byte) {
        _event_integrator.alterSchedulePeriod(temp_byte*10);
        local_log.concatf("Set integrator schedule to once every %dms.\n", temp_byte*10);
      }
      else {
        _event_integrator.enableSchedule(false);  // Disable the periodic read.
        local_log.concat("Disabled integrator schedule.\n");
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

    case '(':   // CPLD debug
    case ')':   // CPLD debug
      {
        local_log.concat("Setting MAG IRQ to active-high, all axis enabled.\n");
        SPIBusOp* op = _bus->new_op(BusOpcode::TX, this);
        op->setParams(CPLD_REG_RANK_P_M, 1, 3, RegPtrMap::regAddr(RegID::M_INT_CFG) | 0x40);  // 1 byte per IMU.
        op->setBuffer(&_reg_block_m_irq_cfg[0], 3);
        queue_io_job(op);
      }
      break;

    default:
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
  bzero(_reg_block_ident, (2 * LEGEND_DATASET_IIU_COUNT));
  imuIdentitiesRead(false);

  // Because the identity address is the same for both aspects, and their addresses
  //   are continuous, we just read 1 byte from 34 sensors.
  SPIBusOp* op = _bus->new_op(BusOpcode::RX, this);
  op->setParams((CPLD_REG_IMU_DM_P_M | 0x80), 0x01, (2 * LEGEND_DATASET_IIU_COUNT), 0x8F);
  op->setBuffer(_reg_block_ident, (2 * LEGEND_DATASET_IIU_COUNT));
  return queue_io_job(op);
}



int8_t ManuManager::read_fifo_depth() {
  return queue_io_job(&_preformed_fifo_read);
}

int8_t ManuManager::read_ag_frame() {
  return queue_io_job(&_preformed_read_i);
}

int8_t ManuManager::read_mag_frame() {
  return queue_io_job(&_preformed_read_m);
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
  // The order we do this in matters.
  int8_t ret = -1;
  // Step 1: Enable SPI write, multiple-access and disable i2c.
  SPIBusOp* op = _bus->new_op(BusOpcode::TX, this);
  op->setParams((CPLD_REG_RANK_P_M | 0x40), 3, 3, RegPtrMap::regAddr(RegID::M_CTRL_REG3) | 0x40);  // 3 bytes per IMU.
  op->setBuffer(&_reg_block_m_ctrl3_5[0], 9);
  if (0 == queue_io_job(op)) {
    op = _bus->new_op(BusOpcode::TX, this);
    op->setParams((CPLD_REG_RANK_P_I | 0x00), 3, 3, RegPtrMap::regAddr(RegID::AG_CTRL_REG8));  // 3 bytes per IMU.
    op->setBuffer(&_reg_block_ag_ctrl8_9_10[0], 9);
    if (0 == queue_io_job(op)) {
      op = _bus->new_op(BusOpcode::TX, this);
      op->setParams((CPLD_REG_RANK_P_I | 0x00), 2, 3, RegPtrMap::regAddr(RegID::AG_INT1_CTRL));  // 2 bytes per IMU.
      op->setBuffer(&_reg_block_ag_interrupt_conf[0], 6);
      if (0 == queue_io_job(op)) {
        op = _bus->new_op(BusOpcode::TX, this);
        op->setParams((CPLD_REG_RANK_P_M | 0x40), 1, 3, RegPtrMap::regAddr(RegID::M_INT_CFG) | 0x40);  // 1 byte per IMU.
        op->setBuffer(&_reg_block_m_irq_cfg[0], 3);
        if (0 == queue_io_job(op)) {
          op = _bus->new_op(BusOpcode::TX, this);
          op->setParams((CPLD_REG_RANK_P_M | 0x40), 1, 3, RegPtrMap::regAddr(RegID::M_CTRL_REG1) | 0x40);  // 1 byte per IMU.
          op->setBuffer(&_reg_block_m_ctrl1[0], 3);
          if (0 == queue_io_job(op)) {
            op = _bus->new_op(BusOpcode::TX, this);
            op->setParams((CPLD_REG_RANK_P_I | 0x00), 1, 3, RegPtrMap::regAddr(RegID::AG_FIFO_CTRL));  // 1 byte per IMU.
            op->setBuffer(&_reg_block_fifo_ctrl[0], 3);
            ret = queue_io_job(op);
          }
        }
      }
    }
  }

  return ret;
}


/**
* Debug support method. This fxn is only present in debug builds.
* // TODO: This is a lie. Audit MANUVR_DEBUG usage and elaborate it by class to reduce build sizes.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void ManuManager::printIMURollCall(StringBuilder *output) {
  EventReceiver::printDebug(output);
  output->concat("-- Intertial integration units: id(I/M)\n--\n-- Dgt      Prx        Imt        Dst        Reports\n");
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
  //output->concatf("%c\n\n", _bus->digitExists(DigitPort::PORT_5) ? 'Y' : ' ');
  output->concatf("%c\n", _bus->digitExists(DigitPort::PORT_5) ? 'Y' : ' ');
}
