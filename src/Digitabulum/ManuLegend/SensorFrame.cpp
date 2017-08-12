/*
File:   SensorFrame.cpp
Author: J. Ian Lindsay
Date:   2017.01.31

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


#include "SensorFrame.h"

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

uint32_t SensorFrame::_total_sequences = 0;

void SensorFrame::resetSequenceCounter() {
  _total_sequences = 0;
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
* Vanilla constructor. Vector class will initiallize conponents to zero.
*/
SensorFrame::SensorFrame() {
  _read_time = 0.0f;
  _seq = 0;
}

/**
* Wipes the instance. Sets all member data to 0.0.
*/
void SensorFrame::wipe() {
  _read_time = 0.0f;
  for (int i = 0; i < 17; i++) {
    a_data[i](0.0f, 0.0f, 0.0f);
    g_data[i](0.0f, 0.0f, 0.0f);
    m_data[i](0.0f, 0.0f, 0.0f);
    v_data[i](0.0f, 0.0f, 0.0f);
    n_data[i](0.0f, 0.0f, 0.0f);
    p_data[i](0.0f, 0.0f, 0.0f);
    quat[i].set(0.0f, 0.0f, 0.0f, 0.0f);
  }
}


#if defined(MANUVR_IMU_DEBUG)
void SensorFrame::printDebug(StringBuilder* output) {
  output->concatf("SensorFrame (seq %u)  Delta-t = %f", _seq, (double) _read_time);
  StringBuilder a_line;
  StringBuilder g_line;
  StringBuilder m_line;
  output->concatf("HP(%.4f, %.4f, %.4f)\n", (double)hand_position.x, (double)hand_position.y, (double)hand_position.z);
  for (int i = 0; i < 17; i++) {
    switch (i) {
      case 2:   // digit1 begins
      case 5:   // digit2 begins
      case 8:   // digit3 begins
      case 11:  // digit4 begins
      case 14:  // digit5 begins
        a_line.concatHandoff(&g_line);
        a_line.concatHandoff(&m_line);
        a_line.string();
        output->concatHandoff(&a_line);
      default:
        break;
    }
    switch (i) {
      case 1:   // Skip output for the IMU that doesn't exist at digit0.
        a_line.concat("          <N/A>            ");
        g_line.concat("          <N/A>            ");
        m_line.concat("          <N/A>            ");
        break;
      case 0:
        a_line.concat("\n-- 0(MC)    ");
        g_line.concat("\n            ");
        m_line.concat("\n            ");
        break;
      case 2:   // digit1 begins
        a_line.concat("\n-- 1        ");
        g_line.concat("\n            ");
        m_line.concat("\n            ");
        break;
      case 5:   // digit2 begins
        a_line.concat("\n-- 2        ");
        g_line.concat("\n            ");
        m_line.concat("\n            ");
        break;
      case 8:   // digit3 begins
        a_line.concat("\n-- 3        ");
        g_line.concat("\n            ");
        m_line.concat("\n            ");
        break;
      case 11:  // digit4 begins
        a_line.concat("\n-- 4        ");
        g_line.concat("\n            ");
        m_line.concat("\n            ");
        break;
      case 14:  // digit5 begins
        a_line.concat("\n-- 5        ");
        g_line.concat("\n            ");
        m_line.concat("\n            ");
        break;
      default:
        break;
    }
    a_line.concatf("A(%.4f, %.4f, %.4f)  ", (double)a_data[i].x, (double)a_data[i].y, (double)a_data[i].z);
    g_line.concatf("G(%.4f, %.4f, %.4f)  ", (double)g_data[i].x, (double)g_data[i].y, (double)g_data[i].z);
    m_line.concatf("M(%.4f, %.4f, %.4f)  ", (double)m_data[i].x, (double)m_data[i].y, (double)m_data[i].z);
  }
}
#endif  // MANUVR_IMU_DEBUG
