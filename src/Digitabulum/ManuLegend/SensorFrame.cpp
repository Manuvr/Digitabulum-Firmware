/*
File:   SensorFrame.cpp
Author: J. Ian Lindsay
Date:   2014.05.12

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


This is a container class for a single IMU measurement frame.
*/


#include "SensorFrame.h"

/**
* Vanilla constructor. Vector class will initiallize conponents to zero.
*/
SensorFrame::SensorFrame() {
  read_time = 0.0f;
}

/**
* Wipes the instance. Sets all member data to 0.0.
*/
void SensorFrame::wipe() {
  read_time = 0.0f;
  for (int i = 0; i < 17; i++) {
    a_data[i](0.0f, 0.0f, 0.0f);
    g_data[i](0.0f, 0.0f, 0.0f);
    m_data[i](0.0f, 0.0f, 0.0f);
  }
}


#if defined(__MANUVR_DEBUG)
void SensorFrame::printDebug(uint8_t i, StringBuilder* output) {
  output->concatf("Measurement taken at %uus\n", (unsigned long) read_time);
  output->concatf("  G: (%f, %f, %f) taken at %uus\n", (double)g_data[i].x, (double)g_data[i].y, (double)g_data[i].z);
  output->concatf("  A: (%f, %f, %f) taken at %uus\n", (double)a_data[i].x, (double)a_data[i].y, (double)a_data[i].z);
  output->concatf("  M: (%f, %f, %f) taken at %uus\n", (double)m_data[i].x, (double)m_data[i].y, (double)m_data[i].z);
}
#endif
