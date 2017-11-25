/*
File:   IRQRouter.h
Author: J. Ian Lindsay
Date:   2017.10.05

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


IRQ definitions
--------------------------------------------------------------------------------
00  PORT_5
01  PORT_5
02  PORT_5
03  PORT_5
04  PORT_5
05  PORT_5
06  PORT_5
07  PORT_5
08  PORT_5
09  PORT_5
10  PORT_5
11  PORT_5
12  PORT_4
13  PORT_4
14  PORT_4
15  PORT_4
16  PORT_4
17  PORT_4
18  PORT_4
19  PORT_4
20  PORT_4
21  PORT_4
22  PORT_4
23  PORT_4
24  PORT_3
25  PORT_3
26  PORT_3
27  PORT_3
28  PORT_3
29  PORT_3
30  PORT_3
31  PORT_3
32  PORT_3
33  PORT_3
34  PORT_3
35  PORT_3
36  PORT_2
37  PORT_2
38  PORT_2
39  PORT_2
40  PORT_2
41  PORT_2
42  PORT_2
43  PORT_2
44  PORT_2
45  PORT_2
46  PORT_2
47  PORT_2
48  PORT_1
49  PORT_1
50  PORT_1
51  PORT_1
52  PORT_1
53  PORT_1
54  PORT_1
55  PORT_1
56  PORT_1
57  PORT_1
58  PORT_1
59  PORT_1
60  MC
61  MC
62  MC
63  MC
64  MC
65  MC
66  MC
67  MC
68  Metacarpals present.
69  Digit 1 present.
70  Digit 2 present.
71  Digit 3 present.
72  Digit 4 present.
73  Digit 5 present.
74  CONFIG register, bit 2.
75  CPLD_OE
76  0
77  0
78  CPLD_GUARD_BIT_VALUE[0]
79  CPLD_GUARD_BIT_VALUE[1]

*/


#ifndef __CPLD_IRQ_ROUTER_H__
#define __CPLD_IRQ_ROUTER_H__

#define CPLD_GUARD_BIT_VALUE  0x03

//TODO: These values are not correct, but the pattern is.
#define CPLD_IRQ_BITMASK_DRDY_MASK_0   0x11111111  //
#define CPLD_IRQ_BITMASK_INTM_MASK_0   0x22222222  //
#define CPLD_IRQ_BITMASK_INT1_MASK_0   0x44444444  //
#define CPLD_IRQ_BITMASK_INT2_MASK_0   0x88888888  //
#define CPLD_IRQ_BITMASK_DRDY_MASK_1   0x11111111  //
#define CPLD_IRQ_BITMASK_INTM_MASK_1   0x22222222  //
#define CPLD_IRQ_BITMASK_INT1_MASK_1   0x44444444  //
#define CPLD_IRQ_BITMASK_INT2_MASK_1   0x88888888  //
#define CPLD_IRQ_BITMASK_DRDY_MASK_2   0x01        //
#define CPLD_IRQ_BITMASK_INTM_MASK_2   0x02        //
#define CPLD_IRQ_BITMASK_INT1_MASK_2   0x04        //
#define CPLD_IRQ_BITMASK_INT2_MASK_2   0x08        //

#define CPLD_IRQ_BITMASK_IMU_0_MASK    0x0000000F  //
#define CPLD_IRQ_BITMASK_IMU_1_MASK    0x000000F0  //
#define CPLD_IRQ_BITMASK_IMU_2_MASK    0x00000F00  //
#define CPLD_IRQ_BITMASK_IMU_3_MASK    0x0000F000  //
#define CPLD_IRQ_BITMASK_IMU_4_MASK    0x000F0000  //
#define CPLD_IRQ_BITMASK_IMU_5_MASK    0x00F00000  //
#define CPLD_IRQ_BITMASK_IMU_6_MASK    0x0F000000  //
#define CPLD_IRQ_BITMASK_IMU_7_MASK    0xF0000000  //
#define CPLD_IRQ_BITMASK_IMU_8_MASK    0x0000000F  //
#define CPLD_IRQ_BITMASK_IMU_9_MASK    0x000000F0  //
#define CPLD_IRQ_BITMASK_IMU_10_MASK   0x00000F00  //
#define CPLD_IRQ_BITMASK_IMU_11_MASK   0x0000F000  //
#define CPLD_IRQ_BITMASK_IMU_12_MASK   0x000F0000  //
#define CPLD_IRQ_BITMASK_IMU_13_MASK   0x00F00000  //
#define CPLD_IRQ_BITMASK_IMU_14_MASK   0x0F000000  //
#define CPLD_IRQ_BITMASK_IMU_15_MASK   0xF0000000  //
#define CPLD_IRQ_BITMASK_IMU_16_MASK   0x0F        //



  /* These values represent where in the IRQ buffer this IIU's bits lie. */
  inline uint8_t _irq_offset_byte(int idx) {  return (idx >> 1);     };
  inline uint8_t _irq_offset_bit(int idx) {   return (idx << 2);     };

  void _digit_irq_force(uint8_t digit, bool state);

#endif   // __CPLD_IRQ_ROUTER_H__
