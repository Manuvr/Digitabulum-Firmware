# This is not a CHANGELOG
It is a verbose running commentary with no relationship to versioning.
File should be read top-to-bottom for chronological order.
The contents of this file will be periodically archived and purged to keep the log related to the "current events" of the code base.

---J. Ian Lindsay 2016.08.27

------

### 2016.06.17

   text    data     bss     dec     hex
 293348    2848    9668  305864   4aac8 New Baseline
 292784    2848    9668  305300   4a894 Removal of software-driven CPLD clock. Timer works well-enough.
 292812    2848    9668  305328   4a8b0 More cruft-removal.

------

### 2016.07.07
 293536    2848    9796  306180   4ac04 CPLD is at r9.
 293096    2848    9788  305732   4aa44 Removed soft SPI.

------

### 2016.07.08
 287684    2848    9852  300384   49560 SPIDeviceWithRegisters... Initial removal phase.
 287696    2848    9852  300396   4956c SPIDeviceWithRegisters has been cut. But IMUs are broken.

------

### 2016.07.09
 287480    2848    9852  300180   49494 SPI2 conversion to DMA.
 287392    2848    9852  300092   4943c More cleanup.

------

### 2016.07.15
 291496    2848    9952  304296   Prior to up-ending ManuvrOS with BufferPipes.
 293680    2848    9952  306480   BufferPipes have been grafted.

------

### 2016.07.29
 288636    2848    9324  300808   BufferPipe conversion complete. USB class created.
 287852    2848    9308  300008   Cut some of the Kernel logging members.

------

### 2016.08.08
 268808    2816    9500  281124   Major rework in ManuvrOS.

------

### 2016.08.16
 265240    2816    9432  277488   More cleanup and fat trimming.

------

### 2016.08.28
 266104    2856    9420  278380   After platform re-work.

------

### 2016.09.09
 271744    2856   10072  284672   New baseline after massive upheaval in Manuvr.

This needs to be investigated. But I'm willing to bet that it's a simple goof I will find on accident if I ignore it for awhile. I have the whole weekend to knock down some hard problems. Let's see where this goes...

 266344    2856   10072  279272   As expected... Some cleanup in Manuvr yields this.

------

### 2016.09.10
   266752    2816   10076  279644   Following the platform abstraction.

------

### 2016.09.24
Tuned out of this project for two weeks for work and hardware issues. ManuvrOS
  saw steady improvements in that period.
  273192    2816    9836  285844   45c94   New baseline
  273200    2816    9832  285848   45c98   Moved XPORT_MSG defs into Kernel.
  279032    2816    9840  291688   47368   Re-added most of the driver stubs.
