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
  282056    2816    9844  294716   47f3c   Testing the waters of Runnable collapse.
  281768    2800    9772  294340   47dc4   Following flattening of Runnable upstream.

_---J. Ian Lindsay_

------

### 2016.10.09:
  281120    2800    9764  293684   47b34   New baseline.
  281088    2800    9764  293652   47b14   Struck lstdc++ from the linker.
  281080    2800    9764  293644   47b0c   Struck lgcc from the linker.

I forgot why those linkages were ever there to begin with...

_---J. Ian Lindsay_

------

### 2016.10.23:

    281816    2800    9664  294280   47d88   New baseline.

_---J. Ian Lindsay_

------

### 2016.10.31:

RN driver... Prior to member re-work to use a Pin object.

    make DEBUG=1
    281920    2800    9664  294384 Prior to member re-work to use a Pin object.
    281984    2800    9664  294448 Following conversion.
    281856    2800    9664  294320 Following conversion of gpioSetup()
    281936    2800    9664  294400 GPIO treatment is now safe.
    282064    2800    9664  294528 GPIO mapper completed and safe.
    282600    2800    9664  295064 Power down and sleep support.

### 2016.11.04:

    283168    2800    9664  295632 New baseline after adding to PMU.
    283388    2800    9700  295888 Building up support for the BT module.

My god.... RNBase has bad smell. Three refactors died in there and were never cleaned out.

    283564    2800    9716  296080 Can stand it no longer... About to cull...
    283628    2800    9780  296208 My hasty class-merger actually compiled.
    283436    2800    9780  296016 Cut the xenomgs_id field and delegated constructors.
    283436    2800    9748  295984 Cut the redundant buffer members.
    283436    2800    9732  295968 Cut the last remaining boolean member.
    283316    2800    9732  295848 Cruft elimination. Memory management fixes.
    283348    2800    9732  295880 Consolidating some inlines. Adding granularity to IO_WAIT.
    283036    2800    9508  295344 Stripped some complexity from session notify().
    283492    2800    9496  295788 Migrated gpio_5 fxns out of RNBase. Pruned cruft.
    283388    2800    9496  295684 Relocation of hardware-specifics.
    283236    2800    9496  295532 Cutting some needless string.

_---J. Ian Lindsay_

------

### 2016.11.23:

Given the pending manufacture of the digits, about to revisit some things in the IMU data pathways. First, some baselines...

    274996    2800    9456  287252   46214  make
    283220    2800    9496  295516   4825c  make DEBUG=1

    make DEBUG=1
    283220    2800    9496  295516   4825c  New baseline.
    283220    2800    9496  295516   4825c  Rework of if-else chains into switch-case yields no difference.

I very much dislike this arcane key-sequence console.

It might be time to formally abandon the notion of individual IMUs being discrete bus entities, and allow the LegendManager class to be the intermediary (and memory store) for all IIU classes, which would do no allocation for these purposes.

Feed the base pointer in for register space, and let the IMU class calculate appropriate offsets from the base given the IMU count supported. I won't delve into the chain of inference that leads me to this conclusion here, but doing this would have the consequence that all IMUs must be run at the same sample-rate. This might be a sane assumption anyhow.

    Reworking some basics in IIU/IMU classes....
    283476    2800    9496  295772   4835c  Moved boolean members to flags.
    283556    2800    9496  295852   483ac  Making some class members constant.
    283492    2800    9496  295788   4836c  Alignment change to make gains evident.
    283492    2800    9496  295788   4836c  More flag condensation.
    283540    2800    9496  295836   4839c  CPLD conversion to BusAdapter.
    283476    2800   11072  297348   48984  Static allocation of IMU register memory.

_---J. Ian Lindsay_

------

### 2016.11.28:

The CPLD driver is now fully separated from the concerns of IMUs. It is a general bus adapter.

Re-wrote some IIU lookup functions as inlines. They were not referenced anywhere, so they may disappear completely. Now that the drivers don't have to deal with the heterogeneous bus topology that r0 exhibited, the drivers can more cleanly carved into functional units.

    283508    2800   11080  297388   489ac  Rounding out some CPLD function defs.
    283700    2800   35760  322260   4ead4  Temporarily made IIU class static to see mem impact.

And there we have it... All IMU/IIU driver classes are now stack-defined and will not burden the heap.
Turning in for the night.

_---J. Ian Lindsay_

    283572    2800   42960  329332   50674  Moved InertialMeasurement queue to static for a bit.

    284916    2728   42576  330220   New baseline ahead of Manuvr BusOp consolidation.
    284972    2728   43196  330896   Following BusOp consolidation. No RNBase yet.
    285004    2728   43196  330928   Additions to template. About to convert RNBase.
    285604    2728   43160  331492   RNBase now extends BusAdapter.
    285660    2728   43156  331544   52-bytes of flash for 4-bytes of RAM? Yes.
    285436    2728   43156  331320   Taking some gains back from templates.
    285436    2728   43156  331320   Pushing more code replication back into template.
    286036    2728   43156  331920   More untanglement.

### 2016.01.14:

Took a few weeks to do hardware stuff. Digit and metacarpals flex units arrive and test good, aside from my goof with the mirrored connector. Bus operations do not cycle out of the queue after completing successfully. So that's my immediate next task. But I'm going to spend some time culling obsolete code first, to come up to speed.

    make DEBUG=1
    286988    2728   43144  332860   Tonight's baseline.
    287892    2728   43184  333804   Expanded IRQ work.
    287284    2728   43152  333164   Fixed SPI1 for the moment.

_---J. Ian Lindsay_

------

### 2016.01.18:

Bus is fixed (enough). DMA handler still broken on every-other transfer. Code needs to migrate inward from the IMU driver classes, toward ManuManager, and ManuManager need to undergo mitosis to isolate the hardware layer from data concerns. The CPLD in r1 allows us to (in principle) run a single class instance for all the sensor handling that operates on aggregates, rather than r0's one-instance-per-physical-device.

IIU's connective role is vestigial, and its data processing role is no longer confined to individual IMUs. Given that it already has an established relationship with ManuManager, it will (for now) be retained as the front-end of the data pipeline (as it was before), but will be fed by ManuManager, rather than the existing driver classes.

    make DEBUG=1
    287284    2728   43152  333164   Tonight's baseline.

_---J. Ian Lindsay_

------

### 2016.01.28:

    make DEBUG=1
    287284    2728   43152  333164   Former baseline.
    280156    2656   30376  313188   Compiles again following memory re-org.
    280092    2656   30336  313084   Consolidated acc and gyr read ops into one.
    280132    2880   30336  313348   Removed stop-gap mem space. Cleaned up some TODO.
    280228    2880   30272  313380   Cutting fields that don't make sense anymore.
    280220    2880   30272  313372   More field remeval ahead of migrations to Integrator.
Lots of things are broken. There are a few stubs, and much dead code. IMU pointer
maps are contained within a const class and array'd out as non-static members.

Debriding and suturing...

_---J. Ian Lindsay_
