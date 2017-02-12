/* \mainpage Firmware Documentation

    .__       ,    .     .
    |  \* _ *-+- _.|_ . .|. .._ _
    |__/|(_]| | (_][_)(_||(_|[ | )
         ._|                      

----------------------

## Building for STM32F7
You must specify the path to the STM32 toolchain...

    export STM32GCCPATH=/opt/compilers/stm32f7

    # Builds for hardware r1.
    make PLATFORM=STM32F7

    # Builds for STM32F7 Discovery board
    make PLATFORM=STM32F7 BOARD=DISCO

If that succeeded and your system has DFU support, you can then plug your glove into a USB port, press the bootloader button, and...

    make program
