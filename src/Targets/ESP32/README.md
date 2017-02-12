/* \mainpage Firmware Documentation

    .__       ,    .     .
    |  \* _ *-+- _.|_ . .|. .._ _
    |__/|(_]| | (_][_)(_||(_|[ | )
         ._|                      

----------------------

## Building for ESP32
You must specify the path to the ESP32 toolchain. I have mine setup this way...

    export IDF_PATH=~/.esptools
    export PATH=$PATH:$IDF_PATH/xtensa-esp32-elf

    # Configure the build...
    make menuconfig PLATFORM=ESP32

    # Builds using esp-idf
    make PLATFORM=ESP32

If that succeeded, you can then plug your glove into a USB port, press the bootloader button, and...

    make flash PLATFORM=ESP32
