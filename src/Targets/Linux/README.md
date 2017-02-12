/* \mainpage Firmware Documentation

    .__       ,    .     .
    |  \* _ *-+- _.|_ . .|. .._ _
    |__/|(_]| | (_][_)(_||(_|[ | )
         ._|                      


Digitabulum is an inertial motion-capture glove designed to be modular and open.

----------------------

## Building the device firmware under linux
This platform is meant to speed the development cycle where hardware is not a concern. Debugging comm stacks, memory leaks, etc...

    # Builds the emulator with debugging support
    make PLATFORM=LINUX DEBUG=1

If that succeeded, you can run the emulator...

    ./digitabulum --console
