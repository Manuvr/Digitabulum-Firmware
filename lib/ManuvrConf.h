/*
File:   FirmwareDefs.h
Author: J. Ian Lindsay
Date:   2015.03.01


This is one of the files that the application author is required to provide.
This is where definition of (application or device)-specific parameters ought to go.
*/

#ifndef __FIRMWARE_DEFS_H
#define __FIRMWARE_DEFS_H

/*
* Particulars of this Manuvrable.
*/

//#define MANUVR_STORAGE
#define EVENT_MANAGER_PREALLOC_COUNT     8
#define SCHEDULER_MAX_SKIP_BEFORE_RESET  6
#define PLATFORM_RNG_CARRY_CAPACITY     32
#define WITH_MBEDTLS
//#define WITH_BLIND_CRYPTO
//#define MANUVR_OVER_THE_WIRE
//#define MANUVR_SUPPORT_MQTT
//#define MANUVR_SUPPORT_COAP
#define MANUVR_CONSOLE_SUPPORT
#define MANUVR_STDIO
//#define MANUVR_SUPPORT_SERIAL
//#define MANUVR_SUPPORT_TCPSOCKET
//#define MANUVR_SUPPORT_UDP
#define MANUVR_SUPPORT_I2C
//#define MANUVR_GPS_PIPE
#define MANUVR_CBOR
#define __MANUVR_FREERTOS
//#define MANUVR_JSON


#define CPLD_SPI_PREALLOC_COUNT   10
#define CPLD_SPI_MAX_QUEUE_DEPTH  40
#define CPLD_SPI_MAX_QUEUE_PRINT   3
#define PREALLOCD_IMU_FRAMES      11

#define MANUVR_EVENT_PROFILER
#define MANUVR_DEBUG
#define MANUVR_TEST_DRIVER
//#define MANUVR_PIPE_DEBUG
#define MANUVR_IMU_DEBUG

// This is the string that identifies this Manuvrable to other Manuvrables. In MHB's case, this
//   will select the mEngine.
#define FIRMWARE_NAME     "digitabulum-r2"    // This will select Manuvr's debug engine in MHB.

// This would be the version of the Manuvrable's firmware (this program).
#define VERSION_STRING    "0.0.1"

// Hardware is versioned. Manuvrables that are strictly-software should say -1 here.
#define HW_VERSION_STRING "2"


#endif
