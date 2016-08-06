/*
File:   RNBase.h
Author: J. Ian Lindsay
Date:   2014.05.22

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


TODO: This class is in SORE need of the following things:
        1) Proper abstraction WRT serial ports
        2) Proper abstraction WRT GPIO
        3) Proper inherritence relationship WRT Xport/EventReceiver
        4) Scope audit

      It was hastily grafted together in the first place. Make it not suck so
        that it can be easilly extended to cover the RN4020, RN4677, RN41, and
        RN42-HID (from which it came).
*/

#ifndef __RNBASE_H__
#define __RNBASE_H__

#include <inttypes.h>
#include <DataStructures/PriorityQueue.h>
#include "BTQueuedOperation.h"
#include <Drivers/BusQueue/BusQueue.h>
#include <Kernel.h>
#include <Transports/ManuvrXport.h>


#define MANUVR_MSG_BT_EXIT_RESET     0x4295

// Bluetooth Modes
#define RNBASE_MODE_COMMAND       "$$$"
#define RNBASE_MODE_EXITCOMMAND   "---\r\n"
#define RNBASE_MODE_SPP           "S~,0\r\n"
#define RNBASE_MODE_HID           "S~,6\r\n"
#define RNBASE_MODE_AUTOCONNECT   "SM,6\r\n"
#define RNBASE_MODE_MANUCONNECT   "SM,4\r\n"
#define RNBASE_MODE_STATUS        "SO,/#\r\n"

#define RNBASE_AUTH_MODE_SIMPLE   "SA,1\r\n"  // 6-digit pin code
#define RNBASE_AUTH_MODE_JUSTWORK "SA,2\r\n"  // No auth
#define RNBASE_AUTH_MODE_PININPUT "SA,3\r\n"  // We must feed code to the RN module.
#define RNBASE_AUTH_MODE_LEGACY   "SA,4\r\n"  // 4 digit pin code

// Specific command to the module
#define RNBASE_CMD_RECONNECT        "C\r\n"   // Tries to reconnect to the address given by "GR".
#define RNBASE_CMD_CHANGE_NAME      "SN,"
#define RNBASE_CMD_GET_FIRMWARE_REV "V\r\n"
#define RNBASE_CMD_DEV_SCAN         "IQ\r\n"
#define RNBASE_CMD_KILL_CURNT_CONN  "K,\r\n"
#define RNBASE_CMD_REMOTE_MODEM_STS "M\r\n"
#define RNBASE_CMD_HELP             "H\r\n"
#define RNBASE_CMD_CONNECT_STATUS   "GK\r\n"
#define RNBASE_CMD_BASIC_SETTINGS   "D\r\n"
#define RNBASE_CMD_EXTNDED_SETTINGS "E\r\n"
#define RNBASE_CMD_OTHER_SETTINGS   "O\r\n"    // Show other settings.
#define RNBASE_CMD_GET_OUR_BT_ADDR  "GB\r\n"   // Gets our BT MAC.
#define RNBASE_CMD_GET_CP_BT_ADDR   "GF\r\n"   // Gets the BT MAC of the currently connected device.
#define RNBASE_CMD_GET_STOR_BT_ADDR "GR\r\n"   // Gets the BT MAC of the currently bound device.
#define RNBASE_CMD_REBOOT         "R,1\r\n"


#define RNBASE_PROTO_SPP            "AW\r\n"

// Responses that we might get back from the module.
#define RNBASE_STATUS_ACK         "AOK\r\n"
#define RNBASE_STATUS_CMD         "CMD\r\n"
#define RNBASE_STATUS_END         "END\r\n"
#define RNBASE_STATUS_REBOOT      "Reboot!\r\n"


// These are only relevant for debug.
#define RNBASE_MAX_QUEUE_PRINT 3

// Resting memory load parameters.
#define PREALLOCATED_BT_Q_OPS    4    // How many data-carriers should we preallocate?
#define RNBASE_MAX_BT_Q_DEPTH    5    //


/*
* These state flags are hosted by the EventReceiver. This may change in the future.
* Might be too much convention surrounding their assignment across inherritence.
*/
#define RNBASE_FLAG_LOCK_OUT  0x01    // While this is true, don't interact with the RN.
#define RNBASE_FLAG_CMD_MODE  0x02    // Set when the module is verified to be in command mode.
#define RNBASE_FLAG_CMD_PEND  0x04    // Set when we are expecting the module to enter command mode.
#define RNBASE_FLAG_AUTOCONN  0x08    // Should we connect whenever possible?
#define RNBASE_FLAG_REST_PEND 0x10    // Is a reset pending?
#define RNBASE_FLAG_FORCE9600 0x20    // Are we forced into 9600 mode?
#define RNBASE_FLAG_RESERVED0 0x40    //
#define RNBASE_FLAG_RESERVED1 0x80    //


#define RNBASE_CMD_PRIORITY      10   // Command transactions must have a higher priority than CP messages so the queue doesn't block.
#define RNBASE_SYNC_PRIORITY     9    // Sync packets are the most important class of CP message.
#define RNBASE_RETRY_PRIORITY    6    // Retries should have a higher priority to maintain order WRT other CP messages.
#define RNBASE_DEFAULT_PRIORITY  2    // New CP messages have a low priority, and will stack in their natural order.


#define CHARACTER_CHRONOLOGICAL_BREAK 50   // How many ms must pass before we consider the read buffer flushable?


/*
* This is the RN driver. It is an abstraction layer for the Microchip/RovingNetworks integrated
*   bluetooth modules. This class contains pure-virtual members, and must be extended by a driver
*   for a specific module (RN4677, RN4020, RN42HID, etc...).
* Target list:
*   RN4677
*   TODO: RN4020
*   TODO: RN42HID
*
*
*/
class RNBase : public ManuvrXport {
  public:
    RNBase(uint8_t _rst_pin);
    virtual ~RNBase();

    /* Override from BufferPipe. */
    virtual int8_t toCounterparty(StringBuilder* buf, int8_t mm);
    virtual int8_t fromCounterparty(StringBuilder* buf, int8_t mm);

    /* Overrides from the Transport class. */
    virtual int8_t connect();
    //virtual int8_t disconnect();
    virtual int8_t listen();
    virtual int8_t reset();
    bool   write_port(unsigned char* out, int out_len);
    int8_t read_port();

    /* Overrides from EventReceiver */
    void printDebug(StringBuilder *);
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable *);
    #if defined(__MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //__MANUVR_CONSOLE_SUPPORT

    /* These are used to send data to a BT connected device. */
    inline bool roomInQueue() {    return !(work_queue.size() < RNBASE_MAX_BT_Q_DEPTH);  }

    /* Macros for RN commands. */
    void setDevName(char*);
    void sendRebootCommand(void);
    void sendGeneralCommand(const char*);
    void sendGeneralCommand(StringBuilder*);


    static BTQueuedOperation* current_work_item;
    volatile static unsigned long last_gpio_5_event;

    // Volatile statics that serve as ISRs...
    volatile static void irqServiceBT_data_activity(void);
    volatile static void irqServiceBT_data_receive(unsigned char* str, uint8_t len);
    volatile static void usart2_character_rx(unsigned char c);
    volatile static void isr_bt_queue_ready();
    volatile static void bt_gpio_5(unsigned long);

    static void hostRxFlush();
    static void expire_lockout();

    static inline RNBase* getInstance() { return (RNBase*)INSTANCE; };


  protected:
    int configured_bitrate;   // The bitrate we have between the CPU and the RN.

    int8_t idleService();
    void feed_rx_buffer(unsigned char*, uint8_t len);   // Append to the class receive buffer.
    virtual int8_t sendBuffer(StringBuilder*);


    virtual int8_t bootComplete();      // This is called from the base notify().

    // Mandatory overrides.
    virtual void factoryReset(void)    =0;   // Perform the sequence that will factory-reset the RN.
    virtual void gpioSetup();
    virtual void force_9600_mode(bool) =0;   // Call with 'true' to force the module into 9600bps.
    virtual void set_bitrate(int)      =0;   //

    /*
    * These are used as convenience overrides for distinguishing
    *   between destinations (module or counterparty)
    */
    inline void printToHost(char* str) {
      insert_into_work_queue(BusOpcode::TX, new StringBuilder(str));
    };

    inline void printToHost(StringBuilder* str) {
      insert_into_work_queue(BusOpcode::TX, str);
    };


  private:
    ManuvrRunnable event_bt_queue_ready;

    uint8_t _reset_pin = 0;

    /* Members concerned with the work queue and keeping messages atomic. */
    PriorityQueue<BTQueuedOperation*> work_queue;

    StringBuilder tx_buf;     // A scratchpad for this class.

    uint32_t insert_into_work_queue(BusOpcode opcode, StringBuilder* data);
    int8_t burn_or_recycle_current();   // Called during connection turbulence to handle the queued item.

    //int8_t init_dma(uint8_t* buf, unsigned int len);
    void start_lockout(uint32_t milliseconds);   // Hold communication with the RN for so many ms.

    /* Macros for RN radio states. */
    void master_mode(bool);       // Call with 'true' to switch the radio into master mode.
    void setHIDMode(void);
    void setSPPMode(void);
    void setAutoconnect(bool);

    int8_t sendBreak();           // Send a break stream to the counterparty if things get hosed.

    int8_t enterCommandMode();    // Convenience fxn for entering command mode.
    int8_t exitCommandMode();     // Convenience fxn for exiting command mode.



    volatile static RNBase* INSTANCE;

    static uint32_t rejected_host_messages;

    // Prealloc starvation counters...
    static uint32_t prealloc_starves;
    static uint32_t queue_floods;

    static uint32_t _heap_instantiations;
    static uint32_t _heap_frees;

    static PriorityQueue<BTQueuedOperation*> preallocated;     // Messages that we've allocated ahead of time.
    static BTQueuedOperation __prealloc_pool[PREALLOCATED_BT_Q_OPS];

    static BTQueuedOperation* fetchPreallocation();
    static void reclaimPreallocation(BTQueuedOperation*);

};

#endif  //__RNBASE_H__
