/*
File:   RN4677.h
Author: J. Ian Lindsay
Date:   2016.04.17

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


#include "../RNBase.h"

#ifndef __RN4677_H__
#define __RN4677_H__

#define RN4677_BT_SET_MODE        "SG,"
#define RN4677_BT_SET_ISW         "SI,"     // Set inquiry scan window.
#define RN4677_BT_SET_PSW         "SJ,"     // Set page scan window.

#define RNBASE_CMD_CHANGE_NAME      "SN,"
#define RNBASE_CMD_SET_STATUS_DELIM "SO,"   // Set extended status string.
#define RNBASE_CMD_SET_SEC_PIN      "SP,"   // Set security pin.


#define RNBASE_CMD_SET_DEV_CLASS  "SC,"       // Followed by 16-bit BT COD.
#define RNBASE_CMD_SET_UUID_SVC   "SE,"       // Followed by 16-bit service UUID.
#define RNBASE_CMD_FACTORY_RESET  "SF,1\r\n"  // Factory reset.

/*
*
*/
enum class RN4677ModuleMode {
  LINK_DATA    = 0x00,
  LINK_NO_DATA = 0x01,
  ACCESS       = 0x02,
  SHUTDOWN     = 0x03
};


/*
*
*/
enum class RN4677ProtoMode {
  DUAL    = 0x00,
  BLE     = 0x01,
  CLASSIC = 0x02
};

/*
*
*/
enum class RN4677AuthMode {
  SIMPLE   = 0x01,
  JUSTWORK = 0x02,
  PININPUT = 0x03,
  LEGACY   = 0x04
};

/*
*
*/
enum class RN4677ConnectMode {
  SLAVE     = 0x00,
  MASTER    = 0x01,
  AC_MASTER = 0x03,
  PAIRING   = 0x06
};

/*
*
*/
enum class RN4677TxPower {
  PWR_0 = 0x00,
  PWR_1 = 0x01,
  PWR_2 = 0x02,
  PWR_3 = 0x03,
  PWR_4 = 0x04
};


/*
* Pin defs for this module.
*/
class RN4677Pins : public RNPins {
  public:
    int8_t sbt = -1; // WO, SW_BTN
    int8_t swu = -1; // WO, software wake-up.
    int8_t p04 = -1; // RO, Status 0
    int8_t p15 = -1; // RO, Status 1
    int8_t led = -1; // RO, LED_State

    int8_t p20 = -1; // WO, SystemConf. Pull-up.
    int8_t p24 = -1; // WO, SystemConf. Pull-up.
    int8_t ean = -1; // WO, SystemConf. Pull-down.

    int8_t p05 = -1; // Configurable.
    int8_t p31 = -1; //
    int8_t p32 = -1; //
    int8_t p33 = -1; //
    int8_t p34 = -1; //
    int8_t p37 = -1; //

    RN4677ModuleMode getModuleMode();

    static const char* getModuleModeString(RN4677ModuleMode);
};



class RN4677 : public RNBase {
  public:
    RN4677(RN4677Pins*);
    virtual ~RN4677();

    /* Overrides from the Transport class. */
    int8_t connect();
    //virtual int8_t disconnect();
    int8_t listen();
    bool   write_port(unsigned char* out, int out_len);
    int8_t read_port();

    void tx_wakeup();
    void rx_wakeup();

    /* Overrides from EventReceiver */
    void printDebug(StringBuilder *);
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    #if defined(MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //MANUVR_CONSOLE_SUPPORT


  protected:
    void factoryReset(void);   // Perform the sequence that will factory-reset the RN.
    void gpioSetup(void);
    void force_9600_mode(bool);   // Call with 'true' to force the module into 9600bps.
    void set_bitrate(int);    //

    int8_t modulePower(bool);
    int8_t moduleSleep(bool);

    /* Set commands */
    //void setDeviceClass(uint16_t);

    //void setServiceName(const char*);
    //void setServiceClass(uint16_t);
    //void setServiceUUID(uint16_t);

    //void setAuthMode(RN4677AuthMode);
    //void setPinCode(const char*);

    //void setStreamAck(bool);
    //void setProtoMode(RN4677ProtoMode);
    //void setConnectionMode(RN4677ConnectMode);
    //void setInquiryScanWindow(uint16_t);
    //void setPageScanWindow(uint16_t);

    //void setStatusDelimiters(const char* leader, const char* trailer);
    //void setStatusDelimiters(const char* leader) {
    //  setStatusDelimiters(leader, "");
    //};

    //void storeMACAddr(bool, const char*);
    //void storeMACAddr(const char*);

    //void setBLEParams(uint16_t, uint16_t, uint16_t);
    //void setUARTSpeed(const char*);

    //void setLowPowerSniff(bool);
    //void setRoleSwitch(bool);
    //void _prompt_enabled(bool);

    //void setTxPower(RN4677TxPower);
    //void setSerializedName(const char*);
    //void setConfDetectChar();

    ///* Get commands */
    //char* getDeviceName();
    //uint16_t getDeviceClass();

    //const char* getServiceName();
    //uint16_t getServiceClass();
    //uint16_t getServiceUUID();

    //RN4677AuthMode getAuthMode();

    //bool getStreamAck();
    //RN4677ProtoMode getProtoMode();
    //RN4677ConnectMode getConnectionMode();
    //uint16_t getInquiryScanWindow();
    //uint16_t getPageScanWindow();

    //int getUARTSpeed();

    //bool getLowPowerSniff();
    //bool getRoleSwitch();
    //bool _prompt_enabled();

    //RN4677TxPower getTxPower();
    //char getConfDetectChar();



  private:
    RN4677Pins _pins;

    // TODO: Migrate into flags member somewhere else.
    bool _module_power = true;
    bool _module_sleep = false;

};


#endif
