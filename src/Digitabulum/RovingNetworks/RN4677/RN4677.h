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



class RN4677 : public RNBase {
  public:
    RN4677(uint8_t _rst_pin);
    virtual ~RN4677();

    /* Overrides from the Transport class. */


    /* Overrides from EventReceiver */
    void printDebug(StringBuilder *);
    int8_t notify(ManuvrRunnable*);
    int8_t callback_proc(ManuvrRunnable *);
    #if defined(__MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //__MANUVR_CONSOLE_SUPPORT


  protected:
    void factoryReset(void);   // Perform the sequence that will factory-reset the RN.
    void gpioSetup(void);
    void force_9600_mode(bool);   // Call with 'true' to force the module into 9600bps.
    void set_bitrate(int);    //

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
    uint8_t _pin_p04 = 0;
    uint8_t _pin_p15 = 0;
    uint8_t _pin_p05 = 0;
    uint8_t _pin_p20 = 0;

    uint8_t _pin_p31 = 0;
    uint8_t _pin_p32 = 0;
    uint8_t _pin_p33 = 0;
    uint8_t _pin_p34 = 0;
    uint8_t _pin_p37 = 0;
};


#endif
