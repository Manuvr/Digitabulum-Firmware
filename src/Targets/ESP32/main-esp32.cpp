/*
File:   main.cpp
Author: J. Ian Lindsay
Date:   2017.02.09

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

    .__       ,    .     .
    |  \* _ *-+- _.|_ . .|. .._ _
    |__/|(_]| | (_][_)(_||(_|[ | )
         ._|

Intended target is an WROOM32 SoC module.
*/

#include <Platform/Platform.h>
#include <Platform/Peripherals/I2C/I2CAdapter.h>
#include <Drivers/ADP8866/ADP8866.h>
#include <Drivers/ATECC508/ATECC508.h>
#include <Drivers/PMIC/BQ24155/BQ24155.h>
#include <Drivers/PMIC/LTC294x/LTC294x.h>
#include <XenoSession/Console/ManuvrConsole.h>

#include "Digitabulum/CPLDDriver/CPLDDriver.h"
#include "Digitabulum/ManuLegend/ManuManager.h"
#include "Digitabulum/DigitabulumPMU/DigitabulumPMU-r2.h"

#include "esp_task_wdt.h"

#ifdef __cplusplus
  extern "C" {
#endif

/*
* Pin defs given here assume a WROOM32 module.
*/
const CPLDPins cpld_pins(
  26,  // Reset
  21,  // Transfer request
  255, // IO33 (input-only) CPLD's IRQ_WAKEUP pin
  2,   // CPLD clock input
  25,  // CPLD OE pin
  255, // N/A (CPLD GPIO)
  5,   // SPI1 CS
  17,  // SPI1 CLK
  16,  // SPI1 MOSI
  4,   // SPI1 MISO
  35,  // IO35 (SPI2 CS)
  34,  // IO34 (input-only) (SPI2 CLK)
  32   // IO32 (input-only) (SPI2 MOSI)
);


const I2CAdapterOptions i2c_opts(
  0,   // Device number
  14,  // IO14 (sda)
  12,  // IO12 (scl)
  I2C_ADAPT_OPT_FLAG_SDA_PU
);

const ATECC508Opts atecc_opts(
  (uint8_t) 255
);

const ADP8866Pins adp_opts(
  19,  // IO19 (Reset)
  18   // IO18 (IRQ)
);


const BatteryOpts battery_opts (
  1400,    // Battery capacity (in mAh)
  3.60f,   // Battery dead (in volts)
  3.70f,   // Battery weak (in volts)
  4.15f,   // Battery float (in volts)
  4.2f     // Battery max (in volts)
);

const LTC294xOpts gas_gauge_opts(
  13,     // IO13 (Alert pin)
  LTC294X_OPT_ACD_AUTO | LTC294X_OPT_INTEG_SENSE
);

const BQ24155Opts charger_opts(
  68,  // Sense resistor is 68 mOhm.
  255, // N/A (STAT)
  23,  // IO23 (ISEL)
  BQ24155USBCurrent::LIMIT_800,  // Hardware limits (if any) on source draw..
  BQ24155_FLAG_ISEL_HIGH  // We want to start the ISEL pin high.
);

const PowerPlantOpts powerplant_opts(
  255, // 2.5v select pin is driven by the CPLD.
  27,  // Aux regulator enable pin.
  DIGITAB_PMU_FLAG_ENABLED  // Regulator enabled @3.3v
);

#define ESP32_LED_PIN             15  // This is an LED.

/*
Pins
-------------
IO12
IO13
IO14
IO15
IO16
IO17
IO18
IO19
IO21
IO22
IO23
IO25
IO26
IO27
IO32
IO33
IO34
IO35
*/

#if defined (__BUILD_HAS_FREERTOS)

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "apps/sntp/sntp.h"

#define EXAMPLE_WIFI_SSID "WaddleNest_Quarantine"
#define EXAMPLE_WIFI_PASS "idistrustmydevice"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static const char *TAG = "main-cpp";

static void obtain_time(void);
static void initialize_sntp(void);
static void initialise_wifi(void);
static esp_err_t event_handler(void *ctx, system_event_t *event);


static void obtain_time() {
  ESP_ERROR_CHECK( nvs_flash_init() );
  initialise_wifi();
  xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
  initialize_sntp();

  // wait for time to be set
  time_t now = 0;
  struct tm timeinfo = { 0 };
  int retry = 0;
  const int retry_count = 10;
  while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
    ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    time(&now);
    localtime_r(&now, &timeinfo);
  }
  //ESP_ERROR_CHECK( esp_wifi_stop() );
}


static void initialize_sntp() {
  ESP_LOGI(TAG, "Initializing SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
}


static void initialise_wifi() {
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  wifi_config_t wifi_config = {
    .sta = {
      EXAMPLE_WIFI_SSID,
      EXAMPLE_WIFI_PASS,
      WIFI_FAST_SCAN
    }
  };
  ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK( esp_wifi_start() );
}


static esp_err_t event_handler(void* ctx, system_event_t* event) {
  switch(event->event_id) {
    case SYSTEM_EVENT_SCAN_DONE:
      break;
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      break;
    case SYSTEM_EVENT_STA_STOP:
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      /* This is a workaround as ESP32 WiFi libs don't currently
         auto-reassociate. */
      esp_wifi_connect();
      xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
      break;
    default:
      break;
  }
  return ESP_OK;
}



/*******************************************************************************
* Main function                                                                *
*******************************************************************************/
void app_main() {
  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform.platformPreInit();
  Kernel* kernel = platform.kernel();

  I2CAdapter i2c(&i2c_opts);
  kernel->subscribe(&i2c);

  //BQ24155 charger(&charger_opts);
  //i2c.addSlaveDevice((I2CDeviceWithRegisters*) &charger);

  //LTC294x gas_gauge(&gas_gauge_opts, battery_opts.capacity);
  //i2c.addSlaveDevice((I2CDeviceWithRegisters*) &gas_gauge);

  PMU pmu(&i2c, &charger_opts, &gas_gauge_opts, &powerplant_opts, &battery_opts);
  kernel->subscribe((EventReceiver*) &pmu);

  ADP8866 leds(&adp_opts);
  i2c.addSlaveDevice((I2CDeviceWithRegisters*) &leds);
  kernel->subscribe((EventReceiver*) &leds);

  CPLDDriver _cpld(&cpld_pins);
  kernel->subscribe(&_cpld);

  ManuManager _legend_manager(&_cpld);
  kernel->subscribe(&_legend_manager);

  ATECC508 atec(&atecc_opts);
  i2c.addSlaveDevice((I2CDevice*) &atec);
  kernel->subscribe((EventReceiver*) &atec);

  platform.bootstrap();

  unsigned long ms_0 = millis();
  unsigned long ms_1 = ms_0;
  unsigned long ms_w = ms_0;
  bool odd_even = false;

  gpioDefine(ESP32_LED_PIN, GPIOMode::OUTPUT);

  // TODO: Ultimately generalize this... Taken from ESP32 examples....
  // https://github.com/espressif/esp-idf/blob/master/examples/protocols/sntp/main/sntp_example_main.c
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  // Is time set? If not, tm_year will be (1970 - 1900).
  if (timeinfo.tm_year < (2016 - 1900)) {
    ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
    obtain_time();
    // update 'now' variable with current time
    time(&now);
  }
  char strftime_buf[64];

  // Set timezone to Eastern Standard Time and print local time
  setenv("TZ", "MST7MDT,M3.2.0/2,M11.1.0", 1);
  tzset();
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI(TAG, "The current date/time in CO is: %s", strftime_buf);
  // TODO: End generalize block.


  while (1) {
    kernel->procIdleFlags();
    ms_1 = millis();
    kernel->advanceScheduler(ms_1 - ms_0);
    ms_0 = ms_1;
    setPin(ESP32_LED_PIN, odd_even);
    odd_even = !odd_even;
    if (ms_0 > (ms_w+1000)) {
      ms_w = ms_0;
      //esp_task_wdt_feed();
    }
  }
}
#endif  // __BUILD_HAS_FREERTOS



#ifdef __cplusplus
  }
#endif
