#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>

/*** DEVICE SELECTION ***/
#define DEVICENAME      "DavisReader"
#define DEVICETYPE      "WeatherStation"
#define WIFI_ENABLED
#define NTP_ENABLED

/*** WIFI/MQTT Settings ***/
#ifdef WIFI_ENABLED  
  #define WIFI_DEBUG

  #include "Settings_Private.h"
  // WiFi Settings
  //#define WIFI_SSID         "wifi_ssid"
  //#define WIFI_PASS         "wifi_pass"
  
  // MQTT Settings
  //#define MQTT_SERVER       "mqtt_ip"
  //#define MQTT_USERNAME     "mqtt_user"
  //#define MQTT_PASSWORD     "mqtt_pass"
  //#define MQTT_PORT         1883
  
  #define MQTT_TOPIC_STATE                      DEVICETYPE "/" DEVICENAME
  #define MQTT_TOPIC_STATUS                     DEVICETYPE "/" DEVICENAME "/status"
  #define MQTT_TOPIC_SET                        DEVICETYPE "/" DEVICENAME "/set"  
  #define MQTT_TOPIC_CONFIG                     DEVICETYPE "/" DEVICENAME "/config"
  
  #define MQTT_TOPIC_CMD_RAW                    DEVICETYPE "/" DEVICENAME "/cmd_raw"  
  #define MQTT_TOPIC_CMD                        DEVICETYPE "/" DEVICENAME "/cmd"  
  #define MQTT_TOPIC_RESP_RAW                   DEVICETYPE "/" DEVICENAME "/resp_raw"  
  #define MQTT_TOPIC_RESP                       DEVICETYPE "/" DEVICENAME "/resp"  
  
  #define MQTT_TOPIC_RAW_LOOP                   DEVICETYPE "/" DEVICENAME "/raw_loop"  
  #define MQTT_TOPIC_RAW_LOOP2                  DEVICETYPE "/" DEVICENAME "/raw_loop2" 

  #define MQTT_TOPIC_ARCHIVE                    DEVICETYPE "/" DEVICENAME "/archive"  

  #define MQTT_CMD_GET_ARCHIVE                  "get_archive"

  #define MQTT_CMD_GET_TIME                     "get_time"
  #define MQTT_CMD_SET_TIME                     "set_time"
  
  //#define MQTT_TOPIC_GROUP                      DEVICETYPE "/" GROUPNAME
  //#define MQTT_HOMEASSISTANT_DISCOVERY_PREFIX   "homeassistant"
  //#define MQTT_HOMEASSISTANT_DISCOVERY
  
  #define MQTT_PAYLOAD_ON                       "ON"
  #define MQTT_PAYLOAD_OFF                      "OFF"
  
  #define MQTT_STATUS_ONLINE                    "online"
  #define MQTT_STATUS_OFFLINE                   "offline"
  
  #define MQTT_MAX_PACKET_SIZE 640
  
  // OTA Settings
  #define OTA_DEVICENAME    DEVICENAME      //change this to whatever you want to call your device
  #define OTA_PASSWORD      "123"           //the password you will need to enter to upload remotely via the ArduinoIDE
  #define OTA_PORT          8266

#endif //WIFI_ENABLED

#define NTP_UPDATE_INTERVAL_MS    (4UL*60*60*1000)

/*** General Settings ***/

typedef struct {
  uint16_t UpdateIntervalSec;
  
} Settings;

extern Settings g_Settings;

typedef struct 
{
  char      FWDate[32];
  char      FWVersion[32];
  uint8_t   Receivers;
  
  float     InsideTemperature;
  uint8_t   InsideHumidity;
  float     OutsideTemperature;
  uint8_t   OutsideHumidity;
  
  float     BarometricPressure;
  int8_t    BarometricTrend;
  float     DewPoint;
  
  float     WindSpeed;      
  float     AvgWindSpeed;
  float     WindChillTemp;
  
  uint16_t  WindDirection;
  
  float     ExtraTemps[7];
  float     SoilTemps[4];
  float     LeafTemps[4];
  uint8_t   ExtraHumidity[7];
  float     RainRate;
  float     Rain15min;
  float     RainHour;
  float     Rain24Hrs;
  float     RainDaily;

  uint8_t   UVindex;
  uint16_t  SolarRadiation;
  
  uint8_t   Battery_Transmitter;
  float     Battery_Console;
  uint8_t   ForecastIcons;
  uint8_t   ForecastRule;
  char      TimeSunrise[8];
  char      TimeSunset[8];
} StationData;

typedef struct 
{
  uint8_t Seconds;
  uint8_t Minutes;
  uint8_t Hours;
  uint8_t Day;
  uint8_t Month;
  uint16_t Year;
} DateTimeStruct;


extern StationData g_StationData;

#define CMD_MAX_SIZE        64
#define CMD_RESP_MAX_SIZE   512

extern char g_CustomCommand[CMD_MAX_SIZE];
extern char g_CustomCommandResponse[CMD_RESP_MAX_SIZE];
extern bool g_CustomCommandPending;

extern bool g_TriggerArchiveDownload;
extern bool g_TriggerArchiveDownloadWithDateTime;

extern bool g_GetTimeCommand;
extern bool g_SetTimeCommand;
extern DateTimeStruct g_DateTimeStruct;

#endif //SETTINGS_H
