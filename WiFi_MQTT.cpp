/*** INCLUDES ***/
#include "WiFi_MQTT.h"

#ifdef WIFI_ENABLED

#include <ArduinoJson.h>
#ifdef ESP8266
  #include <ESP8266WiFi.h>
#endif
#ifdef ESP32
  #include <WiFi.h>
#endif
#ifdef NTP_ENABLED
  #include <NTPClient.h>
  #include <WiFiUdp.h>
#endif //NTP_ENABLED
#include <PubSubClient.h> // Note: MQTT_MAX_PACKET_SIZE was changed to 1024 in this file
#include <ArduinoOTA.h>

/*** DEFINES ***/
#define WIFI_DEBUG

#ifdef WIFI_DEBUG
    #include <SoftwareSerial.h>
    extern SoftwareSerial g_DebugSerial;
    #define MSG_DBG(...)     g_DebugSerial.printf(__VA_ARGS__);g_DebugSerial.println()
    #define MSG_DBG_LN(m)     g_DebugSerial.println(m)
    #define MSG_DBG_NOLINE(m) g_DebugSerial.print(m)
    //#define MSG_DBG(m)     Serial.print(m)
    //#define MSG_DBG_LN(m)  Serial.println(m)
#else
    #define MSG_DBG(m)     
    #define MSG_DBG_LN(m)  
#endif

#define MS_TIMER_START(tim)               tim = millis();
#define MS_TIMER_ELAPSED(tim, delay)      ((millis() - tim) >= delay)

// JSON Settings
const int JSON_BUFFER_SIZE = JSON_OBJECT_SIZE(50);


/*** TYPE DEFINITIONS ***/
typedef enum {
    STATE_WIFI_DISCONNECTED,
    STATE_WIFI_CONNECTING,
    STATE_MQTT_CONNECTING,
    STATE_WIFI_MQTT_CONNECTED    
} WiFi_MQTT_State;


/*** FORWARD DECLARATIONS ***/
static void OTA_Setup(void);
static void MQTT_Callback(char* inTopic, byte* inPayload, unsigned int inLlength);
static bool MQTT_ParseJSON(char* inMessage);
static void MQTT_Reconnect(void);
static void MQTT_SetOnline(bool inOnline);
#ifdef MQTT_HOMEASSISTANT_DISCOVERY
  static void MQTT_Discovery(void);
#endif //MQTT_HOMEASSISTANT_DISCOVERY

/*** PRIVATE VARIABLES ***/
static WiFi_MQTT_State  s_State;

static WiFiClient       s_WiFiClient;
static PubSubClient     s_MQTTClient(s_WiFiClient);

#ifdef NTP_ENABLED
  static WiFiUDP s_NTP_UDP;
  static NTPClient s_NTP_Client(s_NTP_UDP);
#endif //NTP_ENABLED

static unsigned long s_Timer;

/*** PUBLIC FUNCTIONS ***/
void WiFi_MQTT_Init()
{
    // Register MQTT Server and callback function
    s_MQTTClient.setServer(MQTT_SERVER, MQTT_PORT);
    s_MQTTClient.setCallback(MQTT_Callback);      
    s_State = STATE_WIFI_DISCONNECTED;
}


void WiFi_MQTT_Tick()
{
    if ((s_State != STATE_WIFI_DISCONNECTED) && (s_State != STATE_WIFI_CONNECTING) && (WiFi.status() != WL_CONNECTED))
    {
        MSG_DBG("WIFI Disconnected! Attempting reconnection.");
        s_State = STATE_WIFI_DISCONNECTED;
    }
    
    switch(s_State)
    {
        case STATE_WIFI_DISCONNECTED:
            // We start by connecting to a WiFi network
            MSG_DBG("Connecting to ");
            MSG_DBG_LN(WIFI_SSID);

            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            s_State = STATE_WIFI_CONNECTING;
            break;
        case STATE_WIFI_CONNECTING:
            if (WiFi.status() == WL_CONNECTED)
            {
                MSG_DBG_NOLINE("WiFi connected. IP address: ");
                MSG_DBG_LN(WiFi.localIP());
                OTA_Setup();
                s_Timer = 0;    // no delay before initial connect attempt
                s_State = STATE_MQTT_CONNECTING;

                #ifdef NTP_ENABLED
                  s_NTP_Client.setUpdateInterval(NTP_UPDATE_INTERVAL_MS);
                  s_NTP_Client.begin();  
                #endif //NTP_ENABLED
            }
            break;
        case STATE_MQTT_CONNECTING:
            if ((s_Timer == 0) || MS_TIMER_ELAPSED(s_Timer, 5000))
            {
                // Attempt to connect
                MSG_DBG("Attempting MQTT connection...");
                if (s_MQTTClient.connect(OTA_DEVICENAME, MQTT_USERNAME, MQTT_PASSWORD, MQTT_TOPIC_STATUS, 0, true, MQTT_STATUS_OFFLINE)) 
                {
                  MSG_DBG_LN("connected!");

                  // Subscribe to topics
                  MSG_DBG("Subscribe to topics:");
                  MSG_DBG_LN(MQTT_TOPIC_SET);
                  s_MQTTClient.subscribe(MQTT_TOPIC_SET);
                                    
                  #ifdef MQTT_TOPIC_GROUP
                    MSG_DBG_LN(MQTT_TOPIC_GROUP);
                    s_MQTTClient.subscribe(MQTT_TOPIC_GROUP);
                  #endif //MQTT_TOPIC_GROUP

                  #ifdef MQTT_TOPIC_CMD
                    MSG_DBG_LN(MQTT_TOPIC_CMD);
                    s_MQTTClient.subscribe(MQTT_TOPIC_CMD);
                  #endif //MQTT_TOPIC_CMD
                  
                  #ifdef MQTT_TOPIC_CMD_RAW
                    MSG_DBG_LN(MQTT_TOPIC_CMD_RAW);
                    s_MQTTClient.subscribe(MQTT_TOPIC_CMD_RAW);
                  #endif //MQTT_TOPIC_CMD_RAW
                  
                  MQTT_SetOnline(true);
                  
                  #ifdef MQTT_HOMEASSISTANT_DISCOVERY
                    delay(10);
                    MQTT_Discovery();
                  #endif //MQTT_HOMEASSISTANT_DISCOVERY
                  delay(10);
                  MQTT_SendConfig();
                  delay(10);
                  MQTT_SendState();
                  s_State = STATE_WIFI_MQTT_CONNECTED;
                }
                else 
                {
                  MSG_DBG("failed, rc=%d  try again in 5 seconds", s_MQTTClient.state());
                  // Wait 5 seconds before retrying
                  MS_TIMER_START(s_Timer);
                }
            }
            break;
        case STATE_WIFI_MQTT_CONNECTED:
            if (!s_MQTTClient.loop()) //s_MQTTClient.connected())
            {
                MSG_DBG("MQTT Connection Lost!");
                s_Timer = 1000;    // 1sec delay before reconnect attempt
                s_State = STATE_MQTT_CONNECTING;
            }
            else
            {
                // Service MQTT messages
                
            }
            break;
    }
    
    if (WiFi.status() == WL_CONNECTED)
    {
        // Handle Over-The-Air (OTA) update requests
        ArduinoOTA.handle();
        #ifdef NTP_ENABLED
          s_NTP_Client.update();  
        #endif //NTP_ENABLED
    }
}
#ifdef NTP_ENABLED
const char * WiFi_MQTT_GetTime(void)
{
  return s_NTP_Client.getFormattedTime().c_str();
}
#endif //NTP_ENABLED


void MQTT_SendConfig() 
{
  if (s_MQTTClient.connected())
  {
    StaticJsonBuffer<JSON_BUFFER_SIZE> lvJSONBuffer;
  
    JsonObject& lvRoot = lvJSONBuffer.createObject();
    lvRoot["Name"]                = OTA_DEVICENAME;
    lvRoot["IP"]                  = WiFi.localIP().toString();
    //lvRoot["NumLeds"]             = DEFAULT_NUM_LEDS;
    lvRoot["Davis FW Date"]       = g_StationData.FWDate;
    lvRoot["Davis FW Version"]    = g_StationData.FWVersion;
    lvRoot["UpdateIntervalSec"]   = g_Settings.UpdateIntervalSec;
    
    char lvBuffer[lvRoot.measureLength() + 1];
    lvRoot.printTo(lvBuffer, sizeof(lvBuffer));
  
    MSG_DBG("JSON Config: %s\n", lvBuffer);
  
    MSG_DBG("Publish to topic: %s\n", MQTT_TOPIC_CONFIG);
  
    s_MQTTClient.publish(MQTT_TOPIC_CONFIG, lvBuffer, true);
  }
}

void MQTT_SendState() 
{
  if (s_MQTTClient.connected())
  {
    StaticJsonBuffer<JSON_BUFFER_SIZE> lvJSONBuffer;
  
    JsonObject& lvRoot = lvJSONBuffer.createObject();

#ifdef NTP_ENABLED
    lvRoot["Time"]                = s_NTP_Client.getEpochTime();//WiFi_MQTT_GetTime();
#endif //NTP_ENABLED 

    lvRoot["InsideTemperature"]   = g_StationData.InsideTemperature;
    lvRoot["InsideHumidity"]      = g_StationData.InsideHumidity;
    lvRoot["OutsideTemperature"]  = g_StationData.OutsideTemperature;
    lvRoot["OutsideHumidity"]     = g_StationData.OutsideHumidity;
    
    lvRoot["BarPressure"]         = g_StationData.BarometricPressure;
    lvRoot["BarTrend"]            = g_StationData.BarometricTrend;
    lvRoot["DewPoint"]            = g_StationData.DewPoint;
    
//    lvRoot["WindSpeed"]           = g_StationData.WindSpeed;
//    lvRoot["AvgWindSpeed"]        = g_StationData.AvgWindSpeed;
//    lvRoot["WindDirection"]       = g_StationData.WindDirection;
//    lvRoot["WindChill"]           = g_StationData.WindChillTemp;
/*
    JsonArray& lvExtraTemperatures = lvRoot.createNestedArray("ExtraTemps");
    for (int i = 0; i < 7; i++)
    {
      lvExtraTemperatures.add(g_StationData.ExtraTemps[i]);
    }
    JsonArray& lvExtraHumidity = lvRoot.createNestedArray("ExtraHumidity");
    for (int i = 0; i < 7; i++)
    {
      lvExtraHumidity.add(g_StationData.ExtraHumidity[i]);
    }
    
    JsonArray& lvSoilTemps = lvRoot.createNestedArray("SoilTemps");
    for (int i = 0; i < 4; i++)
    {
      lvSoilTemps.add(g_StationData.SoilTemps[i]);
    }

    JsonArray& lvLeafTemps = lvRoot.createNestedArray("LeafTemps");
    for (int i = 0; i < 4; i++)
    {
      lvLeafTemps.add(g_StationData.LeafTemps[i]);
    }
*/
    lvRoot["RainRate"]            = g_StationData.RainRate;
//    lvRoot["Rain15min"]           = g_StationData.Rain15min;
//    lvRoot["RainHour"]            = g_StationData.RainHour;
    lvRoot["Rain24Hrs"]           = g_StationData.Rain24Hrs;
    lvRoot["RainDaily"]           = g_StationData.RainDaily;
  
//    lvRoot["UVindex"]             = g_StationData.UVindex;
//    lvRoot["SolarRadiation"]      = g_StationData.SolarRadiation;
//    lvRoot["Battery_Transmitter"] = g_StationData.Battery_Transmitter;
//    lvRoot["Battery_Console"]     = g_StationData.Battery_Console;
    lvRoot["ForecastIcons"]       = g_StationData.ForecastIcons;
//    lvRoot["ForecastRule"]        = g_StationData.ForecastRule;
//    lvRoot["TimeSunrise"]         = g_StationData.TimeSunrise;
//    lvRoot["TimeSunset"]          = g_StationData.TimeSunset;
  
    char lvBuffer[lvRoot.measureLength() + 1];
    lvRoot.printTo(lvBuffer, sizeof(lvBuffer));
  
    MSG_DBG("JSON Status: %s", lvBuffer);
  
    MSG_DBG("Publish to topic: %s", MQTT_TOPIC_STATE);
  
    s_MQTTClient.publish(MQTT_TOPIC_STATE, lvBuffer, true);
  }
}

void MQTT_SendRaw(const char* inTopic, uint8_t *inData, uint16_t inLength)
{
  if (s_MQTTClient.connected())
  {
    //char lvTopic[128];
    //snprintf(lvTopic, sizeof(lvTopic), PSTR("%s/%s"), MQTT_TOPIC_STATE, inSubTopic);
    MSG_DBG("Sending %d raw bytes to topic: %s",inLength,inTopic);
    s_MQTTClient.publish(inTopic, inData, inLength);
  }
}

/*** PRIVATE FUNCTIONS ***/
static void OTA_Setup(void)
{
   //OTA SETUP
  ArduinoOTA.setPort(OTA_PORT);
  
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTA_DEVICENAME);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)OTApassword);

  ArduinoOTA.onStart([]() 
  {
    MSG_DBG_LN("OTA Starting");
  });
  ArduinoOTA.onEnd([]() 
  {
    MSG_DBG_LN("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
  {
    MSG_DBG("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) 
  {
    MSG_DBG("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

static void MQTT_Callback(char* inTopic, byte* inPayload, unsigned int inLength)
{
  MSG_DBG("Message arrived [%s]", inTopic);
  
  // copy message in order to append '\0' terminator
  char lvMessage[inLength + 1];
  for (int i = 0; i < inLength; i++) 
  {
    lvMessage[i] = (char)inPayload[i];
  }
  lvMessage[inLength] = '\0';
  MSG_DBG("%s", lvMessage);

  if (strcmp(inTopic, MQTT_TOPIC_SET) == 0)
  {    
    if (!MQTT_ParseJSON(lvMessage)) 
    {
      return;
    }
    MQTT_SendConfig();  
  }
#ifdef MQTT_TOPIC_CMD_RAW 
  else if (strcmp(inTopic, MQTT_TOPIC_CMD_RAW) == 0)
  {
    if (inLength > (CMD_MAX_SIZE-1))
    {
      inLength = (CMD_MAX_SIZE-1);
    }
    memcpy(g_CustomCommand, inPayload, inLength);
    g_CustomCommand[inLength] = '\0';
    g_CustomCommandPending = true;
  }
#endif //MQTT_TOPIC_CMD_RAW
#ifdef MQTT_TOPIC_CMD  
  else if (strcmp(inTopic, MQTT_TOPIC_CMD) == 0)
  {
    if (memcmp(inPayload, MQTT_CMD_GET_ARCHIVE, strlen(MQTT_CMD_GET_ARCHIVE)) == 0)
    {
      char lvTempString[inLength+1];
      memcpy(lvTempString, inPayload, inLength);
      lvTempString[inLength] = '\0';
      int lvArgs[6];
      if (sscanf(lvTempString, MQTT_CMD_GET_ARCHIVE" %d-%d-%d %d:%d:%d", &lvArgs[0], &lvArgs[1], &lvArgs[2], &lvArgs[3], &lvArgs[4], &lvArgs[5]) == 6)
      {
        g_DateTimeStruct.Year = (uint16_t)lvArgs[0];
        g_DateTimeStruct.Month = (uint8_t)lvArgs[1];
        g_DateTimeStruct.Day = (uint8_t)lvArgs[2];
        g_DateTimeStruct.Hours = (uint8_t)lvArgs[3];
        g_DateTimeStruct.Minutes = (uint8_t)lvArgs[4];
        g_DateTimeStruct.Seconds = (uint8_t)lvArgs[5];
        // valid time stamp given command
        g_TriggerArchiveDownloadWithDateTime = true;
      }
      else
      {
         g_TriggerArchiveDownload = true;
      }      
    }
    else if (memcmp(inPayload, MQTT_CMD_GET_TIME, strlen(MQTT_CMD_GET_TIME)) == 0)
    {
      // get time command
      g_GetTimeCommand = true;
    }
    else if (memcmp(inPayload, MQTT_CMD_SET_TIME, strlen(MQTT_CMD_SET_TIME)) == 0)
    {
      char lvTempString[inLength+1];
      memcpy(lvTempString, inPayload, inLength);
      lvTempString[inLength] = '\0';
      int lvArgs[6];
      if (sscanf(lvTempString, MQTT_CMD_SET_TIME" %d-%d-%d %d:%d:%d", &lvArgs[0], &lvArgs[1], &lvArgs[2], &lvArgs[3], &lvArgs[4], &lvArgs[5]) == 6)
      {
        g_DateTimeStruct.Year = (uint16_t)lvArgs[0];
        g_DateTimeStruct.Month = (uint8_t)lvArgs[1];
        g_DateTimeStruct.Day = (uint8_t)lvArgs[2];
        g_DateTimeStruct.Hours = (uint8_t)lvArgs[3];
        g_DateTimeStruct.Minutes = (uint8_t)lvArgs[4];
        g_DateTimeStruct.Seconds = (uint8_t)lvArgs[5];
        // valid set time command
        g_SetTimeCommand = true;
      }
      else
      {
        MSG_DBG("Invalid "MQTT_CMD_SET_TIME" command!");
      }
    }
  }  
#endif //MQTT_TOPIC_CMD
}

static bool MQTT_ParseJSON(char* inMessage) 
{
  StaticJsonBuffer<JSON_BUFFER_SIZE> lvJSONBuffer;

  JsonObject& lvRoot = lvJSONBuffer.parseObject(inMessage);

  if (!lvRoot.success()) 
  {
    MSG_DBG_LN("MQTT_ParseJSON: parseObject() failed");
    return false;
  }
  if (lvRoot.containsKey("UpdateIntervalSec") && lvRoot.is<unsigned short>("UpdateIntervalSec"))
  {
    g_Settings.UpdateIntervalSec = lvRoot.get<unsigned short>("UpdateIntervalSec");
    MSG_DBG("UpdateIntervalSec: ");
    MSG_DBG_LN(g_Settings.UpdateIntervalSec);
  }
  return true;
}

static void MQTT_SetOnline(bool inOnline)
{
  if (inOnline)
  {
     s_MQTTClient.publish(MQTT_TOPIC_STATUS, MQTT_STATUS_ONLINE, true);
  }
  else
  {
    s_MQTTClient.publish(MQTT_TOPIC_STATUS, MQTT_STATUS_OFFLINE, true);
  }
}

#ifdef MQTT_HOMEASSISTANT_DISCOVERY
static void MQTT_Discovery()
{
  StaticJsonBuffer<JSON_BUFFER_SIZE> lvJSONBuffer;
  JsonObject& lvRoot = lvJSONBuffer.createObject();

  lvRoot["name"] = DEVICENAME;
  //lvRoot["platform"] = "mqtt";
  lvRoot["schema"] = "json";
  //lvRoot["rgb"] = true;
  lvRoot["hs"] = true;
  //lvRoot["color_temp"] = true;
  lvRoot["brightness"] = true;
  //lvRoot["white_value"] = true;
  lvRoot["state_topic"] = MQTT_TOPIC_STATE;
  lvRoot["command_topic"] = MQTT_TOPIC_SET;
  lvRoot["availability_topic"] = MQTT_TOPIC_STATUS;
  lvRoot["effect"] = true;
  
  JsonArray& lvEffects = lvRoot.createNestedArray("effect_list");
  for (int i = 0; i < g_NumPrograms; i++)
  {
    lvEffects.add(g_LEDPrograms[i]->Name);
  }

  char lvBuffer[lvRoot.measureLength() + 1];
  lvRoot.printTo(lvBuffer, sizeof(lvBuffer));
  
  char lvDiscoveryTopic[128];
  snprintf(lvDiscoveryTopic, sizeof(lvDiscoveryTopic), PSTR("%s/light/%s/config"), MQTT_HOMEASSISTANT_DISCOVERY_PREFIX, DEVICENAME);

  s_MQTTClient.publish(lvDiscoveryTopic, lvBuffer, true);
}
#endif //MQTT_HOMEASSISTANT_DISCOVERY

static void MQTT_Reconnect() 
{
  // Loop until we're reconnected
  while (!s_MQTTClient.connected()) 
  {
    MSG_DBG("Attempting MQTT connection...");
    // Attempt to connect
    if (s_MQTTClient.connect(OTA_DEVICENAME, MQTT_USERNAME, MQTT_PASSWORD)) 
    {
      MSG_DBG("Connected!");

      // Subscribe to topics
      s_MQTTClient.subscribe(MQTT_TOPIC_SET);
      #ifdef MQTT_TOPIC_GROUP
        s_MQTTClient.subscribe(MQTT_TOPIC_GROUP);
      #endif //MQTT_TOPIC_GROUP
      MQTT_SendState();
    } 
    else 
    {
      MSG_DBG("failed, rc=%d try again in 5 seconds", s_MQTTClient.state());
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


#endif // WIFI_ENABLED
