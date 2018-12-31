
/*** INCLUDES ***/
#include <SoftwareSerial.h>
#include "WiFi_MQTT.h"

/*** DEFINES***/
#define DAVIS_WRITE(a)             Serial.print(a)
#define DAVIS_DATA_AVAILABLE(a)    Serial.available()

#define MSG_DBG(...)               g_DebugSerial.printf(__VA_ARGS__);g_DebugSerial.println();
#define MSG_DBG_NO_LINE(...)       g_DebugSerial.printf(__VA_ARGS__);
//#define DEBUG_LOW_LEVEL

#define ACK         0x06
#define NACK        0x21
#define NACK2       0x15
#define CANCEL      0x18
#define ESC         0x1B

#define RESP_OK_STR     "\n\rOK\n\r"
#define RESP_DONE_STR   "DONE\n\r"

#define DAVIS_WAKEUP_TIMEOUT_MS        1200
#define DAVIS_COMMAND_TIMEOUT_MS        500
#define DAVIS_BYTE_TIMEOUT_MS           50
#define DAVIS_LOOP_COMMAND_TIMEOUT_MS  3000

// Forecast Icons
#define FORECAST_ICON_RAIN        0x01
#define FORECAST_ICON_CLOUD       0x02
#define FORECAST_ICON_PART_CLOUD  0x04
#define FORECAST_ICON_SUN         0x08
#define FORECAST_ICON_SNOW        0x10

// Data conversion macros
#define DAVIS_CONVERT_TEMPERATURE_10TH(raw)      (((((float)raw / 10.0f) - 32.0f) * 5.0f) / 9.0f)
#define DAVIS_CONVERT_EXTRA_TEMPERATURE(raw)     (((((float)raw - 90.0f) - 32.0f) * 5.0f) / 9.0f)
#define DAVIS_CONVERT_WINDSPEED(mph)             ((float)mph * 1.609344f)
#define DAVIS_CONVERT_RAINRATE(clicks)           ((float)clicks * 0.2f)
#define DAVIS_CONVERT_BATT_VOLTAGE(raw)          ((((float)raw * 300.0f)/512.0f)/100.0)
#define DAVIS_CONVERT_BAR_PRESSURE(raw)          ((((float)raw * 33.86389f) / 1000.0))
#define DAVIS_CONVERT_DEW_POINT(raw)             ((((float)raw - 32.0f) * 5.0f) / 9.0f)


#define DATE_TO_DATESTAMP(d,m,y)      (uint16_t)((uint16_t)d + m*32 + (y-2000)*512)
#define TIME_TO_TIMESTAMP(h,m)        (uint16_t)((uint16_t)h*100 + m)

#define DATESTAMP_DAY(ds)             (uint8_t)(ds & 0x1F)
#define DATESTAMP_MONTH(ds)           (uint8_t)((ds >> 5) & 0x0F)
#define DATESTAMP_YEAR(ds)            (uint16_t)(((ds >> 9) & 0x7F) + 2000)
#define TIMESTAMP_HOUR(ts)            (uint8_t)(ts/100)
#define TIMESTAMP_MIN(ts)             (uint8_t)(ts - (100*(ts/100)))

#define DUMP_BYTES(buf, cnt, dbg_type)    \
    if (dbg_type & DEBUG_HEX) { \
      for (int i = 0; i < cnt; i++) { \
        MSG_DBG_NO_LINE("%02X ", buf[i]); \
      } \
    } \
    if (dbg_type & DEBUG_ASCII) { \
      if (dbg_type & DEBUG_HEX) { \
        MSG_DBG_NO_LINE("- "); \
      } \
      for (int i = 0; i < cnt; i++) { \
        if ((buf[i] != '\n') && (buf[i] != '\r')) { \
          MSG_DBG_NO_LINE("%c", (char)buf[i]); \
        } \
      } \
    }
#define PRINT_RESPONSE_TYPE(r) \
  (r == RESP_INVALID ? "INVALID" : \
  (r == RESP_OK ? "OK" : \
  (r == RESP_ACK ? "ACK" : \
  (r == RESP_NACK ? "NACK" : \
  (r == RESP_TIMEOUT ? "TIMEOUT" : "?")))))

#define IS_GOOD_RESPONSE(r) ((r == RESP_OK) || (r == RESP_ACK))

/*** TYPE DEFINITIONS ***/
typedef enum {
  DEBUG_NONE = 0,
  DEBUG_HEX = 1,
  DEBUG_ASCII = 2,
  DEBUG_HEXASCII = 3
} DebugType;

typedef enum {
  RESP_INVALID = 0,
  RESP_OK,
  RESP_ACK,
  RESP_NACK,
  RESP_TIMEOUT
} DavisCommandResponse;

#pragma pack(push)
#pragma pack(1)
typedef struct 
{
  uint8_t Seconds;
  uint8_t Minutes;
  uint8_t Hours;
  uint8_t Day;
  uint8_t Month;
  uint8_t Year;
  uint16_t CRC;
} TimePacket __attribute__((packed));

typedef struct 
{
  uint8_t   Identifier[3];  // 0
  uint8_t   BarTrend;       // 3
  uint8_t   PacketType;     // 4
  uint16_t  NextRecord;     // 5
  int16_t   Barometer;      // 7
  int16_t   InTemperature;  // 9
  uint8_t   InHumidity;     // 11
  int16_t   OutTemperature; // 12
  uint8_t   WindSpeed;      // 14
  uint8_t   AvgWindSpeed;   // 15
  uint16_t  WindDirection;  // 16
  uint8_t   ExtraTemps[7];  // 18
  uint8_t   SoilTemps[4];   // 25
  uint8_t   LeafTemps[4];   // 29
  uint8_t   OutHumidity;    // 33
  uint8_t   ExtraHumidity[7];// 34
  uint16_t  RainRate;       // 41
  uint8_t   UVindex;        // 42
  uint16_t  SolarRadiation; // 44
  uint16_t  StormRain;      // 46
  uint16_t  StormDate;      // 48
  uint16_t  RainDay;        // 50
  uint16_t  RainMonth;      // 52
  uint16_t  RainYear;       // 54
  uint16_t  ET_Day;         // 56
  uint16_t  ET_Month;       // 58
  uint16_t  ET_Year;        // 60
  uint8_t   SoilMoisture[4];// 62
  uint8_t   LeafWetness[4]; // 66
  uint8_t   Alarms_Inside;  // 70
  uint8_t   Alarms_Rain;            // 71
  uint8_t   Alarms_Outside[2];      // 72
  uint8_t   Alarms_ExtraTempHum[8]; // 74
  uint8_t   Alarms_SoilLeaf[4];     // 82
  uint8_t   Battery_Transmitter;    // 86
  uint16_t  Battery_Console;        // 87
  uint8_t   ForecastIcons;          // 89
  uint8_t   ForecastRule;           // 90
  uint16_t  TimeSunrise;            // 91
  uint16_t  TimeSunset;             // 93
  uint8_t   LF;                     // 95
  uint8_t   CR;                     // 96
  uint16_t  CRC;                    // 97
} LoopPacket __attribute__((packed));

typedef struct 
{
  uint8_t   Identifier[3];  // 0
  uint8_t   BarTrend;       // 3
  uint8_t   PacketType;     // 4
  uint16_t  Unused;         // 5
  int16_t   Barometer;      // 7
  int16_t   InTemperature;  // 9
  uint8_t   InHumidity;     // 11
  int16_t   OutTemperature; // 12
  uint8_t   WindSpeed;      // 14
  uint8_t   Unused2;        // 15
  uint16_t  WindDirection;  // 16
  uint16_t  AvgWindSpeed10; // 18
  uint16_t  AvgWindSpeed2;  // 20
  uint16_t  AvgWindGust;    // 22
  uint16_t  WindGustDirection;   // 24
  uint8_t   Unused3[4];     // 26
  int16_t   DewPoint;       // 30
  uint8_t   Unused4;        // 32
  uint8_t   OutHumidity;    // 33
  uint8_t   Unused5;        // 34
  int16_t   HeatIndex;      // 35
  int16_t   WindChill;      // 37
  int16_t   THSWIndex;      // 39
  uint16_t  RainRate;       // 41
  uint8_t   UVindex;        // 42
  uint16_t  SolarRadiation; // 44
  uint16_t  StormRain;      // 46
  uint16_t  StormDate;      // 48
  uint16_t  RainDay;        // 50
  uint16_t  Rain15Min;      // 52
  uint16_t  RainHour;       // 54
  uint16_t  ET_Day;         // 56
  uint16_t  Rain24Hrs;      // 58
  uint8_t   BarReduction;   // 60
  uint16_t  BarOffset;      // 61
  uint16_t  BarCalibNr;     // 63
  uint16_t  BarRaw;         // 65
  uint16_t  BarAbs;         // 67
  uint16_t  Altimeter;      // 69
  uint8_t   Unused6[2];     // 71
  uint8_t   Next10minWindGPtr;    // 73
  uint8_t   Next15minWindGPtr;    // 74
  uint8_t   NextHourlyWindGPtr;   // 75
  uint8_t   NextDailyWindGPtr;    // 76
  uint8_t   NextMinuteRainGPtr;   // 77
  uint8_t   NextStormRainGPtr;    // 78
  uint8_t   MinuteIndex;          // 79
  uint8_t   NextMonthlyRain;      // 80
  uint8_t   NextYearlyRain;       // 81
  uint8_t   NextSeasonalRain;     // 82
  uint8_t   Unused7[2*6];         // 83
  uint8_t   LF;                     // 95
  uint8_t   CR;                     // 96
  uint16_t  CRC;                    // 97
} Loop2Packet __attribute__((packed));

typedef struct 
{
  uint16_t  DateStamp;      // 0
  uint16_t  TimeStamp;      // 2
  int16_t   OutTemperature; // 4
  int16_t   OutTempHigh;    // 6
  int16_t   OutTempLow;     // 8
  uint16_t  RainFall;       // 10
  uint16_t  HighRainRate;   // 12
  uint16_t  Barometer;      // 14
  uint16_t  SolarRadiation; // 16
  uint16_t  WindSamples;    // 18
  int16_t   InTemperature;  // 20
  uint8_t   InHumidity;     // 22
  uint8_t   OutHumidity;    // 23
  uint8_t   AvgWindSpeed;   // 24
  uint8_t   HighWindSpeed;  // 25
  uint8_t   HighWindDirection;  // 26
  uint8_t   PrevWindDirection;  // 27
  uint8_t   AvgUVIndex;     // 28
  uint8_t   ET;             // 29
  uint16_t  HighSolarRadiation; // 30
  uint8_t   HighUVIndex;    // 32
  uint8_t   ForecastRule;   // 33
  uint8_t   LeafTemp[2];    // 34
  uint8_t   LeafWetness[2]; // 36
  uint8_t   SoilTemp[4];    // 38
  uint8_t   RecordType;     // 42
  uint8_t   ExtraHum[2];    // 43
  uint8_t   ExtraTemp[3];   // 45
  uint8_t   SoilMoisture[4];// 48
} ArchiveRecordRevB __attribute__((packed));

typedef struct 
{
  uint8_t           SeqNr;      // 0
  ArchiveRecordRevB Record[5];  // 1
  uint8_t           Unused[4];  // 261
  uint16_t          CRC;        // 265
} ArchivePage __attribute__((packed));

#pragma pack(pop)
/*** GLOBAL VARIABLES ***/
SoftwareSerial g_DebugSerial(3, 1); // RX, TX

Settings g_Settings = {
  .UpdateIntervalSec = 60
};

StationData g_StationData;

char g_CustomCommand[CMD_MAX_SIZE];
char g_CustomCommandResponse[CMD_RESP_MAX_SIZE];
bool g_CustomCommandPending = false;
bool g_TriggerArchiveDownload = false;
bool g_TriggerArchiveDownloadWithDateTime = false;

bool g_GetTimeCommand;
bool g_SetTimeCommand;
DateTimeStruct g_DateTimeStruct;

/*** PRIVATE VARIABLES ***/
static unsigned long s_PrevTimeMs;
static bool s_InitOk = false;

static struct {
  uint8_t Value;
  const char *ForeCastString;
} s_ForeCasts[] = {
  {0x08 ,"Sun Mostly Clear"},
  {0x06 ,"Partly Cloudy"},
  {0x02 ,"Mostly Cloudy"},
  {0x03 ,"Mostly Cloudy, Rain within 12 hours"},
  {0x12 ,"Mostly Cloudy, Snow within 12 hours"},
  {0x13 ,"Mostly Cloudy, Rain or Snow within 12 hours"},
  {0x07 ,"Partly Cloudy, Rain within 12 hours"},
  {0x16 ,"Partly Cloudy, Snow within 12 hours"},
  {0x17 ,"Partly Cloudy, Rain or Snow within 12 hours"}
};

// CRC table
static const uint16_t s_CRC_TABLE [] = {
  0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0xc60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0xe70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0xa1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x2b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0xed1, 0x1ef0,
  };

static uint16_t s_ArchivePageNr = 0;
static uint16_t s_ArchivePageCount = 0;
static uint16_t s_ArchiveRecordStart = 0;
static uint8_t s_ArchivePageBuf[sizeof(ArchivePage)];

/*** PRIVATE FUNCTIONS ***/
uint16_t CalcCrc(uint8_t * inDataPtr, uint16_t inSize)
{
  uint16_t lvCRC = 0;
  for (int i = 0; i < inSize; i++)
  {
    lvCRC = s_CRC_TABLE[(lvCRC >> 8) ^ inDataPtr[i]] ^ (lvCRC << 8);
  }
  return lvCRC;
}

static bool Davis_Init(void);
static bool Davis_SetTime(uint16_t inYear, uint8_t inMonth, uint8_t inDay, uint8_t inHours, uint8_t inMinutes, uint8_t inSeconds);
static bool Davis_GetTime(uint16_t *outYear=0, uint8_t *outMonth=0, uint8_t *outDay=0, uint8_t *outHours=0, uint8_t *outMinutes=0, uint8_t *outSeconds=0);
static bool Davis_SetTime(DateTimeStruct *inDateTimeStruct);
static bool Davis_GetTime(DateTimeStruct *outDateTimeStruct);

  
DavisCommandResponse Davis_ReadCommandResponse(uint16_t inTimeoutMs, DebugType inDebug = DEBUG_HEX);
static void Davis_FlushRx(DebugType inDebug = DEBUG_HEX);
static void Davis_Write(const uint8_t *inBuf, uint16_t inSize, DebugType inDebug = DEBUG_HEX);
static void Davis_Write(const char *inBuf, DebugType inDebug = DEBUG_HEXASCII);
static void Davis_Write(uint8_t inByte, DebugType inDebug = DEBUG_HEXASCII);
static uint16_t Davis_Read(uint8_t *outBuf, uint16_t inBufSize, bool inStopOnNewLine, uint16_t inByteTimeoutMs = DAVIS_BYTE_TIMEOUT_MS, DebugType inDebug = DEBUG_HEX);
static bool Davis_ReadByte(uint8_t *outByte, DebugType inDebug = DEBUG_HEX);

static bool Davis_SendCommand(const char *inCommand, char *outResponse, uint16_t inMaxResponseLength, bool inBinaryResponse=false, uint16_t inTimeoutMs = DAVIS_COMMAND_TIMEOUT_MS);
static uint16_t Davis_SendRawCommand(const char *inCommand, char *outResponse, uint16_t inMaxResponseLength, uint16_t inByteTimeoutMs = DAVIS_BYTE_TIMEOUT_MS);
static bool Davis_WakeUp(void);
static bool Davis_ConvertLoopData(LoopPacket* inLoopPacket, StationData * outStationData);
static bool Davis_ConvertLoop2Data(Loop2Packet* inLoopPacket, StationData * outStationData);

static bool Davis_StartReadArchiveData(uint16_t *outPageCount, uint16_t *outFirstRecord, uint16_t inYear = 2000, uint8_t inMonth = 1, uint8_t inDay = 1, uint8_t inHour = 0, uint8_t inMinute = 0);
static bool Davis_StartReadArchiveData(uint16_t *outPageCount, uint16_t *outFirstRecord, uint16_t inDateStamp, uint16_t inTimeStamp);
static void Davis_StoptReadArchiveData();
static bool Davis_ContinueReadArchiveData(ArchivePage **outArchivePage, uint16_t *outPageNr, uint8_t inRetries);

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(19200);
  Serial.swap();

  pinMode(1, OUTPUT);

  // D1 is the VCC for the RS232 transceiver
  pinMode(D1, OUTPUT);
  digitalWrite(D1, HIGH);   // turn the RS232 transceiver on
  delay(1000);
  
   // set the data rate for the SoftwareSerial port
  g_DebugSerial.begin(19200);
  
  s_PrevTimeMs = millis();

#ifdef WIFI_ENABLED
  WiFi_MQTT_Init();
#endif //WIFI_ENABLED

 // MSG_DBG("sizeof(LoopPacket): %d", sizeof(LoopPacket));
 // MSG_DBG("sizeof(Loop2Packet): %d", sizeof(Loop2Packet));
 // MSG_DBG("sizeof(s_CRC_TABLE): %d", sizeof(s_CRC_TABLE));
 // MSG_DBG("sizeof(ArchiveRecordRevB): %d", sizeof(ArchiveRecordRevB));
 // MSG_DBG("sizeof(ArchivePage): %d", sizeof(ArchivePage));
 
  
}

typedef enum {
  STATE_INIT = 0,
  STATE_IDLE,
  STATE_GET_ARCHIVE_DATA,
} State;
static State s_State = STATE_INIT;

void loop() 
{
  static unsigned long s_LastUpdateTime = 0;
  static bool s_Once = false;
#ifdef WIFI_ENABLED
  WiFi_MQTT_Tick();
#endif //WIFI_ENABLED

  switch (s_State)
  {
    case STATE_INIT:
      if (Davis_Init())
      {
        MSG_DBG("Davis Init OK!");
        s_InitOk = true;
        MQTT_SendConfig();
        s_State = STATE_IDLE;
      }
      else
      {
        delay(2000);
      }
      break;
    case STATE_IDLE:
      if (g_CustomCommandPending)
      {
        if (Davis_WakeUp())
        {
          MSG_DBG("Sending custom command: %s", g_CustomCommand);
          uint16_t lvResponseSize = Davis_SendRawCommand(g_CustomCommand, g_CustomCommandResponse, CMD_RESP_MAX_SIZE);
          MSG_DBG("Response Size: %d bytes", lvResponseSize);
          MQTT_SendRaw(MQTT_TOPIC_RESP_RAW, (uint8_t*)g_CustomCommandResponse, lvResponseSize);
          g_CustomCommandPending = false;
        }
      }
      if ((s_LastUpdateTime == 0) || ((millis() - s_LastUpdateTime) >= (unsigned long)g_Settings.UpdateIntervalSec * 1000))
      {
        s_LastUpdateTime = millis();
        if (Davis_WakeUp())
        {
          bool lvSendUpdate = false;
          char lvLoopResponse[sizeof(LoopPacket)];
          uint16_t lvCRC;
          if (Davis_SendCommand("LOOP 1", lvLoopResponse, sizeof(LoopPacket), true, DAVIS_LOOP_COMMAND_TIMEOUT_MS))
          {
            lvCRC = CalcCrc((uint8_t*)lvLoopResponse, sizeof(LoopPacket));
            if (lvCRC == 0)
            {
              LoopPacket* lvLoopPacket = (LoopPacket*) lvLoopResponse;
              if (Davis_ConvertLoopData(lvLoopPacket, &g_StationData))
              {
                MQTT_SendRaw(MQTT_TOPIC_RAW_LOOP, (uint8_t*)lvLoopPacket, sizeof(LoopPacket));
                delay(500);
                lvSendUpdate = true;
              }
              else
              {
                MSG_DBG("Error converting LOOP data!");
              }
              //MSG_DBG("NextRecord: %d", lvLoopPacket->NextRecord);
              MSG_DBG("InTemperature: %.2f C", ((((float)lvLoopPacket->InTemperature / 10.0f) - 32.0f) * 5.0f) / 9.0f);
              MSG_DBG("InHumidity: %d %%", lvLoopPacket->InHumidity);
            }
            else
            {
              MSG_DBG("Error: CRC failure in LOOP packet! (CRC: 0x%04X)", lvCRC);
            }
          }
          if (Davis_WakeUp())
          {
            if (Davis_SendCommand("LPS 2 1", lvLoopResponse, sizeof(Loop2Packet), true, DAVIS_LOOP_COMMAND_TIMEOUT_MS))
            {
              lvCRC = CalcCrc((uint8_t*)lvLoopResponse, sizeof(Loop2Packet));
              if (lvCRC == 0)
              {
                Loop2Packet* lvLoop2Packet = (Loop2Packet*) lvLoopResponse;
                if (Davis_ConvertLoop2Data(lvLoop2Packet, &g_StationData))
                {
                  MQTT_SendRaw(MQTT_TOPIC_RAW_LOOP2, (uint8_t*)lvLoop2Packet, sizeof(Loop2Packet));
                  delay(500);
                  lvSendUpdate = true;
                }
                else
                {
                  MSG_DBG("Error converting LOOP2 data!");
                }
              }
              else
              {
                MSG_DBG("Error: CRC failure in LOOP2 packet! (CRC: 0x%04X)", lvCRC);
              }
            }
          }
          delay(1000);
          if (lvSendUpdate)
          {
            MQTT_SendState();
          }
        }
        else
        {
          MSG_DBG("Could not wake-up Davis");
        }
      }
      if (g_TriggerArchiveDownload || g_TriggerArchiveDownloadWithDateTime)
      {
        if (Davis_WakeUp())
        {
          s_ArchivePageNr = 0;
          s_ArchivePageCount = 0;
          s_ArchiveRecordStart = 0;
          const char *lvResponse;
          if (!g_TriggerArchiveDownloadWithDateTime)
          {            
            MSG_DBG("Starting download of archived data."); 
            if (Davis_StartReadArchiveData(&s_ArchivePageCount, &s_ArchiveRecordStart))
            {
              s_State = STATE_GET_ARCHIVE_DATA;
              lvResponse = "GetArchive: OK";
            }
            else
            {
              MSG_DBG("Error. Could not start archive data retrieval!");               
              lvResponse = "GetArchive: ERROR";
            }
          }
          else
          {            
            MSG_DBG("Starting download of archived data. Start time: %04d-%02d-%02d %02d:%02d", g_DateTimeStruct.Year, g_DateTimeStruct.Month, g_DateTimeStruct.Day, g_DateTimeStruct.Hours, g_DateTimeStruct.Minutes); 
            if (Davis_StartReadArchiveData(&s_ArchivePageCount, &s_ArchiveRecordStart, g_DateTimeStruct.Year, g_DateTimeStruct.Month, g_DateTimeStruct.Day, g_DateTimeStruct.Hours, g_DateTimeStruct.Minutes))
            {
              s_State = STATE_GET_ARCHIVE_DATA;
              lvResponse = "GetArchive: OK";
            }
            else
            {
              MSG_DBG("Error. Could not start archive data retrieval!"); 
              lvResponse = "GetArchive: ERROR";
            }
          }
          MQTT_SendRaw(MQTT_TOPIC_RESP, (uint8_t*)lvResponse, strlen(lvResponse));
          g_TriggerArchiveDownload = false;
          g_TriggerArchiveDownloadWithDateTime = false;
        }
      }
      if (g_GetTimeCommand && Davis_WakeUp())
      {
        char lvResponse[32];
        if (Davis_GetTime(&g_DateTimeStruct))
        {
          snprintf(lvResponse, sizeof(lvResponse), PSTR("GetTime: %04d-%02d-%02d %02d:%02d:%02d"), g_DateTimeStruct.Year, g_DateTimeStruct.Month, g_DateTimeStruct.Day, g_DateTimeStruct.Hours, g_DateTimeStruct.Minutes, g_DateTimeStruct.Seconds);
        }
        else
        {
          snprintf(lvResponse, sizeof(lvResponse), PSTR("GetTime: ERROR"));
        }
        MQTT_SendRaw(MQTT_TOPIC_RESP, (uint8_t*)lvResponse, strlen(lvResponse));
        g_GetTimeCommand = false;
      }
      if (g_SetTimeCommand && Davis_WakeUp())
      {
        char lvResponse[32];
        if (Davis_SetTime(&g_DateTimeStruct))
        {
          snprintf(lvResponse, sizeof(lvResponse), PSTR("SetTime: OK"));
        }
        else
        {
          snprintf(lvResponse, sizeof(lvResponse), PSTR("SetTime: ERROR"));
        }
        MQTT_SendRaw(MQTT_TOPIC_RESP, (uint8_t*)lvResponse, strlen(lvResponse));        
        g_SetTimeCommand = false;        
      }
      break;
    case STATE_GET_ARCHIVE_DATA:
      ArchivePage *lvArchivePage;
      uint16_t lvPageNr;
      if (Davis_ContinueReadArchiveData(&lvArchivePage, &lvPageNr, 3))
      {
          MSG_DBG("Page %d. SeqNr: %03d", lvPageNr, lvArchivePage->SeqNr);
          for (int j = 0; j < 5; j++)
          {
            ArchiveRecordRevB *lvArchiveRecord = &lvArchivePage->Record[j];
            uint16_t lvDateStamp = lvArchiveRecord->DateStamp;
            uint16_t lvTimeStamp = lvArchiveRecord->TimeStamp;
            if ((lvPageNr == 0) && (j < s_ArchiveRecordStart))
            {
              MSG_DBG(" - [%d] Record Date: %02d-%02d-%04d %02d:%02d (Skipped)", j, DATESTAMP_DAY(lvDateStamp), DATESTAMP_MONTH(lvDateStamp), DATESTAMP_YEAR(lvDateStamp), TIMESTAMP_HOUR(lvTimeStamp), TIMESTAMP_MIN(lvTimeStamp));
            }
            else if ((lvDateStamp == 0xFFFF) || (lvDateStamp == 0xFFFF))
            {
              MSG_DBG(" - [%d] Record Date: %02d-%02d-%04d %02d:%02d (Skipped)", j, DATESTAMP_DAY(lvDateStamp), DATESTAMP_MONTH(lvDateStamp), DATESTAMP_YEAR(lvDateStamp), TIMESTAMP_HOUR(lvTimeStamp), TIMESTAMP_MIN(lvTimeStamp));
            }
            else
            {
              MSG_DBG(" - [%d] Record Date: %02d-%02d-%04d %02d:%02d", j, DATESTAMP_DAY(lvDateStamp), DATESTAMP_MONTH(lvDateStamp), DATESTAMP_YEAR(lvDateStamp), TIMESTAMP_HOUR(lvTimeStamp), TIMESTAMP_MIN(lvTimeStamp));
              char lvTopic[128];
              snprintf(lvTopic, sizeof(lvTopic), PSTR("%s/%04d%02d%02d_%02d%02d"), MQTT_TOPIC_ARCHIVE, DATESTAMP_YEAR(lvDateStamp), DATESTAMP_MONTH(lvDateStamp), DATESTAMP_DAY(lvDateStamp), TIMESTAMP_HOUR(lvTimeStamp), TIMESTAMP_MIN(lvTimeStamp));
              MQTT_SendRaw(lvTopic, (uint8_t*)lvArchiveRecord, sizeof(ArchiveRecordRevB));          
            }
          }
      }
      else
      {
        Davis_StoptReadArchiveData();
        s_State = STATE_IDLE;
      }
      break;
  }
  
}

static bool Davis_Init(void)
{
  if (Davis_WakeUp())
  {
    char lvTemp[64];
    if (Davis_SendCommand("NVER", g_StationData.FWVersion, sizeof(g_StationData.FWVersion)))
    {
      MSG_DBG("Davis FWVersion: %s", g_StationData.FWVersion);
    }
    if (Davis_SendCommand("VER", g_StationData.FWDate, sizeof(g_StationData.FWDate)))
    {
      MSG_DBG("Davis FWDate: %s", g_StationData.FWDate);
    }

    if (Davis_SendCommand("RECEIVERS", (char*)&g_StationData.Receivers, sizeof(g_StationData.Receivers)))
    {
      MSG_DBG("Davis RECEIVERS: 0x%02X", g_StationData.Receivers);
    }
    if (Davis_SendCommand("RXCHECK", lvTemp, sizeof(lvTemp)))
    {
      MSG_DBG("Davis RXCHECK: %s", lvTemp);
    }
    Davis_GetTime();
    return true;
  }
  return false;
}

static bool Davis_GetTime(uint16_t *outYear, uint8_t *outMonth, uint8_t *outDay, uint8_t *outHours, uint8_t *outMinutes, uint8_t *outSeconds)
{
  uint8_t lvPacket[8];
  if (Davis_SendCommand("GETTIME", (char*)lvPacket, sizeof(lvPacket), true))
  {
    TimePacket *lvTimePacket = (TimePacket *)lvPacket;
    MSG_DBG("Current Time: %02d-%02d-%04d %02d:%02d:%02d", lvTimePacket->Day, lvTimePacket->Month, (uint16_t)1900+lvTimePacket->Year, lvTimePacket->Hours, lvTimePacket->Minutes, lvTimePacket->Seconds);
    if (outYear)
    {
      *outYear = (uint16_t)1900+lvTimePacket->Year;
    }
    if (outMonth)
    {
      *outMonth = lvTimePacket->Month;
    }
    if (outDay)
    {
      *outDay = lvTimePacket->Day;
    }
    if (outHours)
    {
      *outHours = lvTimePacket->Hours;
    }
    if (outMinutes)
    {
      *outMinutes = lvTimePacket->Minutes;
    }
    if (outSeconds)
    {
      *outSeconds = lvTimePacket->Seconds;
    }
    return true;
  }
  return false;  
}

static bool Davis_SetTime(uint16_t inYear, uint8_t inMonth, uint8_t inDay, uint8_t inHours, uint8_t inMinutes, uint8_t inSeconds)
{
  uint8_t lvPacket[8];
  TimePacket *lvTimePacket = (TimePacket*)lvPacket;
  lvTimePacket->Seconds = inSeconds;
  lvTimePacket->Minutes = inMinutes;
  lvTimePacket->Hours = inHours;
  lvTimePacket->Day = inDay;
  lvTimePacket->Month = inMonth;
  lvTimePacket->Year = (inYear > 1900 ? (inYear - 1900) : inYear);
  uint16_t lvCRC = CalcCrc((uint8_t*)lvPacket, 6);
  lvPacket[6] = (uint8_t)(lvCRC >> 8);
  lvPacket[7] = (uint8_t)(lvCRC & 0xFF);
  if (Davis_SendCommand("SETTIME", 0, 0))
  { 
    MSG_DBG("Setting date+time to: %02d-%02d-%04d %02d:%02d:%02d", lvTimePacket->Day, lvTimePacket->Month, (uint16_t)1900+lvTimePacket->Year, lvTimePacket->Hours, lvTimePacket->Minutes, lvTimePacket->Seconds);
    Davis_Write(lvPacket, sizeof(lvPacket));
    DavisCommandResponse lvResponse = Davis_ReadCommandResponse(5000);
    if (lvResponse != RESP_ACK)
    {
      MSG_DBG("Error: Invalid response: %s", PRINT_RESPONSE_TYPE(lvResponse));
      return false;
    }
    else
    {
      MSG_DBG("Time set OK!");
      return true;
    }
  }
  return false;
}

static bool Davis_SetTime(DateTimeStruct *inDateTimeStruct)
{
  return Davis_SetTime(inDateTimeStruct->Year, inDateTimeStruct->Month, inDateTimeStruct->Day, inDateTimeStruct->Hours, inDateTimeStruct->Minutes, inDateTimeStruct->Seconds);
}
static bool Davis_GetTime(DateTimeStruct *outDateTimeStruct)
{
  return Davis_GetTime(&outDateTimeStruct->Year, &outDateTimeStruct->Month, &outDateTimeStruct->Day, &outDateTimeStruct->Hours, &outDateTimeStruct->Minutes, &outDateTimeStruct->Seconds);
}


static void Davis_FlushRx(DebugType inDebug)
{
  uint16_t lvBytesFlushed = 0;
  uint8_t lvByte;
  // flush RX buffer
  while(Serial.available())
  {
    lvByte = Serial.read();
#ifdef DEBUG_LOW_LEVEL    
    if (inDebug)
    {
      if (lvBytesFlushed == 0)
      {
        MSG_DBG_NO_LINE("FL: ");
      }
      if (inDebug & DEBUG_HEX)
      {
        MSG_DBG_NO_LINE("%02X ", lvByte);
      }
    }
#endif //DEBUG_LOW_LEVEL
    lvBytesFlushed++;
  }
#ifdef DEBUG_LOW_LEVEL  
  if (inDebug && (lvBytesFlushed > 0))
  {
    MSG_DBG("");
  }
#endif //DEBUG_LOW_LEVEL  
}

static void Davis_Write(const uint8_t *inBuf, uint16_t inSize, DebugType inDebug)
{
  Serial.write(inBuf, inSize);
#ifdef DEBUG_LOW_LEVEL
  if (inDebug)
  {
    MSG_DBG_NO_LINE("TX: ");
    DUMP_BYTES(inBuf, inSize, inDebug);
    MSG_DBG("");
  }
#endif //DEBUG_LOW_LEVEL
}
static void Davis_Write(const char *inBuf, DebugType inDebug)
{
  Davis_Write((const uint8_t *)inBuf, strlen(inBuf), inDebug);
}
static void Davis_Write(uint8_t inByte, DebugType inDebug)
{
  Davis_Write((const uint8_t *)&inByte, 1, inDebug);
}
static bool Davis_ReadByte(uint8_t *outByte, DebugType inDebug)
{
  if (Serial.available()) 
  {
    *outByte = Serial.read();
#ifdef DEBUG_LOW_LEVEL
    if (inDebug)
    {
      MSG_DBG("RX: %02X", *outByte);
    }
#endif //DEBUG_LOW_LEVEL
    return true;
  }
  return false;
}

static uint16_t Davis_Read(uint8_t *outBuf, uint16_t inBufSize, bool inStopOnNewLine, uint16_t inByteTimeoutMs, DebugType inDebug)
{
  uint16_t lvBytesReceived = 0;
  unsigned long lvByteTime = millis();
  while ((lvBytesReceived < inBufSize) && ((millis() - lvByteTime) < inByteTimeoutMs))
  {
    if (Davis_ReadByte(&outBuf[lvBytesReceived], DEBUG_NONE))
    {
      lvByteTime = millis();
      if ((inStopOnNewLine) && (outBuf[lvBytesReceived] == '\n'))
      {
        outBuf[lvBytesReceived] = '\0';
        lvBytesReceived++;
        break;
      }
      lvBytesReceived++;
    }
  }
#ifdef DEBUG_LOW_LEVEL  
  if (inDebug)
  {
    MSG_DBG_NO_LINE("RX: ");
    DUMP_BYTES(outBuf, lvBytesReceived, inDebug);
    MSG_DBG("");
  }
#endif //DEBUG_LOW_LEVEL
  return lvBytesReceived;
}
  
static DavisCommandResponse Davis_ReadCommandResponse(uint16_t inTimeoutMs, DebugType inDebug)
{
  DavisCommandResponse lvResponse = RESP_INVALID;
  unsigned long lvStartTime = millis();
  char lvResponseBuf[8];
  unsigned char lvIdx = 0;
  
  while ((lvResponse == RESP_INVALID) && (lvIdx < sizeof(lvResponseBuf)) && ((millis() - lvStartTime) < inTimeoutMs))
  {
    if (Davis_ReadByte((uint8_t*)&lvResponseBuf[lvIdx], DEBUG_NONE)) 
    {
      if (lvResponseBuf[lvIdx] == ACK)
      {
        lvResponse = RESP_ACK;
      }
      else if (lvResponseBuf[lvIdx] == NACK)
      {
        lvResponse = RESP_NACK;
      }
      lvIdx++;
      if ((lvIdx == strlen(RESP_OK_STR)) && (memcmp(lvResponseBuf, RESP_OK_STR, strlen(RESP_OK_STR)) == 0))
      {
        lvResponse = RESP_OK;
      }
    }
  }
  if (lvResponse == RESP_INVALID)
  {
    lvResponse = RESP_TIMEOUT;
  }
#ifdef DEBUG_LOW_LEVEL 
  if (inDebug)
  {
    MSG_DBG_NO_LINE("RX: ");
    DUMP_BYTES(lvResponseBuf, lvIdx, inDebug);
    MSG_DBG("(%s)", PRINT_RESPONSE_TYPE(lvResponse));
  } 
#endif //DEBUG_LOW_LEVEL
  return lvResponse;
}

static bool Davis_SendCommand(const char *inCommand, char *outResponse, uint16_t inMaxResponseLength, bool inBinaryResponse, uint16_t inTimeoutMs)
{
  // flush RX buffer
  Davis_FlushRx();
  
  Davis_Write(inCommand);
  Davis_Write("\n");
  DavisCommandResponse lvResponse =  Davis_ReadCommandResponse(inTimeoutMs);
 
  if (!IS_GOOD_RESPONSE(lvResponse))
  {
    MSG_DBG("Command '%s' did not return good response! Response: %s", inCommand, PRINT_RESPONSE_TYPE(lvResponse));
    return false;
  }
  
  if (outResponse)
  {
    uint16_t lvResponseSize = Davis_Read((uint8_t*)outResponse, inMaxResponseLength, !inBinaryResponse, DAVIS_BYTE_TIMEOUT_MS, (inBinaryResponse ? DEBUG_HEX : DEBUG_HEXASCII));

    if ((inBinaryResponse && (lvResponseSize != inMaxResponseLength)) || (!inBinaryResponse && (outResponse[lvResponseSize-1] != '\0')))
    {
      MSG_DBG("Timeout while waiting for response from '%s' command! (Response: '%s')", inCommand, outResponse);
      return false;
    }
  }
  return true;
}


static uint16_t Davis_SendRawCommand(const char *inCommand, char *outResponse, uint16_t inMaxResponseLength, uint16_t inByteTimeoutMs)
{
  //unsigned long lvByteTime;
  uint16_t lvBytesReceived = 0;
  
  // flush RX buffer
  Davis_FlushRx();
  
  Davis_Write(inCommand);
  if (inCommand[strlen(inCommand)-1] != '\n')
  {
    Davis_Write("\n");
  }
  lvBytesReceived = Davis_Read((uint8_t*)outResponse, inMaxResponseLength, false);
  return lvBytesReceived;
}

static bool Davis_StartReadArchiveData(uint16_t *outPageCount, uint16_t *outFirstRecord, uint16_t inDateStamp, uint16_t inTimeStamp)
{
  uint16_t lvDateStamp = inDateStamp;
  uint16_t lvTimeStamp = inTimeStamp;
  if (Davis_SendCommand("DMPAFT", 0, 0))
  {
    unsigned long lvByteTime;
    
    uint8_t lvDateTimeStamp[6];
    
    uint16_t lvCRC;
    lvDateTimeStamp[0] = (uint8_t)(lvDateStamp & 0xFF);
    lvDateTimeStamp[1] = (uint8_t)(lvDateStamp >> 8);
    lvDateTimeStamp[2] = (uint8_t)(lvTimeStamp & 0xFF);
    lvDateTimeStamp[3] = (uint8_t)(lvTimeStamp >> 8);
    lvCRC = CalcCrc((uint8_t*)lvDateTimeStamp, 4);
    lvDateTimeStamp[4] = (uint8_t)(lvCRC >> 8);
    lvDateTimeStamp[5] = (uint8_t)(lvCRC & 0xFF);
    //lvDateTimeStamp[6] = '\n';
    Davis_Write(lvDateTimeStamp, sizeof(lvDateTimeStamp));

    uint8_t lvResponse[7];
    uint16_t lvBytesReceived = Davis_Read(lvResponse, sizeof(lvResponse), false, 100*DAVIS_BYTE_TIMEOUT_MS);

    lvCRC = CalcCrc((uint8_t*)lvResponse+1, 6);
    if ((lvBytesReceived != sizeof(lvResponse) || (lvResponse[0] != ACK) || (lvCRC != 0)))
    {
      MSG_DBG_NO_LINE("Invalid response after DMPAFT datetimestamp (%d bytes, CRC: 0x04X): ", lvBytesReceived, lvCRC);
      for (int i = 0; i < lvBytesReceived; i++)
      {
        MSG_DBG_NO_LINE("%02X ", lvResponse[i]);
      }
      MSG_DBG("");
      return false;
    }    
    
    *outPageCount = ((uint16_t)lvResponse[2] << 8) + lvResponse[1];
    *outFirstRecord = ((uint16_t)lvResponse[4] << 8) + lvResponse[3];
    MSG_DBG("Page Count: %d", *outPageCount);
    MSG_DBG("First Record: %d", *outFirstRecord);
    s_ArchivePageNr = 0;
    s_ArchivePageCount = *outPageCount;
    return true;
  }
  return false;
}

static bool Davis_StartReadArchiveData(uint16_t *outPageCount, uint16_t *outFirstRecord, uint16_t inYear, uint8_t inMonth, uint8_t inDay, uint8_t inHour, uint8_t inMinute)
{
  MSG_DBG("Requesting archive data from %04d-%02d-%02d %02d:%02d", inYear, inMonth, inDay, inHour, inMinute);
  uint16_t lvDateStamp = DATE_TO_DATESTAMP(inDay, inMonth, inYear);
  uint16_t lvTimeStamp = TIME_TO_TIMESTAMP(inHour, inMinute);
  return Davis_StartReadArchiveData(outPageCount, outFirstRecord, lvDateStamp, lvTimeStamp);
}

static bool Davis_ContinueReadArchiveData(ArchivePage **outArchivePage, uint16_t *outPageNr, uint8_t inRetries)
{
  if (s_ArchivePageNr < s_ArchivePageCount)
  {
    uint8_t lvRetries = 0;
    Davis_Write(ACK);
    while (lvRetries < inRetries)
    {
      uint16_t lvBytesReceived = Davis_Read(s_ArchivePageBuf, sizeof(s_ArchivePageBuf), false, 100*DAVIS_BYTE_TIMEOUT_MS);
      if (lvBytesReceived == sizeof(s_ArchivePageBuf))
      {
        uint16_t lvCRC = CalcCrc(s_ArchivePageBuf, lvBytesReceived);
        if (lvCRC == 0)
        {
          // CRC valid
          *outArchivePage = (ArchivePage *)s_ArchivePageBuf;
          *outPageNr = s_ArchivePageNr;
          s_ArchivePageNr++;
          return true;
        }
        else
        {
          MSG_DBG("invalid CRC on page %d", s_ArchivePageNr);
          Davis_Write(NACK);
          lvRetries++;          
        }
      }
      else
      {
        MSG_DBG("Not enough bytes received for page %d (%d)", s_ArchivePageNr, lvBytesReceived);
        Davis_Write(NACK);
        lvRetries++;
      }
    }
    if (lvRetries >= inRetries)
    {
      MSG_DBG("Retry limit reached on page %d (%d)", s_ArchivePageNr, lvRetries);
      Davis_StoptReadArchiveData();
    }  
  }
  return false;
}

static void Davis_StoptReadArchiveData()
{
  Davis_Write(ESC);
}

static bool Davis_WakeUp(void)
{
  //Console Wakeup procedure:
  //1. Send a Line Feed character, ‘\n’ (decimal 10, hex 0x0A).
  //2. Listen for a returned response of Line Feed and Carriage Return characters, (‘\n\r’).
  //3. If there is no response within a reasonable interval (say 1.2 seconds), then try steps 1 and
  //2 again up to a total of 3 attempts.
  //4. If the console has not woken up after 3 attempts, then signal a connection error

  char lvResponseBuf[2];
  bool lvCorrectResponseReceived = false;
  uint8_t lvWakeupAttempts = 3;

  //MSG_DBG("Attempting wake-up of Davis console.");
  do
  {
    // flush RX buffer
    Davis_FlushRx();
    Davis_Write("\n\n");

    //lvStartTime = millis();
    //lvIdx = 0;
    lvCorrectResponseReceived = false;
    lvResponseBuf[0] = 0;
    lvResponseBuf[1] = 0;
    Davis_Read((uint8_t*)lvResponseBuf, sizeof(lvResponseBuf), false, DAVIS_WAKEUP_TIMEOUT_MS);

    if ((lvResponseBuf[0] == '\n') && (lvResponseBuf[1] == '\r'))
    {
       lvCorrectResponseReceived = true;
    }

  } while(!lvCorrectResponseReceived && (--lvWakeupAttempts > 0));
  if (!lvCorrectResponseReceived)
  {
    MSG_DBG("Failed to wake-up Davis Weather Station!");
    return false;
  }
  //MSG_DBG("Wake-up successful!");
  return true;
}

static bool Davis_ConvertLoopData(LoopPacket* inLoopPacket, StationData * outStationData)
{
  outStationData->InsideTemperature  = DAVIS_CONVERT_TEMPERATURE_10TH(inLoopPacket->InTemperature);
  outStationData->OutsideTemperature = DAVIS_CONVERT_TEMPERATURE_10TH(inLoopPacket->OutTemperature);
   
  outStationData->BarometricPressure = DAVIS_CONVERT_BAR_PRESSURE(inLoopPacket->Barometer);
  outStationData->BarometricTrend = inLoopPacket->BarTrend;
  outStationData->InsideHumidity = inLoopPacket->InHumidity;
  outStationData->OutsideHumidity = inLoopPacket->OutHumidity;

  outStationData->WindSpeed = DAVIS_CONVERT_WINDSPEED(inLoopPacket->WindSpeed);
  outStationData->AvgWindSpeed = DAVIS_CONVERT_WINDSPEED(inLoopPacket->AvgWindSpeed);
  
  outStationData->WindDirection = inLoopPacket->WindDirection;

  for (int i = 0; i < 7; i++)
  {
    outStationData->ExtraTemps[i] = DAVIS_CONVERT_EXTRA_TEMPERATURE(inLoopPacket->ExtraTemps[i]);
  }
  for (int i = 0; i < 4; i++)
  {
    outStationData->SoilTemps[i] = DAVIS_CONVERT_EXTRA_TEMPERATURE(inLoopPacket->SoilTemps[i]);
  }
  for (int i = 0; i < 4; i++)
  {
    outStationData->LeafTemps[i] = DAVIS_CONVERT_EXTRA_TEMPERATURE(inLoopPacket->LeafTemps[i]);
  }
  for (int i = 0; i < 7; i++)
  {
    outStationData->ExtraHumidity[i] = inLoopPacket->ExtraHumidity[i];
  }
  outStationData->RainRate = DAVIS_CONVERT_RAINRATE(inLoopPacket->RainRate);
  outStationData->RainDaily = DAVIS_CONVERT_RAINRATE(inLoopPacket->RainDay);
  
  outStationData->UVindex = inLoopPacket->UVindex;
  outStationData->SolarRadiation = inLoopPacket->SolarRadiation;

  outStationData->Battery_Transmitter = inLoopPacket->Battery_Transmitter;
  outStationData->Battery_Console = DAVIS_CONVERT_BATT_VOLTAGE(inLoopPacket->Battery_Console);

  outStationData->ForecastIcons = inLoopPacket->ForecastIcons;
  outStationData->ForecastRule = inLoopPacket->ForecastRule;

  snprintf(outStationData->TimeSunrise, sizeof(outStationData->TimeSunrise), "%02d:%02d", (inLoopPacket->TimeSunrise / 100), inLoopPacket->TimeSunrise % 100);
  snprintf(outStationData->TimeSunset, sizeof(outStationData->TimeSunset), "%02d:%02d", (inLoopPacket->TimeSunset / 100), inLoopPacket->TimeSunset % 100);
  return true;
}

static bool Davis_ConvertLoop2Data(Loop2Packet* inLoop2Packet, StationData * outStationData)
{
  outStationData->DewPoint = DAVIS_CONVERT_DEW_POINT(inLoop2Packet->DewPoint);
  outStationData->WindChillTemp = DAVIS_CONVERT_DEW_POINT(inLoop2Packet->WindChill);
 
  outStationData->Rain15min = DAVIS_CONVERT_RAINRATE(inLoop2Packet->Rain15Min);
  outStationData->RainHour = DAVIS_CONVERT_RAINRATE(inLoop2Packet->RainHour);
  outStationData->Rain24Hrs = DAVIS_CONVERT_RAINRATE(inLoop2Packet->Rain24Hrs);
  
  return true;
}
