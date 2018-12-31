
/*** INCLUDES ***/
#include "Settings.h"

#ifdef WIFI_ENABLED


void WiFi_MQTT_Init(void);
void WiFi_MQTT_Tick(void);

#ifdef NTP_ENABLED
 const char *WiFi_MQTT_GetTime(void);
#endif // NTP_ENABLED
        
void MQTT_SendConfig(void);
void MQTT_SendState(void);
 
void MQTT_SendRaw(const char* inTopic, uint8_t *inData, uint16_t inLength);

#endif // WIFI_ENABLED
