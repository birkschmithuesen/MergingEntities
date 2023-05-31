#include "NetworkConfig.hpp"

void setupWifi(WiFiUDP &Udp,const char* ssid,const  char* password, uint16_t localPort)
{
    Serial.print("Connecting to wifi network \"");
    Serial.print(ssid);
    Serial.print("\" .");
    // Mode of the WiFi
    //   STA = STATION MODE (connect to access point),
    //   APM = Access Point Mode (create a network)
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    long start = millis();
    // try for ten seconds to connect every 500 ms (i.e. make 10000/500 = 20 attempts)
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
    {
        Serial.print(".");
        delay(500);
    }
    // esp_wifi_set_ps(WIFI_PS_NONE);
    // print result of connection attempt(s) on serial console
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(" failed with status ");
        Serial.println(WiFi.status());
    }
    else
    {
        Serial.println(" succeeded");
        Serial.print("local IP address is ");
        Serial.println(WiFi.localIP());

        Serial.print("Starting UDP connection to local port ");
        Serial.print(localPort);
        if (0 == Udp.begin(localPort))
        {
            // no socket available for use
            Serial.println(" ... failed");
        }
        Serial.println(" ... succeeded");
            
    }

}