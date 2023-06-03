#pragma once
#include <esp_wifi.h>
#include <WiFi.h>

#include <WiFiUdp.h>

  

void setupWifi(WiFiUDP &Udp,const  char* ssid, const char* password, uint16_t localPort);              /**< try toconnect to Wifi */