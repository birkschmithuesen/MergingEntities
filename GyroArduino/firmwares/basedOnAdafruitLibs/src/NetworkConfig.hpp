#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>


#define WIFI_SSID "syntheticwire"     /**< SSID / name of the wifi network to use */
#define WIFI_PASS "doesnotmatter"  /**< password for the wifi network to use */


#define  UDP_localPort  8888    

void setupWifi(WiFiUDP& Udp);               /**< try toconnect to Wifi */