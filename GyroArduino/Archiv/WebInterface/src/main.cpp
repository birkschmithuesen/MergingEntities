//Connections : Please always connect 2 hard mpu (builtin I2C bus) to your specified pins
//Indicate the numbers of hard, soft I2C, connect the soft I2C in the order of the specified pins
//Specify your IP address and Wifi
//Mag calibration desactivated right now, see if it's usefull

//-------LIBRARIES-------
//Library to use Arduino cmd
#include <Arduino.h>

//Libraries for Comm
#include <WiFi.h>
#include <OSCMessage.h>
#include <WiFiUdp.h>

//-------WIFI SETTINGS AND FUNCTIONS-------
//Settings to connect to WiFi

//Settings to other WiFi
#define BUFF_SIZE 64
char WIFI_SSID[BUFF_SIZE] = "TheaterDo-GAST";
char WIFI_PASSWORD[BUFF_SIZE] = "theaterdortmund";

WiFiUDP Udp;
IPAddress outIp(192,168,0,2); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP

//Function to connect WiFi
void connectWiFi() //Let's connect a WiFi
{
  Serial.print("Starting WiFi connection ...");
  
  WiFi.mode(WIFI_STA); //Mode of the WiFi, STA = STATION MODE (connect to stg), APM = Access Point Mode (create a network)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); //Connecting the ESP

  long start = millis();

  while(WiFi.status() != WL_CONNECTED && millis() - start < 10000)
  {
    Serial.print(".");
    delay(100);
  }

  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("__Not connected__");
  }
  else
  {
    Serial.print("__Connected. IP adress : ");
    Serial.println(WiFi.localIP());
  }
}

//Function to start Udp communication on arduino side
void startUdp() {
  Serial.print("Starting Udp connection to local port : ");
  Udp.begin(localPort); //set the connection with the computer
  Serial.println(localPort);
}

//Function to read the incoming OSCMessage - if a new setting is given, it switch to it
void setWifiSetting() {

  OSCMessage wifiSetting;
  char buffSSID[BUFF_SIZE];
  char buffPASSWORD[BUFF_SIZE];

  int size = Udp.parsePacket();

  //SEND : Default Wifi setting : ###### / #####
  //SEND : To change Wifi setting, send WIFI_SSID WIFI_PASSWORD. To keep this setting, send anything else

  while (size == 0) {
    size = Udp.parsePacket();

    if (size > 0) { //We fill the packet with incoming data
      while (size--) {
        wifiSetting.fill(Udp.read());
      }
    }
  }

  if((wifiSetting.isString(0)) && (wifiSetting.isString(1))) {
    
    WiFi.disconnect();
    wifiSetting.getString(0, WIFI_SSID, BUFF_SIZE);
    wifiSetting.getString(0, WIFI_PASSWORD, BUFF_SIZE);
    connectWiFi();
    startUdp();
  }

}

//-------SETUP-------
void setup() {
  Serial.begin(115200);
  Serial.flush(); //Clean buffer

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp();

  setWifiSetting();
}

void loop() {

}