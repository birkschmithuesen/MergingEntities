//Library to use Arduino cmd
#include <Arduino.h>

//Libraries for Comm
#include <WiFi.h>
#include <OSCMessage.h>
#include <WiFiUdp.h>

//Settings to connect to WiFi
#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"

//Settings to communicate through WiFi
WiFiUDP Udp;
IPAddress outIp(192,168,193,221); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP

float message = 0;

//Function to connect WiFi
void connectWiFi() //Let's connect a WiFi
{
  Serial.print("Starting WiFi connection ...");
  
  WiFi.mode(WIFI_STA); //Mode of the WiFi, STA = STATION MODE (connect to stg), APM = Access Point Mode (create a network)
  WiFi.begin(WIFI_SSID, WIFI_PASS); //Connecting the ESP

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

//-------SETUP-------

void setup() {
  Serial.begin(115200);

  //Connect Wifi
  connectWiFi();
  startUdp();
}

void loop() {

  //-------OSC comm--------
  OSCMessage gyroQuater1("/gyro1/quater");
  OSCMessage gyroAngle1("/gyro1/angle");
  
  gyroQuater1.add(message);//We put the quater data into the message

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyroQuater1.send(Udp); //sends the message
  Udp.endPacket();//terminates the connection

  gyroQuater1.empty();
}