//Library to use Arduino cmd
#include <Arduino.h>

//Libraries for Comm
#include <WiFi.h>
#include <OSCMessage.h>
#include <WiFiUdp.h>

//MPU library
#include "MPU9250.h"

//Settings to connect to WiFi
#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"

//Settings to communicate through WiFi
WiFiUDP Udp;
IPAddress outIp(192,168,193,221); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP

//Setting to initialiye gyro recording
MPU9250 mpu1; // You can also use MPU9255 as is
MPU9250 mpu2;
MPU9250Setting setting;

float qX1 = 0, qY1 = 0, qZ1 = 0, qW1 = 0;
float qX2 = 0, qY2 = 0, qZ2 = 0, qW2 = 0;

int deltaT = 50; //Communication rate

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

  connectWiFi();
  startUdp();
  
  Wire.begin();
  delay(2000);

  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  mpu1.setup(0x68, setting);  // change to your own address
  mpu2.setup(0x69, setting);

  mpu1.setMagneticDeclination(2.53);
  mpu2.setMagneticDeclination(2.53);
  Serial.println("Calibration of acceleration : don't move devices");
  mpu1.calibrateAccelGyro();
  mpu2.calibrateAccelGyro();
  Serial.println("Calibration of 1 : round the MPU");
  mpu1.calibrateMag();
  Serial.println("Calibration of 2 : round the MPU");
  mpu2.calibrateMag();
  
}

void loop() {
  //--------MPU recording--------
  if (mpu1.update() && mpu2.update()) {
        Serial.print(mpu1.getEulerX()); Serial.print(", ");
        Serial.print(mpu1.getEulerY()); Serial.print(", ");
        Serial.print(mpu1.getEulerZ()); Serial.print(" /////");
        Serial.print(mpu2.getEulerX()); Serial.print(", ");
        Serial.print(mpu2.getEulerY()); Serial.print(", ");
        Serial.println(mpu2.getEulerZ());

        qX1 = mpu1.getQuaternionX();
        qY1 = mpu1.getQuaternionY();
        qZ1 = mpu1.getQuaternionZ();
        qW1 = mpu1.getQuaternionW();

        qX2 = mpu2.getQuaternionX();
        qY2 = mpu2.getQuaternionY();
        qZ2 = mpu2.getQuaternionZ();
        qW2 = mpu2.getQuaternionW();
    }

  //-------OSC comm--------
  OSCMessage gyro1("/gyro1/quater");
  OSCMessage gyro2("/gyro2/quater");
  
  gyro1.add(qX1).add(qY1).add(qZ1).add(qW1);//We put the quater data into the message
  gyro2.add(qX2).add(qY2).add(qZ2).add(qW2);

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyro1.send(Udp); //sends the message
  Udp.endPacket();//terminates the connection

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyro2.send(Udp); //sends the message
  Udp.endPacket();

  gyro1.empty();
  gyro2.empty();
}