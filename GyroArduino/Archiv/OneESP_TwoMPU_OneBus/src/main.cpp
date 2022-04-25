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
MPU9250Setting setting1;
MPU9250Setting setting2;

float qX1 = 0, qY1 = 0, qZ1 = 0, qW1 = 0;
float qX2 = 0, qY2 = 0, qZ2 = 0, qW2 = 0;

float oX1 = 0, oY1 = 0, oZ1 = 0;
float oX2 = 0, oY2 = 0, oZ2 = 0;

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

  setting1.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting1.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting1.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting1.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting1.gyro_fchoice = 0x03;
  setting1.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting1.accel_fchoice = 0x01;
  setting1.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  setting2.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting2.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting2.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting2.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting2.gyro_fchoice = 0x03;
  setting2.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting2.accel_fchoice = 0x01;
  setting2.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  mpu1.setup(0x68, setting1);  // change to your own address
  mpu2.setup(0x69, setting2);

  Serial.println("Calibration of acceleration : don't move devices");
  mpu1.calibrateAccelGyro();
  mpu2.calibrateAccelGyro();
  Serial.println("Calibration of mag 1");
  mpu1.setMagneticDeclination(2.53);
  //mpu1.calibrateMag();

  Serial.println("Calibration of mag 2");
  mpu2.setMagneticDeclination(2.53);
  //mpu2.calibrateMag();

  QuatFilterSel sel1{QuatFilterSel::MADGWICK};
  QuatFilterSel sel2{QuatFilterSel::MADGWICK};
  
  mpu1.selectFilter(sel1);
  mpu2.selectFilter(sel2);

  mpu1.setFilterIterations(10);
  mpu2.setFilterIterations(10);
}

void loop() {
  //--------MPU recording--------
  mpu2.update();
  //mpu1.update();

  static unsigned long last_print=0;
  
  
  if (millis()-last_print > 100) {
        Serial.print(mpu1.getQuaternionX()); Serial.print(", ");
        Serial.print(mpu1.getQuaternionY()); Serial.print(", ");
        Serial.print(mpu1.getQuaternionZ()); Serial.print(" /////");
        Serial.print(mpu2.getQuaternionX()); Serial.print(", ");
        Serial.print(mpu2.getQuaternionY()); Serial.print(", ");
        Serial.println(mpu2.getQuaternionZ());

        qX1 = mpu1.getQuaternionX();
        qY1 = mpu1.getQuaternionY();
        qZ1 = mpu1.getQuaternionZ();
        qW1 = mpu1.getQuaternionW();

        qX2 = mpu2.getQuaternionX();
        qY2 = mpu2.getQuaternionY();
        qZ2 = mpu2.getQuaternionZ();
        qW2 = mpu2.getQuaternionW();

        oX1 = mpu1.getEulerX();
        oY1 = mpu1.getEulerY();
        oZ1 = mpu1.getEulerZ();

        oX2 = mpu2.getEulerX();
        oY2 = mpu2.getEulerY();
        oZ2 = mpu2.getEulerZ();


        last_print=millis();
  }

  //-------OSC comm--------
  OSCMessage gyroQuater1("/gyro1/quater");
  OSCMessage gyroQuater2("/gyro2/quater");

  OSCMessage gyroAngle1("/gyro1/angle");
  OSCMessage gyroAngle2("/gyro2/angle");
  
  gyroQuater1.add(qX1).add(qY1).add(qZ1).add(qW1);//We put the quater data into the message
  gyroQuater2.add(qX2).add(qY2).add(qZ2).add(qW2);

  gyroAngle1.add(oX1).add(oY1).add(oZ1);//We put the angle data into the message
  gyroAngle2.add(oX2).add(oY2).add(oZ2);

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyroQuater1.send(Udp); //sends the message
  Udp.endPacket();//terminates the connection

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyroQuater2.send(Udp); //sends the message
  Udp.endPacket();

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyroAngle1.send(Udp); //sends the message
  Udp.endPacket();//terminates the connection

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyroAngle2.send(Udp); //sends the message
  Udp.endPacket();

  gyroQuater1.empty();
  gyroQuater2.empty();

  gyroAngle1.empty();
  gyroAngle2.empty();
}