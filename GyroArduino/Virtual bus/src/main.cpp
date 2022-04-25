//Library to use Arduino cmd
#include <Arduino.h>

//Libraries for Comm
#include <WiFi.h>
#include <OSCMessage.h>
#include <WiFiUdp.h>

//MPU library
#include "MPU9250.h"
#include <BitBang_I2C.h> //Simulate i2c bus

#define SDA_PIN 25
#define SCL_PIN 26

//Settings to connect to WiFi
#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"

//Seting to simulqte i2c bus
BBI2C bbi2c;

//Settings to communicate through WiFi
WiFiUDP Udp;
IPAddress outIp(192,168,193,221); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP

//Setting to initialiye gyro recording
#define MPU_ADDRESS 0x68 //Set 0x68 or 0x69
MPU9250 mpu1; // You can also use MPU9255 as is
MPU9250Setting setting;

float qX1 = 0, qY1 = 0, qZ1 = 0, qW1 = 0;
float oX1 = 0, oY1 = 0, oZ1 = 0;

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

  //Wireless communication setup
  connectWiFi();
  startUdp();

  //Virtual I2C setup
  bbi2c.bWire = 0; // use bit bang, not wire library
  bbi2c.iSDA = SDA_PIN;
  bbi2c.iSCL = SCL_PIN;
  I2CInit(&bbi2c, 100000L);
  delay(100);

  //Mpu setup
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  mpu1.setup(MPU_ADDRESS, setting, &bbi2c);  // change to your own address

  Serial.println("Calibration of acceleration : don't move devices");
  mpu1.calibrateAccelGyro();
  Serial.println("Acceleration calibration done.");
  Serial.println("Calibration of mag 1");
  mpu1.setMagneticDeclination(2.53);
  mpu1.calibrateMag();


  mpu1.setFilterIterations(10);

  QuatFilterSel sel{QuatFilterSel::MADGWICK};

  mpu1.selectFilter(sel);


}

void loop() {
  //--------MPU recording--------
  
  mpu1.update();

  static unsigned long last_print=0;
  
  
  if (millis()-last_print > 100) {
        Serial.print(mpu1.getQuaternionX()); Serial.print(", ");
        Serial.print(mpu1.getQuaternionY()); Serial.print(", ");
        Serial.println(mpu1.getQuaternionZ());

        qX1 = mpu1.getQuaternionX();
        qY1 = mpu1.getQuaternionY();
        qZ1 = mpu1.getQuaternionZ();
        qW1 = mpu1.getQuaternionW();

        oX1 = mpu1.getEulerX();
        oY1 = mpu1.getEulerY();
        oZ1 = mpu1.getEulerZ();


        last_print=millis();
  }

  //-------OSC comm--------
  OSCMessage gyroQuater1("/gyro1/quater");
  OSCMessage gyroAngle1("/gyro1/angle");
  
  gyroQuater1.add(qX1).add(qY1).add(qZ1).add(qW1);//We put the quater data into the message
  gyroAngle1.add(oX1).add(oY1).add(oZ1);//We put the angle data into the message

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyroQuater1.send(Udp); //sends the message
  Udp.endPacket();//terminates the connection

  Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
  gyroAngle1.send(Udp); //sends the message
  Udp.endPacket();//terminates the connection

  gyroQuater1.empty();
  gyroAngle1.empty();
}