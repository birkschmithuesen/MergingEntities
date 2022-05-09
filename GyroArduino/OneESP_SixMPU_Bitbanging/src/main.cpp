//-------LIBRARIES-------
//Library to use Arduino cmd
#include <Arduino.h>

//Libraries for Comm
#include <WiFi.h>
#include <OSCMessage.h>
#include <WiFiUdp.h>

//MPU and Bitbang library
#include "MPU9250.h"
#include <SoftWire.h>

//-------MPU SETTINGS-------
//Parameters of the setup
#define nbrSoftMpu 4
#define nbrHardMpu 2
#define nbrMpu nbrSoftMpu + nbrHardMpu

//Addresses and pin of MPU's
#define MPU_ADDRESS_1 0x68 //Set 0x68 or 0x69
#define MPU_ADDRESS_2 0x69 //Set 0x68 or 0x69

int SDA_PIN[nbrSoftMpu + nbrHardMpu] = {16, 18, 5, 32, 21, 26}; //16 = RX2, 26 and 27 should't be used with Wifi ...
int SCL_PIN[nbrSoftMpu + nbrHardMpu] = {17, 19, 23, 33, 22, 27}; //17 = TX2

//Software I2c
MPU9250_<SoftWire> Smpu[nbrSoftMpu]; //Mpu objects for I2c
SoftWire sw[] = {SoftWire(SDA_PIN[0],SCL_PIN[0]), SoftWire(SDA_PIN[1],SCL_PIN[1]), SoftWire(SDA_PIN[2],SCL_PIN[2]), SoftWire(SDA_PIN[3],SCL_PIN[3])}; //Wire objects for I2C

char swTxBuffer[nbrSoftMpu][64]; //Buffers for softReading, as big as the biggest reading/writing
char swRxBuffer[nbrSoftMpu][64];

//Hardware I2c
MPU9250 Hmpu[nbrHardMpu];

//Setting variable
MPU9250Setting setting;

//Store data from MPU
float qX[nbrMpu] = {0};
float qY[nbrMpu] = {0};
float qZ[nbrMpu] = {0};
float qW[nbrMpu] = {0};

float oX[nbrMpu] = {0};
float oY[nbrMpu] = {0};
float oZ[nbrMpu] = {0};

//-------WIFI SETTINGS AND FUNCTIONS-------
//Settings to connect to WiFi
#define WIFI_SSID "ArtNet4Hans"
#define WIFI_PASS "kaesimira"
#define LED_BUILTIN 2

WiFiUDP Udp;
IPAddress outIp(192,168,0,2); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP

OSCMessage gyroQ[nbrMpu] = {OSCMessage("/gyro0/quater"), OSCMessage("/gyro1/quater"),OSCMessage("/gyro2/quater"),OSCMessage("/gyro3/quater"),OSCMessage("/gyro4/quater"),OSCMessage("/gyro5/quater")};
OSCMessage gyroA[nbrMpu] = {OSCMessage("/gyro0/angle"), OSCMessage("/gyro1/angle"),OSCMessage("/gyro2/angle"),OSCMessage("/gyro3/angle"),OSCMessage("/gyro4/angle"),OSCMessage("/gyro5/angle")};

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
  Serial.flush(); //Clean buffer

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp();

  //-------MPU SETUP------
  //Software I2C
  for(int i=0; i<nbrSoftMpu; i++) {
    sw[i].setTxBuffer(swTxBuffer[i], sizeof(swTxBuffer[i]));//Initialisze buffers and connections
    sw[i].setRxBuffer(swRxBuffer[i], sizeof(swRxBuffer[i]));
    sw[i].setDelay_us(5);
    sw[i].begin();//Initialize I2C comm
  }

  //Hardware I2c
  Wire.begin(SDA_PIN[4],SCL_PIN[4]);
  Wire1.begin(SDA_PIN[5], SCL_PIN[5]);
  delay(2000);

  //MPU parameters (sensitivity, etc)
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  //Lauch communication with the 6 mpus
  for(int i=0; i<nbrSoftMpu; i++) {
    Smpu[i].setup(MPU_ADDRESS_1, setting, sw[i]);
  }
  Hmpu[0].setup(MPU_ADDRESS_1, setting, Wire);
  Hmpu[1].setup(MPU_ADDRESS_1, setting, Wire1);

  //Selection of filters
  /*QuatFilterSel sel{QuatFilterSel::MADGWICK};
  mpu1.selectFilter(sel);
  mpu1.setFilterIterations(10);*/
  


  /*Serial.println("Calibration of acceleration : don't move devices");
  //mpu1.calibrateAccelGyro();
  Serial.println("Acceleration calibration done.");

  Serial.println("Calibration of mag 1");
  mpu1.setMagneticDeclination(2.53);
  //mpu1.calibrateMag();
  Serial.println("Mag calibration done.");

  mpu1.setFilterIterations(10);
  QuatFilterSel sel{QuatFilterSel::MADGWICK};
  mpu1.selectFilter(sel);*/


}

void loop() {
  //--------MPU recording--------
  
  //Read mpu data
  Smpu[0].update();
  Smpu[1].update();
  Smpu[2].update();
  Smpu[3].update();
  Hmpu[0].update();
  Hmpu[1].update();

  //Print values for debugging
  static unsigned long last_print=0;
  if (millis()-last_print > 100) {

        Serial.print(Smpu[0].getEulerX()); Serial.print("// ");
        Serial.print(Smpu[1].getEulerX()); Serial.print("// ");
        Serial.print(Smpu[2].getEulerX()); Serial.print("// ");
        Serial.print(Smpu[3].getEulerX()); Serial.print("// ");
        Serial.print(Hmpu[0].getEulerX()); Serial.print("// ");
        Serial.print(Hmpu[1].getEulerX()); Serial.print("// ");

        Serial.println();

        last_print=millis();
  }

  //Store values
  for(int i=0; i<nbrSoftMpu; i++) {
    qX[i] = Smpu[i].getQuaternionX();
    qY[i] = Smpu[i].getQuaternionY();
    qZ[i] = Smpu[i].getQuaternionZ();
    qW[i] = Smpu[i].getQuaternionW();

    oX[i] = Smpu[i].getEulerX();
    oY[i] = Smpu[i].getEulerY();
    oZ[i] = Smpu[i].getEulerZ();
  }

  for(int i=0; i<nbrHardMpu; i++) {
    qX[i] = Hmpu[i].getQuaternionX();
    qY[i] = Hmpu[i].getQuaternionY();
    qZ[i] = Hmpu[i].getQuaternionZ();
    qW[i] = Hmpu[i].getQuaternionW();

    oX[i] = Hmpu[i].getEulerX();
    oY[i] = Hmpu[i].getEulerY();
    oZ[i] = Hmpu[i].getEulerZ();
  }

  //-------OSC communication--------


  for(int i=0; i<nbrHardMpu; i++) {
    gyroQ[i].add(qX[i]).add(qY[i]).add(qZ[i]).add(qW[i]); //Fill OSC message with data
    gyroA[i].add(oX[i]).add(oY[i]).add(oZ[i]);

    Udp.beginPacket(outIp, outPort); //intitializes packet transmission -- by giving the IP and the port = UDP way to communicate
    gyroQ[i].send(Udp); //sends the message
    Udp.endPacket(); //Finish the packet

    Udp.beginPacket(outIp, outPort);
    gyroA[i].send(Udp);
    Udp.endPacket();

    gyroQ[i].empty(); //Empty the OSC message
    gyroA[i].empty();
  }


}