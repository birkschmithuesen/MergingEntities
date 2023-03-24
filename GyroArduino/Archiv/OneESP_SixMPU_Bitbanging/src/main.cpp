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

//MPU and Bitbang library
#include "MPU9250.h"
#include <SoftWire.h>

//-------MPU SETTINGS-------
//Parameters of the setup
#define nbrSoftMpu 0
#define nbrHardMpu 1

#define nbrMpu nbrSoftMpu + nbrHardMpu

//Addresses and pin of MPU's
#define MPU_ADDRESS_1 0x68 //Set 0x68 or 0x69
#define MPU_ADDRESS_2 0x69 //Set 0x68 or 0x69

//Number of the suit where this code is flashed - 1,2 or 3
#define BODY_ADDRESS "/body/1/"

//SDA and SCL pin of the soft and hard wire mode
int SSDA_PIN[] = {16, 18, 5, 26}; //16 = RX2, 26 and 27 should't be used with Wifi ..., add a couple for 6th I2C
int SSCL_PIN[] = {17, 19, 23, 27}; //17 = TX2

int HSDA_PIN[] = {21,32}; //Careful to these pins and your own pins !
int HSCL_PIN[] = {22,33};

//Software I2c
MPU9250_<SoftWire> Smpu[nbrSoftMpu]; //Mpu objects for soft I2c - MAX 4 SOFT I2C
SoftWire sw[] = {SoftWire(SSDA_PIN[0],SSCL_PIN[0]), SoftWire(SSDA_PIN[1],SSCL_PIN[1]), SoftWire(SSDA_PIN[2],SSCL_PIN[2]), SoftWire(SSDA_PIN[3],SSCL_PIN[3])}; //Wire objects for I2C, add a SoftWire for 6 I2C

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

float gX[nbrMpu] = {0};
float gY[nbrMpu] = {0};
float gZ[nbrMpu] = {0};

//-------WIFI SETTINGS AND FUNCTIONS-------
//Settings to connect to WiFi
#define WIFI_SSID "ArtNet4Hans"
#define WIFI_PASS "kaesimira"
#define LED_BUILTIN 2
IPAddress outIp(192,168,0,2); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP

//Settings to other WiFi
/*#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"
IPAddress outIp(192,168,193,221); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP*/


WiFiUDP Udp;


OSCMessage body(BODY_ADDRESS);
OSCMessage body1[] = {OSCMessage ("/body/1/gyro/1/"), OSCMessage ("/body/1/gyro/2/"), OSCMessage ("/body/1/gyro/3/"), OSCMessage ("/body/1/gyro/4/"), OSCMessage ("/body/1/gyro/5/"), OSCMessage ("/body/1/gyro/6/")};
OSCMessage calibration("/calibration");

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
    sw[i].setTxBuffer(swTxBuffer[i], sizeof(swTxBuffer[i]));//Initialize buffers and connections
    sw[i].setRxBuffer(swRxBuffer[i], sizeof(swRxBuffer[i]));
    sw[i].setDelay_us(5);
    sw[i].begin();//Initialize I2C comm
  }

  //Hardware I2c
  Wire.begin(HSDA_PIN[0],HSCL_PIN[0]); //The two last pin, change to 4,5 for 6 I2C
  Wire1.begin(HSDA_PIN[1], HSCL_PIN[1]);
  delay(2000);

  //MPU parameters (sensitivity, etc)
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
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
  
  QuatFilterSel sel{QuatFilterSel::MADGWICK};
  for(int i=0; i<nbrSoftMpu; i++) {
    Smpu[i].selectFilter(sel);
    Smpu[i].setFilterIterations(10);
  }
  for(int i=0; i<nbrHardMpu; i++) {
    Hmpu[i].selectFilter(sel);
    Hmpu[i].setFilterIterations(10);
  }
  
  //Calibration of acceleration and magnetic offsets
  
  Serial.println("Calibration of acceleration : don't move devices");
  for(int i=0; i<nbrSoftMpu; i++) {
    Smpu[i].calibrateAccelGyro();
  }
  for(int i=0; i<nbrHardMpu; i++) {
    Hmpu[i].calibrateAccelGyro();
  }
  Serial.println("Acceleration calibration done.");

  Serial.println("Calibration of mag");

  for(int i=0; i<nbrSoftMpu; i++) {
    calibration.add("Calibration of ").add(i+1);

    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();

    Smpu[i].setMagneticDeclination(2.53);
    Smpu[i].calibrateMag();



    calibration.empty();
  }
  for(int i=0; i<nbrHardMpu; i++) {
    calibration.add("Calibration of ").add(i+5);

    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();

    Hmpu[i].setMagneticDeclination(2.53);
    Hmpu[i].calibrateMag();

    calibration.empty();
  }
  Serial.println("Mag calibration done.");

}

void loop() {
  //--------MPU recording--------
  
  //Read mpu data
  for(int i=0; i<nbrSoftMpu; i++) {
    Smpu[i].update();
  }

  Hmpu[0].update();
  Hmpu[1].update();

  //Print values for debugging
  
  static unsigned long last_print=0;
  if (millis()-last_print > 100) {

        for(int i=0;i<nbrMpu;i++) {
          Serial.print(oX[i]); Serial.print("// ");
        }

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

    gX[i] = Smpu[i].getGyroX();
    gY[i] = Smpu[i].getGyroY();
    gZ[i] = Smpu[i].getGyroZ();
  }

  for(int i=0; i<nbrHardMpu; i++) {
    qX[i+nbrSoftMpu] = Hmpu[i].getQuaternionX();
    qY[i+nbrSoftMpu] = Hmpu[i].getQuaternionY();
    qZ[i+nbrSoftMpu] = Hmpu[i].getQuaternionZ();
    qW[i+nbrSoftMpu] = Hmpu[i].getQuaternionW();

    oX[i+nbrSoftMpu] = Hmpu[i].getEulerX();
    oY[i+nbrSoftMpu] = Hmpu[i].getEulerY();
    oZ[i+nbrSoftMpu] = Hmpu[i].getEulerZ();

    gX[i+nbrSoftMpu] = Hmpu[i].getGyroX();
    gY[i+nbrSoftMpu] = Hmpu[i].getGyroY();
    gZ[i+nbrSoftMpu] = Hmpu[i].getGyroZ();
  }



  //-------OSC communication--------

  //Send all the data in one OSCMessage - Work "okay"
  /*
  for(int i=0; i<nbrMpu; i++) {
    body.add(qX[i]).add(qY[i]).add(qZ[i]).add(qW[i]); //Fill OSC message with data
    body.add(oX[i]).add(oY[i]).add(oZ[i]);
    body.add(gX[i]).add(gY[i]).add(gZ[i]);

  }

  Udp.beginPacket(outIp, outPort);
  body.send(Udp);
  Udp.endPacket();

  body.empty(); //Empty the OSC message
  */

 //Send data in a 6 separated messages, working okay
  for(int i=0; i<nbrMpu; i++) {
    body1[i].add(qX[i]).add(qY[i]).add(qZ[i]).add(qW[i]); //Fill OSC message with data
    body1[i].add(oX[i]).add(oY[i]).add(oZ[i]);
    body1[i].add(gX[i]).add(gY[i]).add(gZ[i]);

    Udp.beginPacket(outIp, outPort);
    body1[i].send(Udp);
    Udp.endPacket();

    body1[i].empty();
  }


}