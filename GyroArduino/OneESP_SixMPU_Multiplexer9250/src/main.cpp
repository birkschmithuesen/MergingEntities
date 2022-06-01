//Connections : Please always connect 2 hard mpu (builtin I2C bus) to your specified pins
//Indicate the numbers of hard, soft I2C, connect the soft I2C in the order of the specified pins
//Specify your IP address and Wifi
//Mag calibration desactivated right now, see if it's usefull
//Eventually speed up if TCA is fast enough

//Attention : select the right channel with the TCA function (+2 woth prototype in wood, i regular without)

//-------LIBRARIES-------
//Library to use Arduino cmd
#include <Arduino.h>

//Libraries for Comm
#include <WiFi.h>
#include <OSCMessage.h>
#include <WiFiUdp.h>

//MPU and Bitbang library
#include "MPU9250.h"

//-------GENERAL SETTINGS-------

//Select network to connnect

#define KAESIMIRA
//#define THEATER

//Define or not define PROTO1 to adapt the code to the old prototype : it just change the channel number
//#define PROTO1

//Define the number of the body : 1, 2 or 3
#define BODY_1

//-------END GENERAL SETTINGS-------

#ifdef PROTO1
#define CHANNEL i+2
#endif

#ifndef PROTO1
#define CHANNEL i
#endif
//-------MPU SETTINGS AND FUNCTIONS-------
//Parameters of the setup

//Test function to see mpu
void check()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");

      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");

      Serial.println(address,HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");

  delay(5000); // wait 5 seconds for next scan
}

#define nbrMpu 6

//Addresses and pin of MPU and TCA9548A(=multiplexer)
#define MPU_ADDRESS_1 0x68 //Set 0x68 or 0x69
#define MPU_ADDRESS_2 0x69 //Set 0x68 or 0x69

#define TCA_ADDRESS 0x70

//SDA and SCL pin of the soft and hard wire mode
int SDA_PIN = 21;
int SCL_PIN = 22;

//LED pin for info showing

int RED_PIN = 32;
int YEL_PIN = 33;
int state = HIGH;

//Hardware I2c
MPU9250 mpu[nbrMpu];

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

//Calibration data for automatic calibration :

float magbiais[6][3] = {{19.45692,176.272,-354.0199},
{-57.16423,-83.95995,6.887821},
{195.9925,77.82183,-227.1107},
{385.3314,224.4164,-140.9016},
{-42.7326,-89.02625,-84.08881},
{14.19734,95.83208,-402.2347}};

float magscale[6][3] = {{1.017677,0.9950617,0.987745},
{1.013836,1.025445,0.9629629},
{1.009987,0.980606,1.009987},
{1.02594,0.9987373,0.9765432},
{1.011236,0.9782609,1.011236},
{0.9856115,0.9927536,1.022388}};

//Function to switch the channel on the multiplexer
void TCA(uint8_t channel){
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

//-------WIFI SETTINGS AND FUNCTIONS-------
//Settings to connect to WiFi

#ifdef KAESIMIRA
#define WIFI_SSID "ArtNet4Hans"
#define WIFI_PASS "kaesimira"
IPAddress outIp(192,168,0,2); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP
#endif

#ifdef THEATER
#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"
IPAddress outIp(192,168,193,221); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP*/
#endif

WiFiUDP Udp;

#ifdef BODY_1
OSCMessage body[] = {OSCMessage ("/body/1/gyro/1/"), OSCMessage ("/body/1/gyro/2/"), OSCMessage ("/body/1/gyro/3/"), OSCMessage ("/body/1/gyro/4/"), OSCMessage ("/body/1/gyro/5/"), OSCMessage ("/body/1/gyro/6/")};
#endif

#ifdef BODY_2
OSCMessage body[] = {OSCMessage ("/body/2/gyro/1/"), OSCMessage ("/body/2/gyro/2/"), OSCMessage ("/body/2/gyro/3/"), OSCMessage ("/body/2/gyro/4/"), OSCMessage ("/body/2/gyro/5/"), OSCMessage ("/body/2/gyro/6/")};
#endif

#ifdef BODY_3
OSCMessage body[] = {OSCMessage ("/body/3/gyro/1/"), OSCMessage ("/body/3/gyro/2/"), OSCMessage ("/body/3/gyro/3/"), OSCMessage ("/body/3/gyro/4/"), OSCMessage ("/body/3/gyro/5/"), OSCMessage ("/body/3/gyro/6/")};
#endif


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

  //Led initialization

  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp();

  //-------MPU SETUP------
  //Launch comm with multiplexer

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(2000);
  Serial.println("Connection to multiplexer OK");

  //MPU parameters (sensitivity, etc)
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  //Lauch communication with the 6 mpus - Switch to channel i and lauch comm with mpu numer i
  for(int i=0; i<nbrMpu; i++) {
    TCA(CHANNEL);
    //check();
    mpu[i].setup(MPU_ADDRESS_1, setting, Wire);
  }

  //Selection of filters
  
  QuatFilterSel sel{QuatFilterSel::MADGWICK};
  for(int i=0; i<nbrMpu; i++) {
    TCA(CHANNEL);
    mpu[i].selectFilter(sel);
    mpu[i].setFilterIterations(10);
  }
  
  // ------- CALIBRATION -------
  
  //Calibration of acceleration
  //Print in Serial / Send to osc that the accel calibration begins
  Serial.println("Calibration of acceleration : don't move devices");

  calibration.add("Calibration of acceleration : don't move devices");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  digitalWrite(RED_PIN, HIGH);
  for(int i=0; i<nbrMpu; i++) {
    Serial.println(i);
    TCA(CHANNEL);
    mpu[i].calibrateAccelGyro();
  }
  digitalWrite(RED_PIN, LOW);
  Serial.println("Acceleration calibration done.");

  //Calibration of magnetometer

  //Manual calibration : get ready to turn the mpu
  /*
  Serial.println("Calibration of mag");

  for(int i=0; i<nbrMpu; i++) {
    if (state == HIGH){
      digitalWrite(YEL_PIN, state);
      state = LOW;
    }
    else{
      digitalWrite(YEL_PIN, state);
      state = HIGH;
    }
    calibration.add("Calibration of ").add(i+1);

    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();

    Serial.println(i);

    TCA(CHANNEL);
    mpu[i].setMagneticDeclination(2.53);
    mpu[i].calibrateMag();

    calibration.empty();
    }
    */

  //Automatic calibration : With data of the stage. Respect the gyro wiring
  calibration.add("Automatic calibration of the magnetometer gyro");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  for(int i=0; i<nbrMpu; i++) {
    mpu[i].setMagneticDeclination(2.53);
    mpu[i].setMagBias(magbiais[i][0], magbiais[i][1], magbiais[i][2]);
    mpu[i].setMagScale(magscale[i][0], magscale[i][1], magscale[i][2]);
  }
  Serial.println("Mag calibration done.");
  //Blinking yel light= calib over
  digitalWrite(YEL_PIN, LOW);
  delay(200);
  digitalWrite(YEL_PIN, HIGH);
  delay(200);
  digitalWrite(YEL_PIN, LOW);

  /*
  //Print/send the calibration of magnetometer - USE IT TO AUTO CALIB AGAIN 
  Serial.println("Calibration data :");
  for(int i=0; i<nbrMpu; i++) {
    TCA(CHANNEL);

    //Send calibration data to TouchDesigner
    calibration.add("MAGSCALE").add(i).add(mpu[i].getMagScaleX()).add(mpu[i].getMagScaleY()).add(mpu[i].getMagScaleZ());
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();

    calibration.add("MAGBIAIS").add(i).add(mpu[i].getMagBiasX()).add(mpu[i].getMagBiasY()).add(mpu[i].getMagBiasZ());
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();
  }*/

}

void loop() {

  //--------MPU recording--------
  
  //Read mpu data
  for(int i=0; i<nbrMpu; i++) {
    TCA(CHANNEL);
    mpu[i].update();
  }

  //Print values for debugging
  /*
  static unsigned long last_print=0;
  if (millis()-last_print > 100) {

        for(int i=0;i<nbrMpu;i++){
          Serial.print(mpu[i].getRoll()); Serial.print("// ");
        }
      
        Serial.println();

        last_print=millis();
  }*/

  //Store values
  
  for(int i=0; i<nbrMpu; i++) {
    qX[i] = mpu[i].getQuaternionX();
    qY[i] = mpu[i].getQuaternionY();
    qZ[i] = mpu[i].getQuaternionZ();
    qW[i] = mpu[i].getQuaternionW();

    oX[i] = mpu[i].getEulerX();
    oY[i] = mpu[i].getEulerY();
    oZ[i] = mpu[i].getEulerZ();

    gX[i] = mpu[i].getGyroX();
    gY[i] = mpu[i].getGyroY();
    gZ[i] = mpu[i].getGyroZ();
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
    body[i].add(qX[i]).add(qY[i]).add(qZ[i]).add(qW[i]); //Fill OSC message with data
    body[i].add(oX[i]).add(oY[i]).add(oZ[i]);
    body[i].add(gX[i]).add(gY[i]).add(gZ[i]);

    Udp.beginPacket(outIp, outPort);
    body[i].send(Udp);
    Udp.endPacket();

    body[i].empty();
  }


}