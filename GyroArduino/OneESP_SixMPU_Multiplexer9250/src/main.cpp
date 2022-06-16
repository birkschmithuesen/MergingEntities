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
#include <Preferences.h>
//MPU and Bitbang library
#include "MPU9250.h"

//-------GENERAL SETTINGS-------
#define nbrMpu 6
//Select network to connnect
#define KAESIMIRA
//#define THEATER

//Define the number of the body : 1, 2 or 3
#define BODY_2

//Set the magnetic declination of the light
#define MAG_DECLINATION 2.53

//Define the calibration mode : MANUAL_CALIBRATION for manual, AUTO_CALIBRATION otherwise
#define AUTO_CALIBRATION

//Define BUTTON to activate the button
#define BUTTON

//-------END GENERAL SETTINGS-------

//-------MPU SETTINGS AND FUNCTIONS-------
//Parameters of the setup

//Addresses and pin of MPU and TCA9548A(=multiplexer)
#define MPU_ADDRESS_1 0x68 //Set 0x68 or 0x69
#define MPU_ADDRESS_2 0x69 //Set 0x68 or 0x69

#define TCA_ADDRESS 0x70


//SDA and SCL pin of the soft and hard wire mode
#define SDA_PIN 21
#define SCL_PIN 22

//LED pin for info showing, BUTTON pin for communication

#define RED_PIN 32
#define YEL_PIN 33
#define BUTTON_PIN 5

#define MPU_NORTH 1 //Mpu used to set the north
float theta = 0; //angle to the north
float time_converge = 0;

int state = HIGH;
int state_button = LOW;

//Instance to store data on ESP32, name of the preference
Preferences preferences;
char mpuPref[10];

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

float accbias[6][3];
float gyrobias[6][3];

//Function to switch the channel on the multiplexer
void TCA(uint8_t channel){
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

//Function to see mpu connected
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

//-------WIFI SETTINGS AND FUNCTIONS-------
//Settings to connect to WiFi

WiFiUDP Udp;

#ifdef KAESIMIRA
#define WIFI_SSID "ArtNet4Hans"
#define WIFI_PASS "kaesimira"
IPAddress outIp(192,168,0,2); //IP of the computer
int localPort = 8888; //Port of ESP
#endif

#ifdef THEATER
#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"
IPAddress outIp(192,168,193,221); //IP of the computer
int localPort = 8888; //Port of ESP*/
#endif

#ifdef BODY_1
int outPort = 8000; //Port on PC
OSCMessage body[] = {OSCMessage ("/body/1/gyro/1/"), OSCMessage ("/body/1/gyro/2/"), OSCMessage ("/body/1/gyro/3/"), OSCMessage ("/body/1/gyro/4/"), OSCMessage ("/body/1/gyro/5/"), OSCMessage ("/body/1/gyro/6/")};
OSCMessage calibration("/calibration/1");
//Mag calibrtion data for A
float magscale[6][3] = {{1.01,1.06,0.94},
{1.01,1.00,0.99},
{1.04,0.98,0.98},
{1.06,0.97,0.98},
{0.99,1.00,1.01},
{1.02,1.03,0.95}};

float magbias[6][3] = {{-40.82,-108.61,-405.33},
{128.62,104.29,-164.14},
{103.32,249.40,-116.79},
{-15.87,157.95,-46.02},
{-3.55,46.14,-403.94},
{-17.57,327.23,-390.66}};
#endif

#ifdef BODY_2
int outPort = 8001; //Port on PC
OSCMessage body[] = {OSCMessage ("/body/2/gyro/1/"), OSCMessage ("/body/2/gyro/2/"), OSCMessage ("/body/2/gyro/3/"), OSCMessage ("/body/2/gyro/4/"), OSCMessage ("/body/2/gyro/5/"), OSCMessage ("/body/2/gyro/6/")};
OSCMessage calibration("/calibration/2");
float magscale[6][3] = {{0.99,1.01,1.00},
{0.98,1.00,1.02},
{0.98,1.03,0.98},
{1.03,0.99,0.99},
{1.02,0.99,1.00},
{1.01,0.98,1.01}};

float magbias[6][3] = {{98.40,-5.27,-345.30},
{399.67,242.51,-126.99},
{-48.23,-92.89,67.16},
{8.90,-89.03,-82.37},
{5.31,188.74,-324.95},
{183.41,101.35,-152.56}};
#endif

#ifdef BODY_3
int outPort = 8002; //Port on PC
OSCMessage body[] = {OSCMessage ("/body/3/gyro/1/"), OSCMessage ("/body/3/gyro/2/"), OSCMessage ("/body/3/gyro/3/"), OSCMessage ("/body/3/gyro/4/"), OSCMessage ("/body/3/gyro/5/"), OSCMessage ("/body/3/gyro/6/")};
OSCMessage calibration("/calibration/3");
#endif

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
  //-------HARDWARE SETUP-------
  Serial.begin(115200);
  Serial.flush(); //Clean buffer

  //Led initialization
  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp();

  //-------MPU SETUP------
  //Launch comm with multiplexer

  Wire.begin(SDA_PIN, SCL_PIN);
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

  //Lauch communication with the 6 mpus - Switch to channel i and lauch comm with mpu numer i
  for(int i=0; i<nbrMpu; i++) {
    TCA(i);
    //check();
    mpu[i].setup(MPU_ADDRESS_1, setting, Wire);
  }

  //Selection of filters
  QuatFilterSel sel{QuatFilterSel::MADGWICK};
  for(int i=0; i<nbrMpu; i++) {
    mpu[i].selectFilter(sel);
    mpu[i].setFilterIterations(10);
  }
  
  // ------- CALIBRATION AND SET THE NORTH-------
  //CURRENT PROCESS WITHOUT BUTTON CHOICE

  #ifndef BUTTON
  //Calibration of acceleration
  Serial.println("Calibration of acceleration : don't move devices");
  calibration.add("Calibration of acceleration : don't move devices");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  digitalWrite(RED_PIN, HIGH);//Calibrate one by one
  for(int i=0; i<nbrMpu; i++) {
    Serial.println(i);
    TCA(i);
    mpu[i].calibrateAccelGyro();
  }
  digitalWrite(RED_PIN, LOW);
  Serial.println("Acceleration calibration done.");

  //Calibration of magnetometer

  //Manual calibration : get ready to turn the mpu
  #ifdef MANUAL_CALIBRATION
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

    TCA(i);
    mpu[i].setMagneticDeclination(MAG_DECLINATION);
    mpu[i].calibrateMag();

    calibration.empty();
    }
  #endif

  #ifdef AUTO_CALIBRATION
  //Automatic calibration : With data of the stage. Respect the gyro wiring
  calibration.add("Automatic calibration of the magnetometer gyro");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  for(int i=0; i<nbrMpu; i++) {
    mpu[i].setMagneticDeclination(MAG_DECLINATION);
    mpu[i].setMagBias(magbias[i][0], magbias[i][1], magbias[i][2]);
    mpu[i].setMagScale(magscale[i][0], magscale[i][1], magscale[i][2]);
  }
  Serial.println("Mag calibration done.");
  #endif
  //Blinking light= calib over
  digitalWrite(YEL_PIN, LOW);
  delay(200);
  digitalWrite(YEL_PIN, HIGH);
  delay(200);
  digitalWrite(YEL_PIN, LOW);
  #endif

  #ifdef BUTTON
  //-------FIRST CHOICE-------Two leds are lighted : you have 6 seconds to press or not to press the button, to launch a calibration process
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, HIGH);
  delay(6000);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  state_button = digitalRead(BUTTON_PIN);
  //state_button = LOW;
  if(state_button == HIGH){
    Serial.println("HIGH : Launch calibration sequence");
    //Acceleration : get data calibration + calibrate
    Serial.println("Calibration of acceleration : don't move devices");
    calibration.add("Calibration of acceleration : don't move devices");
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();

    digitalWrite(RED_PIN, HIGH);
    for(int i=0; i<nbrMpu; i++) {
      Serial.println(i);
      TCA(i);
      mpu[i].calibrateAccelGyro();
    }
    digitalWrite(RED_PIN, LOW);
    Serial.println("Acceleration calibration done.");

    // Acceleration : store calibration data
    for(int i=0; i<nbrMpu; i++){
      itoa(i,mpuPref,10); //Key names = number of mpu
      preferences.begin(mpuPref, false);

      preferences.putFloat("accbiasX", mpu[i].getAccBiasX());
      preferences.putFloat("accbiasY", mpu[i].getAccBiasY());
      preferences.putFloat("accbiasZ", mpu[i].getAccBiasZ());

      preferences.putFloat("gyrobiasX", mpu[i].getGyroBiasX());
      preferences.putFloat("gyrobiasY", mpu[i].getGyroBiasY());
      preferences.putFloat("gyrobiasZ", mpu[i].getGyroBiasZ());

      preferences.end();
    }

    //Magnetometer : get data calibration + calibrate
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

      TCA(i);
      mpu[i].setMagneticDeclination(MAG_DECLINATION);
      mpu[i].calibrateMag();

      calibration.empty();
      
    }
    Serial.println("Calibration of mag done");

    //Magnetometer : store calibration data
    for(int i=0; i<nbrMpu; i++){
      itoa(i,mpuPref,10); //Key names = number of mpu
      preferences.begin(mpuPref, false);

      preferences.putFloat("magbiasX", mpu[i].getMagBiasX());
      preferences.putFloat("magbiasY", mpu[i].getMagBiasY());
      preferences.putFloat("magbiasZ", mpu[i].getMagBiasZ());

      preferences.putFloat("magscaleX", mpu[i].getMagScaleX());
      preferences.putFloat("magscaleY", mpu[i].getMagScaleY());
      preferences.putFloat("magscaleZ", mpu[i].getMagScaleZ());

      preferences.end();
    }
  }

  //Button not pushed : we read the stored calibration data and calibrate
  else {
    Serial.println("LOW : Load calibration data");
    for(int i=0; i<nbrMpu; i++){
      itoa(i,mpuPref,10); //Key names = number of mpu
      preferences.begin(mpuPref, false);

      //Set acceleration calibration data
      
      mpu[i].setAccBias(preferences.getFloat("accbiasX", 0),preferences.getFloat("accbiasY", 0),preferences.getFloat("accbiasZ", 0));
      mpu[i].setGyroBias(preferences.getFloat("gyrobiasX", 0),preferences.getFloat("gyrobiasY", 0),preferences.getFloat("gyrobiasZ", 0));

      //Test to see what the fuck
      /*
      digitalWrite(RED_PIN, HIGH);
      for(int i=0; i<nbrMpu; i++) {
        Serial.println(i);
        TCA(i);
        mpu[i].calibrateAccelGyro();
      }
      digitalWrite(RED_PIN, LOW);
      Serial.println("Acceleration calibration done.");*/

      //Set magnetometer calibration data
      mpu[i].setMagBias(preferences.getFloat("magbiasX", 0),preferences.getFloat("magbiasY", 0),preferences.getFloat("magbiasZ", 0));
      mpu[i].setMagScale(preferences.getFloat("magscaleX", 0),preferences.getFloat("magscaleY", 0),preferences.getFloat("magscaleZ", 0));
      mpu[i].setMagneticDeclination(MAG_DECLINATION);
      preferences.end();
    }
    Serial.println("Calibration loaded");    
  }  

  //Two leds are blinking, saying calibration is over
  for(int i=0; i<20; i++){
    digitalWrite(RED_PIN, state);
    digitalWrite(YEL_PIN, state);
    if(state==HIGH){state = LOW;}
    else{state = HIGH;}
    delay(200);
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  //-------SECOND CHOICE------- Two leds are lighted : you have 10 seconds to orientate the MPU 1 over the new north and press the button
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, HIGH);

  //We let the mpu run for 10 seconds to have the good yaw if we need it
  TCA(MPU_NORTH-1);

  time_converge = millis();
  while(millis() - time_converge < 10000){
    mpu[MPU_NORTH-1].update();
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  state_button = digitalRead(BUTTON_PIN);
  //state_button = HIGH;
  if(state_button == HIGH){
    Serial.println("HIGH : setting north");
    //We save the north direction and send it to the library
    theta = mpu[MPU_NORTH-1].getYaw() * (-1);
    
    preferences.begin("setNorth", false);
    preferences.putFloat("north", theta);
    preferences.end();
  }
  else{
    Serial.println("LOW : Load former north");
  }

  //We get the north and set it
  preferences.begin("setNorth", false);    
  for(int i=0;i<nbrMpu;i++){
    mpu[i].setNorth(preferences.getFloat("north",0));
  }
  preferences.end();
  Serial.println("North set");

  //Two leds are blinking, saying north is set
  for(int i=0; i<20; i++){
    digitalWrite(RED_PIN, state);
    digitalWrite(YEL_PIN, state);
    if(state==HIGH){state = LOW;}
    else{state = HIGH;}
    delay(200);
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);
  #endif


  /*
  //Print/send the calibration of magnetometer - USE IT TO AUTO CALIB AGAIN 
  Serial.println("Calibration data :");
  for(int i=0; i<nbrMpu; i++) {
    TCA(i);

    //Send calibration data to TouchDesigner
    calibration.add("MAGSCALE").add(i).add(mpu[i].getMagScaleX()).add(mpu[i].getMagScaleY()).add(mpu[i].getMagScaleZ());
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();

    calibration.add("MAGBIAS").add(i).add(mpu[i].getMagBiasX()).add(mpu[i].getMagBiasY()).add(mpu[i].getMagBiasZ());
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
    TCA(i);
    mpu[i].update();
  }

  //Print values for debugging
  
  static unsigned long last_print=0;
  if (millis()-last_print > 100) {

        for(int i=0;i<nbrMpu;i++){
          
          Serial.print(mpu[i].getYaw()); Serial.print("// ");
          Serial.print(mpu[i].getYaw_r()); Serial.print("// ");
          Serial.print(mpu[i].getNorth()); Serial.print("// ");
/*
          preferences.begin("0", false);
          Serial.print(preferences.getFloat("accbiasX", 0)); Serial.print("/");
          Serial.print(mpu[i].getAccBiasX()); Serial.print("_");
          Serial.print(preferences.getFloat("accbiasY", 0)); Serial.print("/");
          Serial.print(mpu[i].getAccBiasY()); Serial.print("_");
          Serial.print(preferences.getFloat("accbiasZ", 0)); Serial.print("/");
          Serial.print(mpu[i].getAccBiasZ()); Serial.print("_");
          preferences.end();*/
/*        
          Serial.print(preferences.getFloat("magbiasX", 0)); Serial.print("/");
          Serial.print(mpu[i].getMagBiasX()); Serial.print("_");
          Serial.print(preferences.getFloat("magbiasY", 0)); Serial.print("/");
          Serial.print(mpu[i].getMagBiasY()); Serial.print("_");
          Serial.print(preferences.getFloat("magbiasZ", 0)); Serial.print("/");
          Serial.print(mpu[i].getMagBiasZ()); Serial.print("_");

          

          Serial.print(preferences.getFloat("magscaleX", 0)); Serial.print("/");
          Serial.print(mpu[i].getMagScaleX()); Serial.print("_");
          Serial.print(preferences.getFloat("magscaleY", 0)); Serial.print("/");
          Serial.print(mpu[i].getMagScaleY()); Serial.print("_");
          Serial.print(preferences.getFloat("magscaleZ", 0)); Serial.print("/");
          Serial.print(mpu[i].getMagScaleZ()); Serial.print("_");

          Serial.print(preferences.getFloat("gyrobiasX", 0)); Serial.print("/");
          Serial.print(mpu[i].getGyroBiasX()); Serial.print("_");
          Serial.print(preferences.getFloat("gyrobiasY", 0)); Serial.print("/");
          Serial.print(mpu[i].getGyroBiasY()); Serial.print("_");
          Serial.print(preferences.getFloat("gyrobiasZ", 0)); Serial.print("/");
          Serial.print(mpu[i].getGyroBiasZ()); Serial.print("_");

          */
          /*
          Serial.print(mpu[i].getQuaternionW()); Serial.print("// ");
          Serial.print(mpu[i].getQuaternionX()); Serial.print("// ");
          Serial.print(mpu[i].getQuaternionY()); Serial.print("// ");
          Serial.print(mpu[i].getQuaternionZ()); Serial.print("// ");*/

          /*
          Serial.print(mpu[i].getMagScaleX());
          Serial.print("/");
          Serial.print(mpu[i].getMagScaleY());
          Serial.print("/");
          Serial.print(mpu[i].getMagScaleZ());

          Serial.print("__");

          Serial.print(mpu[i].getMagBiasX());
          Serial.print("/");
          Serial.print(mpu[i].getMagBiasY());
          Serial.print("/");
          Serial.print(mpu[i].getMagBiasZ());
          Serial.print("/");

          Serial.println();*/
        }
      
        Serial.println();

        last_print=millis();
  }

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