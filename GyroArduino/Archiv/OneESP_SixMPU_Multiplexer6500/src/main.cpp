//Connections : Please always connect 2 hard mpu (builtin I2C bus) to your specified pins
//Indicate the numbers of hard, soft I2C, connect the soft I2C in the order of the specified pins
//Specify your IP address and Wifi
//Mag calibration desactivated right now, see if it's usefull
//Eventually speed up if TCA is fast enough

//-------LIBRARIES-------
//Library to use Arduino cmd
#include <Arduino.h>

//Libraries for Comm
#include <WiFi.h>
#include <OSCMessage.h>
#include <WiFiUdp.h>

//Modified MPU6500  library
#include "MPU9250.h"

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

//Number of the suit where this code is flashed - 1,2 or 3
#define BODY_ADDRESS "/body/1/"

//SDA and SCL pin of the soft and hard wire mode
int SDA_PIN = 21;
int SCL_PIN = 22;

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

//Function to switch the channel on the multiplexer
void TCA(uint8_t channel){
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

//-------WIFI SETTINGS AND FUNCTIONS-------

#define LED_BUILTIN 2
WiFiUDP Udp;

//Settings to connect to WiFi
/*#define WIFI_SSID "ArtNet4Hans"
#define WIFI_PASS "kaesimira"
IPAddress outIp(192,168,0,2); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP*/

//Settings to other WiFi
#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"
IPAddress outIp(192,168,193,221); //IP of the computer
int outPort = 8000; //Port on PC
int localPort = 8888; //Port of ESP



OSCMessage body(BODY_ADDRESS);
OSCMessage body1[] = {OSCMessage ("/body/1/gyro/1/"), OSCMessage ("/body/1/gyro/2/"), OSCMessage ("/body/1/gyro/3/"), OSCMessage ("/body/1/gyro/4/"), OSCMessage ("/body/1/gyro/5/"), OSCMessage ("/body/1/gyro/6/")};

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
  //Launch comm with multiplexer

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(2000);
  Serial.println("Connection to multiplexer OK");

  //MPU parameters (sensitivity, etc)
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  //Lauch communication with the 6 mpus - Switch to channel i and lauch comm with mpu numer i
  for(int i=0; i<nbrMpu; i++) {
    TCA(i+2);
    check(); //Check list of visile i2c devices
    mpu[i].setup(MPU_ADDRESS_1, setting, Wire);
  }

  //Calibration of acceleration
  
  Serial.println("Calibration of acceleration : don't move devices.");
  for(int i=0; i<nbrMpu; i++) {
    TCA(i+2);
    Serial.print("Calibration of one mpu ");
    Serial.println(i);
    mpu[i].calibrateAccelGyro();
    
  }
  Serial.println("Acceleration calibration done.");

}

void loop() {
  //--------MPU recording--------
  
  //Read mpu data
  for(int i=0; i<nbrMpu; i++) {
    TCA(i+2);
    mpu[i].update();
  }

  //Print values for debugging
  

  for(int i=0; i<nbrMpu; i++){
    Serial.print(mpu[i].getEulerX()); Serial.print("// ");
    //Serial.print(mpu[i].getEulerY()); Serial.print("// ");
    //Serial.print(mpu[i].getEulerZ()); Serial.print(" -- ");
  }

  Serial.println();


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

  //Send all the data in one OSCMessage - Work "okay" - SHOULD NOT BE USEF ANYMORE - ONLY FOR DEBUGGING
  

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