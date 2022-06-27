//Connections : Please always connect 2 hard mpu (builtin I2C bus) to your specified pins
//Indicate the numbers of hard, soft I2C, connect the soft I2C in the order of the specified pins
//Specify your IP address and Wifi
//Mag calibration desactivated right now, see if it's usefull
//Eventually speed up if TCA is fast enough

//-------LIBRARIES-------
//Library to use Arduino cmd
#include <Arduino.h>

//MPU and Bitbang library
#include "MPU9250.h"

//-------MPU SETTINGS AND FUNCTIONS-------
//Parameters of the setup

#define nbrMpu 1

//Addresses and pin of MPU and TCA9548A(=multiplexer)
#define MPU_ADDRESS_1 0x68 //Set 0x68 or 0x69

#define TCA_ADDRESS 0x70


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

//-------SETUP-------
void setup() {
  Serial.begin(115200);
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

  //Lauch communication with the mpu

  //TCA(0);
  check();
  mpu[0].setup(MPU_ADDRESS_1, setting, Wire);
  //TCA(1);
  //check();
  //mpu[1].setup(MPU_ADDRESS_1, setting, Wire);

  delay(1000);

}

void loop() {
  //--------MPU recording--------
  
  //Read mpu data
  
  //TCA(0);
  mpu[0].update();
  //TCA(1);
  //mpu[1].update();

  //Print values for debugging
  
  Serial.print(mpu[0].getEulerX());
  Serial.print("//");
  //Serial.print(mpu[1].getEulerX());

  Serial.println();


  //Store values
  
  /*oX[0] = mpu1.getEulerX();
  oY[0] = mpu1.getEulerY();
  oZ[0] = mpu1.getEulerZ();

  oX[1] = mpu2.getEulerX();
  oY[1] = mpu2.getEulerY();
  oZ[1] = mpu2.getEulerZ();*/
}
