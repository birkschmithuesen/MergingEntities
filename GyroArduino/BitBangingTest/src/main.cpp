//Library to use Arduino cmd
#include <Arduino.h>
#include <SoftWire.h>

//MPU library
#include "MPU9250.h"

//Parameters of the setup
#define nbrSoftMpu 4
#define nbrHardMpu 2

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

void setup() {
  Serial.begin(115200);
  Serial.flush(); //Clean buffer

  //Software I2C
  for(int i=0; i<nbrSoftMpu; i++) {
    sw[i].setTxBuffer(swTxBuffer[i], sizeof(swTxBuffer[i]));//Initialisze buffers and connections
    sw[i].setRxBuffer(swRxBuffer[i], sizeof(swRxBuffer[i]));
    sw[i].setDelay_us(5);
    sw[i].begin();
  }

  //Hardware I2c
  Wire.begin(SDA_PIN[4],SCL_PIN[4]);
  Wire1.begin(SDA_PIN[5], SCL_PIN[5]);


  delay(2000);

  //MPU parameters (sensitivity, etc)
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  //Launch mpu setup
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
}