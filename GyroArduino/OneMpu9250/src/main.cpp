//Library to use Arduino cmd
#include <Arduino.h>
#include "MPU9250.h"

MPU9250 mpu;
MPU9250Setting setting1;

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

void setup() {
    Serial.begin(115200);
    Wire.begin();

  setting1.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting1.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting1.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting1.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting1.gyro_fchoice = 0x03;
  setting1.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting1.accel_fchoice = 0x01;
  setting1.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  check();
  mpu.setup(0x68, setting1, Wire);

  mpu.calibrateAccelGyro();

  Serial.println("Calibration of mag 1");
  mpu.setMagneticDeclination(2.53);
  mpu.calibrateMag();
}

void loop() {

  mpu.update();

  Serial.print(mpu.getYaw());
  Serial.print("//");
  Serial.print(mpu.getPitch());
  Serial.print("//");
  Serial.print(mpu.getRoll());

  Serial.println("");
}
