//Library to use Arduino cmd
#include <Arduino.h>
#include "MPU9250.h"

MPU9250 mpu;
MPU9250Setting setting1;

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
