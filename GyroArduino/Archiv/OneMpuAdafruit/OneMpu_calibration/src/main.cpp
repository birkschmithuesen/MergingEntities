//Libraries
#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Preferences.h>


#define TCA_ADDRESS 0x70

//Variables
Preferences preferences;
char mpuPref[10];
int calibDone = 0;

#define nbrMpu 1

//IMU variables
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

Adafruit_Sensor_Calibration_EEPROM cal;
sensors_event_t mag_event, gyro_event, accel_event;

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

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

byte caldata[68]; // buffer to receive magnetic calibration data
byte calcount=0;

void receiveCalibration(int nImu) {
  uint16_t crc;
  byte b, i;

  while (Serial.available()) {
    b = Serial.read();
    if (calcount == 0 && b != 117) {
      // first byte must be 117
      return;
    }
    if (calcount == 1 && b != 84) {
      // second byte must be 84
      calcount = 0;
      return;
    }
    // store this byte
    caldata[calcount++] = b;
    if (calcount < 68) {
      // full calibration message is 68 bytes
      return;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i=0; i < 68; i++) {
      crc = crc16_update(crc, caldata[i]);
    }

    //We record the data - means first calib over
    if (crc == 0) {
      // data looks good, use it
      float offsets[16] = {0};
      memcpy(offsets, caldata+2, 16*4);

      //We store the calib data
      itoa(nImu,mpuPref,10); //Key names = number of mpu
      preferences.begin(mpuPref, false);

      preferences.putFloat("maghard0", offsets[6]);
      preferences.putFloat("maghard1", offsets[7]);
      preferences.putFloat("maghard2", offsets[8]);

      preferences.putFloat("magfield", offsets[9]);

      preferences.putFloat("magsoft0",offsets[10]);
      preferences.putFloat("magsoft0",offsets[13]);
      preferences.putFloat("magsoft0",offsets[14]);
      preferences.putFloat("magsoft0",offsets[13]);
      preferences.putFloat("magsoft0",offsets[11]);
      preferences.putFloat("magsoft0",offsets[15]);
      preferences.putFloat("magsoft0",offsets[14]);
      preferences.putFloat("magsoft0",offsets[15]);
      preferences.putFloat("magsoft0",offsets[12]);

      preferences.end();
      calibDone++;

    }
    // look for the 117,84 in the data, before discarding
    for (i=2; i < 67; i++) {
      if (caldata[i] == 117 && caldata[i+1] == 84) {
        // found possible start within data
        calcount = 68 - i;
        memmove(caldata, caldata + i, calcount);
        return;
      }
    }
    // look for 117 in last byte
    if (caldata[67] == 117) {
      caldata[0] = 117;
      calcount = 1;
    } else {
      calcount = 0;
    }
  }
}

void calibrate_mag(int channel){
  TCA(channel);
  while(!init_sensors()){
    Serial.println("No Sensor");
    delay(500);}

  while(calibDone != 1){
    magnetometer->getEvent(&mag_event);
    gyroscope->getEvent(&gyro_event);
    accelerometer->getEvent(&accel_event);
  
    // 'Raw' values to match expectation of MotionCal
    Serial.print("Raw:");
    Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
    Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
    Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.x*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.y*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.z*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(mag_event.magnetic.x*10)); Serial.print(",");
    Serial.print(int(mag_event.magnetic.y*10)); Serial.print(",");
    Serial.print(int(mag_event.magnetic.z*10)); Serial.println("");

    // unified data
    Serial.print("Uni:");
    Serial.print(accel_event.acceleration.x); Serial.print(",");
    Serial.print(accel_event.acceleration.y); Serial.print(",");
    Serial.print(accel_event.acceleration.z); Serial.print(",");
    Serial.print(gyro_event.gyro.x, 4); Serial.print(",");
    Serial.print(gyro_event.gyro.y, 4); Serial.print(",");
    Serial.print(gyro_event.gyro.z, 4); Serial.print(",");
    Serial.print(mag_event.magnetic.x); Serial.print(",");
    Serial.print(mag_event.magnetic.y); Serial.print(",");
    Serial.print(mag_event.magnetic.z); Serial.println("");

    //Wait for calibration data of MotionCal
    receiveCalibration(channel);
  }

  calibDone = 0;
}




void setup() {
  Serial.begin(115200);
      // will pause Zero, Leonardo, etc until serial console opens
  
  //Wire.setClock(400000); // 400KHz
  TCA(0);
  check();
  for(int i=0; i<nbrMpu; i++){
    
    //calibrate_mag(i);
  }
  
  
}

void loop() {

  int v = 0;
  if(v == 1){
    itoa(0,mpuPref,10);
    preferences.begin(mpuPref, false);

    //Set acceleration & gyro calibration data
    Serial.println(preferences.getFloat("maghard0", 0));
    Serial.println(preferences.getFloat("maghard1", 0));
    Serial.println(preferences.getFloat("maghard2", 0));

    preferences.end();
  }
 
  delay(1000); 
}