#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

#include <Preferences.h>
//Variables
Preferences preferences;
char mpuPref[10];
int calibDone = 0;
int state = LOW;

//General settings
#define nbrMpu 6

#define RED_PIN 32
#define YEL_PIN 33
#define BUTTON_PIN 5

#define TCA_ADDRESS 0x70

//Mpu settings
#define FILTER_UPDATE_RATE_HZ 80


//Mpu data containers
Adafruit_Sensor *accelerometer[nbrMpu], *gyroscope[nbrMpu], *magnetometer[nbrMpu];
#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout //We declare it here to allow the .h to use the previous variables

Adafruit_NXPSensorFusion filter[nbrMpu];

Adafruit_Sensor_Calibration_EEPROM cal[nbrMpu];

sensors_event_t accel[nbrMpu], gyro[nbrMpu], mag[nbrMpu];

float qW[nbrMpu], qX[nbrMpu], qY[nbrMpu], qZ[nbrMpu];
float oX[nbrMpu], oY[nbrMpu], oZ[nbrMpu];
float gX[nbrMpu], gY[nbrMpu], gZ[nbrMpu];

float mg[6][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
float ma[6][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

//Others

uint32_t timestamp;

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

//Function to update one imu
void update(int i){
  TCA(i);
  accelerometer[i]->getEvent(&accel[i]);
  gyroscope[i]->getEvent(&gyro[i]);
  magnetometer[i]->getEvent(&mag[i]);

  //Substrat the offset - CALIBRATION 
  /*
  cal[i].calibrate(mag[i]); 
  cal[i].calibrate(accel[i]);
  cal[i].calibrate(gyro[i]);*/

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  /*
  gX[i] = gyro[i].gyro.x * SENSORS_RADS_TO_DPS;
  gY[i] = gyro[i].gyro.y * SENSORS_RADS_TO_DPS;
  gZ[i] = gyro[i].gyro.z * SENSORS_RADS_TO_DPS;*/


  // Update the SensorFusion filter - QUATERNION
  /*
  filter[i].update(gX[i], gY[i], gZ[i], 
                  accel[i].acceleration.x, accel[i].acceleration.y, accel[i].acceleration.z, 
                  mag[i].magnetic.x, mag[i].magnetic.y, mag[i].magnetic.z);
  filter[i].getQuaternion(&qW[i], &qX[i], &qY[i], &qZ[i]);*/
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
      //Clear old
      preferences.remove("magsoft0");preferences.remove("magsoft0");preferences.remove("magsoft0");

      preferences.remove("magfield");

     
      preferences.remove("maghard0");preferences.remove("maghard1");preferences.remove("maghard2");
      preferences.remove("maghard3");preferences.remove("maghard4");preferences.remove("maghard5");
      preferences.remove("maghard6");preferences.remove("maghard7");preferences.remove("maghard8");
      //Store
      preferences.putFloat("maghard0", offsets[6]);
      preferences.putFloat("maghard1", offsets[7]);
      preferences.putFloat("maghard2", offsets[8]);

      preferences.putFloat("magfield", offsets[9]);

      preferences.putFloat("magsoft0",offsets[10]);
      preferences.putFloat("magsoft1",offsets[13]);
      preferences.putFloat("magsoft2",offsets[14]);
      preferences.putFloat("magsoft3",offsets[13]);
      preferences.putFloat("magsoft4",offsets[11]);
      preferences.putFloat("magsoft5",offsets[15]);
      preferences.putFloat("magsoft6",offsets[14]);
      preferences.putFloat("magsoft7",offsets[15]);
      preferences.putFloat("magsoft8",offsets[12]);

      preferences.end();
      calibDone = 1;

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
  delay(10);
  TCA(channel);
  delay(10);
  while(!init_sensors(0)){
    Serial.println("No Sensor");
    delay(500);}

  while(calibDone != 1){
    delay(10);
    magnetometer[0]->getEvent(&mag[0]);
    gyroscope[0]->getEvent(&gyro[0]);
    accelerometer[0]->getEvent(&accel[0]);
  
    // 'Raw' values to match expectation of MotionCal
    Serial.print("Raw:");
    Serial.print(int(accel[0].acceleration.x*8192/9.8)); Serial.print(",");
    Serial.print(int(accel[0].acceleration.y*8192/9.8)); Serial.print(",");
    Serial.print(int(accel[0].acceleration.z*8192/9.8)); Serial.print(",");
    Serial.print(int(gyro[0].gyro.x*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(gyro[0].gyro.y*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(gyro[0].gyro.z*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(mag[0].magnetic.x*10)); Serial.print(",");
    Serial.print(int(mag[0].magnetic.y*10)); Serial.print(",");
    Serial.print(int(mag[0].magnetic.z*10)); Serial.println("");

    // unified data
    Serial.print("Uni:");
    Serial.print(accel[0].acceleration.x); Serial.print(",");
    Serial.print(accel[0].acceleration.y); Serial.print(",");
    Serial.print(accel[0].acceleration.z); Serial.print(",");
    Serial.print(gyro[0].gyro.x, 4); Serial.print(",");
    Serial.print(gyro[0].gyro.y, 4); Serial.print(",");
    Serial.print(gyro[0].gyro.z, 4); Serial.print(",");
    Serial.print(mag[0].magnetic.x); Serial.print(",");
    Serial.print(mag[0].magnetic.y); Serial.print(",");
    Serial.print(mag[0].magnetic.z); Serial.println("");

    //Wait for calibration data of MotionCal
    receiveCalibration(channel);
  }

  calibDone = 0;
  Serial.println("Calib gyro OK");
}

void calibrate_gyro(int channel){
  Serial.println("Calib gyro : begin");

  TCA(channel);
  delay(10);
  
  while(!init_sensors(0)){
    Serial.println("No Sensor");
    delay(500);
    }

  for(int i=0; i<100; i++){
    gyroscope[0]->getEvent(&gyro[0]);
    mg[channel][0] += gyro[0].gyro.x;
    mg[channel][1] += gyro[0].gyro.y;
    mg[channel][2] += gyro[0].gyro.z;
    delay(10);
  }
  /*
  mg[channel][0] = mg[channel][0]/100;
  mg[channel][1] = mg[channel][1]/100;
  mg[channel][2] = mg[channel][2]/100;*/

  Serial.println(mg[channel][0]);
  Serial.println(mg[channel][1]);
  Serial.println(mg[channel][2]);


  itoa(channel,mpuPref,10); //Key names = number of mpu
  preferences.begin(mpuPref, false);
  preferences.remove("gyrobiasX");preferences.remove("gyrobiasY");preferences.remove("gyrobiasZ");

  preferences.putChar("gyrobiasX", char(mg[channel][0]));
  preferences.putChar("gyrobiasY", char(mg[channel][1]));
  preferences.putChar("gyrobiasZ", char(mg[channel][2]));


  preferences.end();
  Serial.println("Calib gyro : end");

}

void calibrate_accel(int channel){
  Serial.println("Calib accel : begin");

  TCA(channel);
  delay(10);
  
  while(!init_sensors(0)){
    Serial.println("No Sensor");
    delay(500);
    }

  for(int i=0; i<100; i++){
    accelerometer[0]->getEvent(&accel[0]);
    ma[channel][0] += accel[0].acceleration.x;
    ma[channel][1] += accel[0].acceleration.y;
    ma[channel][2] += accel[0].acceleration.z;
    /*
    Serial.print(accel[0].acceleration.x);
    Serial.print("//");
    Serial.print(accel[0].acceleration.y);
    Serial.print("//");
    Serial.println(accel[0].acceleration.z);
    delay(10);*/
  }
/*
  ma[channel][0] = ma[channel][0]/100;
  ma[channel][1] = ma[channel][1]/100;
  ma[channel][2] = ma[channel][2]/100;*/

  ma[channel][2] -= 981;

  Serial.println(ma[channel][0]);
  Serial.println(ma[channel][1]);
  Serial.println(ma[channel][2]);


  itoa(channel,mpuPref,10); //Key names = number of mpu
  preferences.begin(mpuPref, false);
  preferences.remove("accbiasX");preferences.remove("accbiasY");preferences.remove("accbiasZ");

  preferences.putChar("accbiasX", char(ma[channel][0]));
  preferences.putChar("accbiasY", char(ma[channel][1]));
  preferences.putChar("accbiasZ", char(ma[channel][2]));

  preferences.end();
  Serial.println("Calib accel : end");

}

void setup() {
  delay(3000);

  Serial.begin(115200);

  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  digitalWrite(RED_PIN, HIGH);
  delay(5000);
  digitalWrite(RED_PIN, LOW);

  state = digitalRead(BUTTON_PIN);
    
  //Initialize all gyros
  for(int i=0; i<nbrMpu; i++){

    TCA(i);
    delay(1000);
    Wire.setClock(400000);

    if(!init_sensors(0)){
      digitalWrite(RED_PIN, HIGH);
      delay(100);
      digitalWrite(RED_PIN, LOW);
      delay(100);
      digitalWrite(RED_PIN, HIGH);
      delay(100);
      digitalWrite(RED_PIN, LOW);
      delay(100);
      digitalWrite(RED_PIN, HIGH);
      delay(100);
      digitalWrite(RED_PIN, LOW);
      delay(1000);
      Serial.println("No init sensor");
    }

    //Calib of mag : red blink when over
    if(state == LOW){
      digitalWrite(YEL_PIN, HIGH);
      calibrate_mag(i);
      digitalWrite(YEL_PIN, LOW);
      delay(1000);
    }

    if(state == HIGH){
      //Calib gyro 
      digitalWrite(RED_PIN, HIGH);
      calibrate_gyro(i);
      digitalWrite(RED_PIN, LOW);
      delay(5000);

      //Calib accel
      digitalWrite(YEL_PIN, HIGH);
      calibrate_accel(i);
      digitalWrite(YEL_PIN, LOW);

      delay(5000);
    }
  }

}

void loop() {
  
}

