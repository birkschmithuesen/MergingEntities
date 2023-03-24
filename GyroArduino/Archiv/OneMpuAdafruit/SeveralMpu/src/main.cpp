#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

//General settings
#define nbrMpu 6

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
  cal[i].calibrate(mag[i]); 
  cal[i].calibrate(accel[i]);
  cal[i].calibrate(gyro[i]);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gX[i] = gyro[i].gyro.x * SENSORS_RADS_TO_DPS;
  gY[i] = gyro[i].gyro.y * SENSORS_RADS_TO_DPS;
  gZ[i] = gyro[i].gyro.z * SENSORS_RADS_TO_DPS;


  // Update the SensorFusion filter - QUATERNION
  filter[i].update(gX[i], gY[i], gZ[i], 
                  accel[i].acceleration.x, accel[i].acceleration.y, accel[i].acceleration.z, 
                  mag[i].magnetic.x, mag[i].magnetic.y, mag[i].magnetic.z);
  filter[i].getQuaternion(&qW[i], &qX[i], &qY[i], &qZ[i]);
}

//Function to calibrate the gyro
/*
void calibrateGyro(int i) {
  //LOAD


  //OR CREATE NEW DATA

  float mg[3] = {0};

  mg[0] += gyro[i].gyro.x;
  mg[1] += gyro[i].gyro.y;
  mg[2] += gyro[i].gyro.z;  
  c++;

  if(c == 500) {
  Serial.print(mg[0]/500);
  Serial.print("//");
  Serial.print(mg[1]/500);
  Serial.print("//");
  Serial.print(mg[2]/500);
  Serial.print(" -- ");
  Serial.print(ma[0]/500);
  Serial.print("//");
  Serial.print(ma[1]/500);
  Serial.print("//");
  Serial.print(ma[2]/500);
  }
}*/

void setup() {
    Serial.begin(115200);
    Serial.flush();
    delay(1000);
    
    //Initialize all gyros
    for(int i=0; i<nbrMpu; i++){
    TCA(i);
    if(!init_sensors(i)){
      Serial.print("No ");
    } 
    else{
      Serial.print("Yes ");
    }
    //Fill accelerometer, gyro and mag of the data to us them - MULTIPLEXER
    filter[i].begin(FILTER_UPDATE_RATE_HZ);
    delay(5000);
    }


    //Calibrate : get data from ESP + store them in these tabs + cal.softiron, cal.mag, ... - RUSHY, TO CHANGE
    for(int i=0; i<nbrMpu; i++){
    cal[i].accel_zerog[0] = 0;
    cal[i].accel_zerog[1] = 0;
    cal[i].accel_zerog[2] = 0;

    cal[i].gyro_zerorate[0] = 0;
    cal[i].gyro_zerorate[1] = 0;
    cal[i].gyro_zerorate[2] = 0;
    }
  timestamp = millis();
  Wire.setClock(400000); // 400KHz

}

void loop() {
  
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();

  for(int i=0; i<nbrMpu;i++){
    //Update data
    update(i);

    //------- PRINT / SEND DATA -------

    oX[i] = filter[i].getRoll();
    oY[i] = filter[i].getPitch();
    oZ[i] = filter[i].getYaw();

    
    Serial.print(oX[i]);
    Serial.print("//");
    Serial.print(oY[i]);
    Serial.print("//");
    Serial.print(oZ[i]);
    Serial.print(" --- ");
/*
    Serial.print(accel[i].acceleration.x);
    Serial.print("//");
    Serial.print(accel[i].acceleration.y);
    Serial.print("//");
    Serial.print(accel[i].acceleration.z);
    Serial.print(" --- ");*/

    }

    Serial.println();
  }

