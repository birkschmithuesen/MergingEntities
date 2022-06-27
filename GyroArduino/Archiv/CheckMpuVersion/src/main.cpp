#include <Arduino.h>
#include "quaternionFilters.h"
#include "MPU9250.h"

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

#define TCA_ADDRESS 0x70

 void TCA(uint8_t channel){
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}



MPU9250 myIMU0(MPU9250_ADDRESS_AD0, I2Cport, I2Cclock);
MPU9250 myIMU1(MPU9250_ADDRESS_AD1, I2Cport, I2Cclock);

 byte c = 0x00;
 byte d = 0x00;
 bool ledOn = true;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){};
  //TCA(0);
  pinMode(13, OUTPUT);
  
}
void scan()
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
  #define adress 0c
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

  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");        // wait 5 seconds for next scan
}
void loop() {
  // put your main code here, to run repeatedly:
  
  c = myIMU0.readByte(MPU9250_ADDRESS_AD0, WHO_AM_I_MPU9250);
  d = myIMU1.readByte(MPU9250_ADDRESS_AD1, WHO_AM_I_MPU9250);

  Serial.print("Received AD0: 0x");
  Serial.println(c, HEX);
  

  //scan();
  delay(500);

}