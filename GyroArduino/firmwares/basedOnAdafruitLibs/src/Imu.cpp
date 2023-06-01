#include "Imu.hpp"
#define AHRS_DEBUG_OUTPUT
Imu::Imu() {}
void Imu::setup(const char *oscName, Adafruit_Sensor_Calibration *calibration)
{
  strcpy(this->oscName, oscName);
  this->calibration = calibration;
  configureSensor();
  oscMessageMutex.lock(); // keep "Wifi send task" from accessing data while it is written
  oscMessage.init(oscName, 17);
  oscMessageMutex.unlock();
  filter.begin(78); // update frequency guess
  filter.setBeta(0.5);
}
// read sensor only, don't do any processing
void Imu::readSensor(){
  // time keeping for sensor filter integration


  sensor.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

#if defined(AHRS_DEBUG_OUTPUT)
  // Serial.print("I2C took "); Serial.print(micros()-timestamp); Serial.println(" mus");
#endif

}
// process last data set read from Sensor (altering values in place)
void Imu::processCurrentData(){
  uint32_t timestamp = micros();
  uint32_t microsSinceLastUpdate = timestamp - lastUpdateMicros;
  lastUpdateMicros = timestamp;
  float deltaT = (float)microsSinceLastUpdate / 1000000.0f;
//constrain deltaT to keep the filter stable 
if(deltaT<0.0001)deltaT=0.0001;
if(deltaT>0.1)deltaT=0.1;

  calibration->calibrate(accel_event);
  calibration->calibrate(gyro_event);
  calibration->calibrate(temp_event);
  calibration->calibrate(mag_event);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx =  gyro_event.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro_event.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro_event.gyro.z * SENSORS_RADS_TO_DPS;
///
/*
        float an = -a[0];
        float ae = +a[1];
        float ad = +a[2];
        float gn = +g[0] * DEG_TO_RAD;
        float ge = -g[1] * DEG_TO_RAD;
        float gd = -g[2] * DEG_TO_RAD;
        float mn = +m[1];
        float me = -m[0];
        float md = +m[2];
        
         filter.update(gx, -gy, -gz,
                -accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mag_event.magnetic.y, -mag_event.magnetic.x, mag_event.magnetic.z, deltaT);
                */
        ///
  // Update the SensorFusion filter
  
    filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mag_event.magnetic.x, -mag_event.magnetic.y, -mag_event.magnetic.z, deltaT);

          


#if defined(AHRS_DEBUG_OUTPUT)
  // Serial.print("Update took "); Serial.print(micros()-timestamp); Serial.println("mus");
#endif


}
// put current values into osc message
void Imu::prepareOscMessage(){

  // put new values into the osc Message buffer
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  oscMessageMutex.lock(); // keep "Wifi send task" from accessing data while it is written
  oscMessage.setFloat(0,qw);
  oscMessage.setFloat(1, qx);
  oscMessage.setFloat(2, qy);
  oscMessage.setFloat(3, qz);
  oscMessage.setFloat(4, roll);
  oscMessage.setFloat(5, pitch);
  oscMessage.setFloat(6, yaw);
  oscMessage.setFloat(7, gx);
  oscMessage.setFloat(8, gy);
  oscMessage.setFloat(9, gz);

  oscMessage.setFloat(10, accel_event.acceleration.x);
  oscMessage.setFloat(11, accel_event.acceleration.y);
  oscMessage.setFloat(12, accel_event.acceleration.z);

  oscMessage.setFloat(13, mag_event.magnetic.x);
  oscMessage.setFloat(14,  mag_event.magnetic.y);
  oscMessage.setFloat(15,  mag_event.magnetic.z);


  oscMessage.hasBeenSent=false;
  oscMessageMutex.unlock();

}

void Imu::update()
{
  
  checkReconnect();
  readSensor();
  processCurrentData();
  prepareOscMessage();
}

void Imu::printSerial()
{
  //Serial.print("Timestamp: ");
  //Serial.println(accel_event.timestamp);

  Serial.print("accel: ");
  Serial.print(accel_event.acceleration.x, 4);
  Serial.print(", ");
  Serial.print(accel_event.acceleration.y, 4);
  Serial.print(", ");
  Serial.print(accel_event.acceleration.z, 4);
  Serial.print("\tgyro: ");
  Serial.print(gyro_event.gyro.x, 4);
  Serial.print(", ");
  Serial.print(gyro_event.gyro.y, 4);
  Serial.print(", ");
  Serial.print(gyro_event.gyro.z, 4);
  Serial.print("\tmag: "); 
  Serial.print(mag_event.magnetic.x, 4);
  Serial.print(", ");
  Serial.print(mag_event.magnetic.y, 4);
  Serial.print(", ");
  Serial.print(mag_event.magnetic.z, 4);
  Serial.print("\t");

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Quaternion: ");
  Serial.print(qw, 4);
  Serial.print(", ");
  Serial.print(qx, 4);
  Serial.print(", ");
  Serial.print(qy, 4);
  Serial.print(", ");
  Serial.println(qz, 4);
}

void Imu::sendOsc(WiFiUDP &Udp, IPAddress &receiverIp, int receiverPort)
{

  // for comparison: the "old way" of building osc messages
  /*
  OSCMessage message(oscName);
      // set the quaternion data

  message.add(qx)
      .add(qy)
      .add(qz)
      .add(qw);
  // set the euler angle data
  message.add(roll)
      .add(pitch)
      .add(yaw);
  // set the gyro data
  message.add(gx).add(gy).add(gz);
  Udp.beginPacket(receiverIp, receiverPort);
  message.send(Udp);
  Udp.endPacket();
  */

  oscMessageMutex.lock();
  if (!oscMessage.hasBeenSent)
  {
    Udp.beginPacket(receiverIp, receiverPort);
    oscMessage.setFloat(16, debugPackageIndex);
    debugPackageIndex++;
    Udp.write((uint8_t *)oscMessage.buffer, oscMessage.messageLength);
    oscMessage.hasBeenSent = true;
    oscMessageMutex.unlock();
    Udp.endPacket();
  }
  else
  {
    oscMessageMutex.unlock();
  }
}
// if the controller has forgotten one of its settings, reinitialize it.
void Imu::checkReconnect()
{
  if(updatesSinceLastReconnectCheck<50){
    updatesSinceLastReconnectCheck++;
    return;
  }
  updatesSinceLastReconnectCheck=0;
  if (reconnectCount < 1000)
  {
    // try to detect sensor reconnect by checking if mag data rate is the same that we set.
    if (ICM20948_ACCEL_RANGE_4_G != sensor.getAccelRange())
    {
      Serial.print("Sensor ");
      Serial.print(oscName);
      Serial.println("is not functional - trying to revive it.");
      
      // check if there is any connection at all
      Wire.beginTransmission(ICM_ADDRESS);
      int result = Wire.endTransmission();
      if (result == ESP_OK)
      {
        configureSensor();
        reconnectCount++;
      }else{
        Serial.print("Could not reach it by I2C Error Code");
        Serial.println(result);
      }
          
    }

  }
}

void Imu::configureSensor()
{

  Serial.println("Connecting to Sensor...");
  sensor.begin_I2C(); // let the library set uop its internal I2C business.

  lastErrorState = sensorConfigFailed;
  // Use a "raw I2" connect to get some more meaningful error codes
  Wire.beginTransmission(ICM_ADDRESS);
  int result = Wire.endTransmission();
  switch (result)
  {
  case 0:
    Serial.println(".. worked");
    break;
  case 2:
    Serial.println(".. failed (error 2) maybe not connected?");
    return;
    break;
  case 5:
    Serial.println(".. failed (I2C bus timeout)");
    return;
    break;
  default:
    Serial.print(".. failed (error ");
    Serial.print(result);
    Serial.println(")");
    return;

    break;
  }

  // accel range +/- 4g
  sensor.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  if (ICM20948_ACCEL_RANGE_4_G != sensor.getAccelRange())
  {
    lastErrorCode = 4;
    Serial.println("Failed: ICM20948_ACCEL_RANGE_4_G");
    return;
  }
  // gyro 500 degree/s;
  sensor.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  if (ICM20948_GYRO_RANGE_500_DPS != sensor.getGyroRange())
  {
    lastErrorCode = 5;
    Serial.println("Failed: ICM20948_GYRO_RANGE_500_DPS");
    return;
  }
  // highest data rate (MPU9250 fifo rate 125 Hz)
  if (!sensor.setMagDataRate(AK09916_MAG_DATARATE_100_HZ))
  {
    lastErrorCode = 6;
    Serial.println("Failed: AK09916_MAG_DATARATE_100_HZ");
    return;
  }
  if (AK09916_MAG_DATARATE_100_HZ != sensor.getMagDataRate())
  {
    lastErrorCode = 7;
    Serial.println("Failed: AK09916_MAG_DATARATE_100_HZ");
    return;
  }
  lastErrorState = sensorConfigSuccess;
}

void Imu::sendMotionCal()
{
  sensor.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

if(false){
  // as a check - send calibrated values, they should come out in motioncal as "perfect"
  calibration->calibrate(accel_event);
  calibration->calibrate(gyro_event);
  calibration->calibrate(temp_event);
  calibration->calibrate(mag_event);
}

  // raw data format
  Serial.print("Raw:");
  Serial.print(int(this->accel_event.acceleration.x * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(this->accel_event.acceleration.y * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(this->accel_event.acceleration.z * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(this->gyro_event.gyro.x * SENSORS_RADS_TO_DPS * 16));
  Serial.print(",");
  Serial.print(int(this->gyro_event.gyro.y * SENSORS_RADS_TO_DPS * 16));
  Serial.print(",");
  Serial.print(int(this->gyro_event.gyro.z * SENSORS_RADS_TO_DPS * 16));
  Serial.print(",");
  Serial.print(int(this->mag_event.magnetic.x * 10));
  Serial.print(",");
  Serial.print(int(this->mag_event.magnetic.y * 10));
  Serial.print(",");
  Serial.print(int(this->mag_event.magnetic.z * 10));
  Serial.println("");
  // unified data format
  Serial.print("Uni:");
  Serial.print(this->accel_event.acceleration.x);
  Serial.print(",");
  Serial.print(this->accel_event.acceleration.y);
  Serial.print(",");
  Serial.print(this->accel_event.acceleration.z);
  Serial.print(",");
  Serial.print(this->gyro_event.gyro.x, 4);
  Serial.print(",");
  Serial.print(this->gyro_event.gyro.y, 4);
  Serial.print(",");
  Serial.print(this->gyro_event.gyro.z, 4);
  Serial.print(",");
  Serial.print(this->mag_event.magnetic.x);
  Serial.print(",");
  Serial.print(this->mag_event.magnetic.y);
  Serial.print(",");
  Serial.print(this->mag_event.magnetic.z);
  Serial.println("");
}