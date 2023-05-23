#include "Imu.hpp"
#define AHRS_DEBUG_OUTPUT
Imu::Imu(){}
void Imu::setup (const char * oscName, Adafruit_Sensor_Calibration* calibration){+
    strcpy(this->oscName, oscName);
    this->calibration= calibration;
    configureSensor();
    oscMessageMutex.lock(); // keep "Wifi send task" from accessing data while it is written
    oscMessage.init(oscName,10);
    oscMessageMutex.unlock();
    filter.begin(23); // update drewuency guess 
}

void Imu::update(){
    //time keeping for sensor filter integration
    uint32_t timestamp = micros();
    uint32_t microsSinceLastUpdate=timestamp-lastUpdateMicros;
    lastUpdateMicros=timestamp;
    float deltaT= (float)microsSinceLastUpdate/1000000.0f;

    sensor.getEvent(&accel_event, &gyro_event,&temp_event, &mag_event);
#if defined(AHRS_DEBUG_OUTPUT)
  // Serial.print("I2C took "); Serial.print(micros()-timestamp); Serial.println(" mus");
#endif

    calibration->calibrate(accel_event);
    calibration->calibrate(gyro_event);
    calibration->calibrate(temp_event);
    calibration->calibrate(mag_event);
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro_event.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro_event.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro_event.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, 
        accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z, 
        mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z,deltaT);
#if defined(AHRS_DEBUG_OUTPUT)
   // Serial.print("Update took "); Serial.print(micros()-timestamp); Serial.println("mus");
#endif

  // put new values into the osc Message buffer
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();

    float qw, qx, qy, qz;   
    filter.getQuaternion(&qw, &qx, &qy, &qz);

    oscMessageMutex.lock(); // keep "Wifi send task" from accessing data while it is written
    oscMessage.setData(qw, qx, qy, qz,roll,pitch,yaw,gx,gy,gz);
    //oscMessage.setData(1, 2, 3, 4,5,6,7,8,9,10);
    oscMessageMutex.unlock();
}

void Imu::printSerial(){
  Serial.print("Timestamp: ");Serial.println(accel_event.timestamp);

  Serial.print("Raw: ");
  Serial.print(accel_event.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel_event.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel_event.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag_event.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag_event.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag_event.magnetic.z, 4); Serial.println("");
    
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

void Imu::sendOsc(WiFiUDP& Udp,IPAddress& receiverIp,int receiverPort){

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

    Udp.beginPacket(receiverIp, receiverPort);
    oscMessageMutex.lock(); 
    Udp.write((uint8_t*)oscMessage.buffer,oscMessage.messageLength);
    oscMessageMutex.unlock(); 
    Udp.endPacket();
    
}


void Imu::configureSensor()
{

    Serial.println("Connecting to Sensor...");
    sensor.begin_I2C(); // let the library set uop its internal I2C business.

    lastErrorState = sensorConfigFailed;
    // Use a "raw I2" connect to get some more meaningful error codes
    Wire.beginTransmission(ICM_ADDRESS);
    int result = Wire.endTransmission();
    switch (result) {
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

  void Imu::sendMotionCal() {
        sensor.getEvent(&accel_event, &gyro_event,&temp_event, &mag_event);
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