// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>



#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define TCA_ADDRESS 0x70 /**< I2C address of the TCA9548A (I2C multiplexer) */

Adafruit_ICM20948 sensor = Adafruit_ICM20948();
// SDA and SCL pin of the soft and hard wire mode
#define SDA_PIN 22       /**< I2C data pin (on ESP32) */
#define SCL_PIN 21       /**< I2C clock pin (on ESP32) */
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;


// pick your filter! slower == better quality output
//Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 1
#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();





  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  }
  //{1.009117,-0.011831,0.000662,-0.011831,0.985828,0.007041,0.000662,0.007041,1.005404},//softIron
cal.mag_softiron[0]=1.009117;
cal.mag_softiron[1]=-0.011831;
cal.mag_softiron[2]=0.000662;
cal.mag_softiron[3]=-0.011831;
cal.mag_softiron[4]=0.985828;

cal.mag_softiron[5]=0.007041;
cal.mag_softiron[6]=0.000662;
cal.mag_softiron[7]=0.007041;
cal.mag_softiron[8]=1.005404;
//{-36.121483,-24.739384,6.435102},//hardIron
cal.mag_hardiron[0]=-36.121483;
cal.mag_hardiron[1]=-24.739384;
cal.mag_hardiron[2]=6.435102;

//{0.000608,0.001241,-0.001649},//gyrobias
cal.gyro_zerorate[0]=0.000608;
cal.gyro_zerorate[1]=0.001241;
cal.gyro_zerorate[1]=-0.001649;
//48.263016//magfield
cal.mag_field=48.263016;

  Serial.println("Connecting to Sensor...");

        // set up I2C communication
    Serial.print("setting up I2C pins .");

    Wire.begin(SDA_PIN, SCL_PIN,(uint32_t)400000);
    Wire.setTimeOut(1); // timeout in milllis
        // select the multiplexer by its hardware address
    Wire.beginTransmission(TCA_ADDRESS);
    // select a channel on the multiplexer
    int channel=0;
    Wire.write(1 << channel);
    Wire.endTransmission();


  sensor.begin_I2C(); // let the library set uop its internal I2C business.

 accelerometer = sensor.getAccelerometerSensor();
  gyroscope = sensor.getGyroSensor();
  magnetometer = sensor.getMagnetometerSensor();
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  sensor.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  if (ICM20948_ACCEL_RANGE_4_G != sensor.getAccelRange())
  {
    Serial.println("Failed: ICM20948_ACCEL_RANGE_4_G");
    return;
  }
  // gyro 500 degree/s;
  sensor.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  if (ICM20948_GYRO_RANGE_500_DPS != sensor.getGyroRange())
  {
    Serial.println("Failed: ICM20948_GYRO_RANGE_500_DPS");
    return;
  }
  // highest data rate (MPU9250 fifo rate 125 Hz)
  if (!sensor.setMagDataRate(AK09916_MAG_DATARATE_100_HZ))
  {
    Serial.println("Failed: AK09916_MAG_DATARATE_100_HZ");
    return;
  }
  if (AK09916_MAG_DATARATE_100_HZ != sensor.getMagDataRate())
  {
    Serial.println("Failed: AK09916_MAG_DATARATE_100_HZ");
    return;
  }



  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
}


void loop() {
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
#if defined(AHRS_DEBUG_OUTPUT)
 // Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter

  //!!!!!!!! ATTENTION: magnetic y and z axes need to be reversed for ICM20948
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x,-mag.magnetic.y, -mag.magnetic.z);
#if defined(AHRS_DEBUG_OUTPUT)
 // Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); 
#endif

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);
/*
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
  */
#if defined(AHRS_DEBUG_OUTPUT)
  //Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif
}
