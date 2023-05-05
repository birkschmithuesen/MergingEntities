/** @file
 * This firmware uses one TCA9548A to read data from 8 ICM20948 sensors
 * via I2C. This data is slightly prepocessed passed on via OSC.
 *
 * @todo implement conversion for quaternions
 * @todo implement conversion for euler angles
 * @todo implement OSC message sending
 */

#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// SDA and SCL pin of the soft and hard wire mode
#define SDA_PIN 21       /**< I2C data pin (on ESP32) */
#define SCL_PIN 22       /**< I2C clock pin (on ESP32) */
#define TCA_ADDRESS 0x70 /**< I2C address of the TCA9548A (I2C multiplexer) */
#define ICM_ADDRESS 0x69 /**< I2C address of the ICM20948 sensor */

#define NUMBER_OF_SENSORS 8 /**< number of ICM20948 sensors */

/**
 * Switch to the given channel on the multiplexer for I2C communication.
 *
 * This function updates the control register in the switch to select one
 * of the eight I2C devices (numbered 0..7) attached to it.
 *
 * @param channel The channel to select to communicate with I2C client
 * @return true if selection was successful, false if not
 * @see setup()
 * @see loop()
 * @todo limit processing to valid values (0..7)
 */
bool selectI2cMultiplexerChannel(uint8_t channel) {
  bool result = false;
  // select the multiplexer by its hardware address
  Wire.beginTransmission(TCA_ADDRESS);
  // select a channel on the multiplexer
  if (1 == Wire.write(1 << channel)) {
    result = true;
  }
  Wire.endTransmission();
  return result;
}

/**
 * A data structure to handle hardware related data and
 * communication of one ICM20948.
 */
struct ICM20948socket {
  const char *label; /**< human readable identification of the sensor (for OSC path) */
  uint8_t channel; /**< channel used on the I2C multiplexer */
  Adafruit_ICM20948 sensor; /**< software handler/abstraction for ICM20948 at given channel */
  bool usable = false; /**< indicate that sensor (data) is present and no errors occured */
  sensors_event_t accel_event; /**< accelerometer information (transmitted via event) */
  sensors_event_t gyro_event;  /**< gyroscope information (transmitted via event) */
  sensors_event_t mag_event;   /**< magnetometer information (transmitted via event) */
  sensors_event_t temp_event;  /**< temperature information (transmitted via event) */

  /**
   * Set up / configure the sensor.
   *
   * @returns true if successful
   * @todo more detailed error codes
   */
  bool setup() {
    if (!selectI2cMultiplexerChannel(this->channel)) {
      return false;
    }
    // accel range +/- 4g
    this->sensor.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
    if (ICM20948_ACCEL_RANGE_4_G != this->sensor.getAccelRange()) {
      return false;
    }
    // gyro 500 degree/s;
    this->sensor.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
    if (ICM20948_GYRO_RANGE_500_DPS != this->sensor.getGyroRange()) {
      return false;
    }

    // highest data rate (MPU9250 fifo rate 125 Hz)
    if (!this->sensor.setMagDataRate(AK09916_MAG_DATARATE_100_HZ)) {
      return false;
    }
    if (AK09916_MAG_DATARATE_100_HZ != this->sensor.getMagDataRate()) {
      return false;
    }
    return true;
  }

  /**
   * Update the internal sensor information.
   *
   * @return false if error occured
   */
  bool update() {
	if (!selectI2cMultiplexerChannel(this->channel)) {
      return false;
    }
    if (this->usable) {
      this->sensor.getEvent(&this->accel_event, &this->gyro_event,
                            &this->temp_event, &this->mag_event);
      return true;
    }
    return false;
  }

  /**
   * @return x axis acceleration of the gyro (in m/s^2)
   */
  float getGyroX() const { return this->gyro_event.gyro.x; }

  /**
   * @return y axis acceleration of the gyro (in m/s^2)
   */
  float getGyroY() const { return this->gyro_event.gyro.y; }

  /**
   * @return z axis acceleration of the gyro (in m/s^2)
   */
  float getGyroZ() const { return this->gyro_event.gyro.z; }

  /**
   * @return euler angle around x axis
   *
   * @todo implementation
   */
  float getEulerX() const { return 0.0f; }

  /**
   * @return euler angle around y axis
   *
   * @todo implementation
   */
  float getEulerY() const { return 0.0f; }

  /**
   * @return euler angle around z axis
   *
   * @todo implementation
   */
  float getEulerZ() const { return 0.0f; }

  float getQuaternionRX() const { return 0.0f; }
  float getQuaternionRY() const { return 0.0f; }
  float getQuaternionRZ() const { return 0.0f; }
  float getQuaternionRW() const { return 0.0f; }
};

ICM20948socket socket[NUMBER_OF_SENSORS]; /**< a (global) list of sockets to bundle communication */


/**
 * Bring the controller into a working state.
 *
 * @see loop()
 * @see selectI2cMultiplexerChannel(uint8_t channel)
 * @see ICM20948socket
 * @todo dedicated setup function for single sensor
 */
void setup(void) {
  uint8_t result = 0;     // track results/error codes
  uint8_t channel_missing = 0; // track number of channels missing
  uint8_t sensors_missing = 0; // track number of missing sensors
  uint8_t sensors_failed = 0; // track number of non-working sensors
  //-------HARDWARE SETUP-------
  Serial.begin(115200);
  // pause until serial line is available
  while (!Serial) {
    delay(10);
  }
  Serial.flush(); // clear serial buffer
  delay(1000);

  // print a nice banner :)
  Serial.println("----------------------------------------");
  Serial.println("  ┌─┐┬─┐┌┬┐┌─┐┌─┐  ┌┬┐┌─┐┌┐ ┬┬  ┌─┐┌─┐");
  Serial.println("  ├─┤├┬┘ │ ├┤ └─┐  ││││ │├┴┐││  ├┤ └─┐");
  Serial.println("  ┴ ┴┴└─ ┴ └─┘└─┘  ┴ ┴└─┘└─┘┴┴─┘└─┘└─┘");
  Serial.println("----------------------------------------");
  Serial.println();
  delay(1000);

  Serial.print("setting up sensor sockets .");
  socket[0].label = "label_1";
  socket[0].channel = 0;
  socket[1].label = "label_2";
  socket[1].channel = 1;
  socket[2].label = "label_3";
  socket[2].channel = 2;
  socket[3].label = "label_4";
  socket[3].channel = 3;
  socket[4].label = "label_5";
  socket[4].channel = 4;
  socket[5].label = "label_6";
  socket[5].channel = 5;
  socket[6].label = "label_7";
  socket[6].channel = 6;
  socket[7].label = "label_8";
  socket[7].channel = 7;
  Serial.println(".. done");

  // inital sanity check
  if (ICM_ADDRESS == TCA_ADDRESS) {
    Serial.println("error: sensor(s) and multiplexer have the same address");
    Serial.println("this leads to communication conflicts");
    Serial.println("... stopping now");
    // put ESP32 into deep sleep (closest to shutdown)
    esp_deep_sleep_start();
  }

  // set up I2C communication
  Serial.print("setting up I2C pins .");
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);
  Serial.println(".. done");

  // see if (expected) multiplexer is there
  Serial.print("searching TCA9548A .");
  Wire.beginTransmission(TCA_ADDRESS);
  result = Wire.endTransmission(true);
  switch (result) {
  case 0:
    Serial.println(".. found");
    break;
  case 5:
    Serial.println(".. failed (I2C bus timeout)");
    Serial.println("... stopping everything");
    // put ESP32 into deep sleep (closest to shutdown)
    esp_deep_sleep_start();
    break;
  default:
    Serial.print(".. failed (error ");
    Serial.print(result);
    Serial.println(")");
    Serial.println("... stopping everything");
    // put ESP32 into deep sleep (closest to shutdown)
    esp_deep_sleep_start();
    break;
  }

  // see if (expected) sensors are there
  for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
    Serial.print("checking for serial communication for sensor on channel ");
    Serial.print(i);
    Serial.print(" .");
    if (!selectI2cMultiplexerChannel(i)) {
      Serial.println(".. failed (channel selection)");
      channel_missing += 1;
      continue;
    }
    Wire.beginTransmission(ICM_ADDRESS);
    result = Wire.endTransmission();
    switch (result) {
    case 0:
      Serial.println(".. worked");
      break;
    case 2:
      Serial.println(".. failed (error 2) maybe not connected?");
      break;
    case 5:
      Serial.println(".. failed (I2C bus timeout)");
      break;
    default:
      Serial.print(".. failed (error ");
      Serial.print(result);
      Serial.println(")");
      break;
    }

    // check for correct sensor communication
    Serial.print("checking sensor on channel ");
    Serial.print(socket[i].channel);
    Serial.print(" (");
    Serial.print(socket[i].label);
    Serial.print(") .");
    if (socket[i].sensor.begin_I2C()) {
      socket[i].usable = true;
      Serial.println(".. works");
    } else {
      sensors_missing += 1;
      Serial.println(".. failed");
    }

    // configure the sensor
    if(!socket[i].setup()){
      sensors_failed += 1;
      continue;
    }
  }

  // general state of sensors
  Serial.print("found ");
  Serial.print(NUMBER_OF_SENSORS - channel_missing);
  Serial.println(" multiplexer channels");

  Serial.print("can communicate with ");
  Serial.print(NUMBER_OF_SENSORS - sensors_missing);
  Serial.println(" sensors");

  Serial.print("setup of ");
  Serial.print(NUMBER_OF_SENSORS - sensors_failed);
  Serial.println(" sensors worked");
}

/**
 * The main processing loop.
 *
 * @see setup()
 * @see selectI2cMultiplexerChannel(uint8_t channel)
 */
void loop() {
  Serial.print(".");
  delay(1000);

  // sequentially get all sensor data via each channel
  for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
	  socket[i].update();
      // quaternion: x,y,z,w
      // euler angle: x,y,z
      // gyro: x,y,z
      Serial.print(socket[i].getGyroX());
  }
}
