/** @file
 * This firmware uses one TCA9548A to read data from 8 ICM20948 sensors
 * via I2C. This data is slightly prepocessed passed on via OSC.
 *
 * @todo implement magnetic north calibration
 * @todo implement sensor resurection
 * @todo implement sensor calibration
 */
// libraries for local sensor communication
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// libraries for wireless communication
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

//-------BEGIN NETWORK SETTINGS--------
WiFiUDP Udp;                         /**< handler for UDP communication */
#define WIFI_SSID "ArtNet4Hans"     /**< SSID / name of the wifi network to use */
#define WIFI_PASS "kaesimira"  /**< password for the wifi network to use */
IPAddress receiverIp(192, 168, 0, 2);     /**< IP address of the (target) OSC server */
int receiverPort = 8000;             /**< UDP server port on OSC receiver (i.e. central server) */
int localPort = 8888;                /**< source port for UDP communication on ESP32 */
//-------END NETWORK SETTINGS--------
#define MAGNETIC_DECLINATION 4.80 /**< difference between true north and magnetic north, see https://www.magnetic-declination.com/ */

// SDA and SCL pin of the soft and hard wire mode
#define SDA_PIN 22       /**< I2C data pin (on ESP32) */
#define SCL_PIN 21       /**< I2C clock pin (on ESP32) */
#define TCA_ADDRESS 0x70 /**< I2C address of the TCA9548A (I2C multiplexer) */
#define ICM_ADDRESS 0x69 /**< I2C address of the ICM20948 sensor */

#define ID_PIN1 27   /**< 1st bit pin of ID DIP switch (D27) */
#define ID_PIN2 14   /**< 2nd bit pin of ID DIP switch (D14) */
#define ID_PIN3 12   /**< 3rd bit pin of ID DIP switch (D12) */
#define ID_PIN4 13   /**< 4rd bit pin of ID DIP switch (D13) */

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
 * Retrieve the (unique) ID configured for this controller.
 * This ID is used in the OSC messages to identify the sender.
 * Its value is between 0 and 15 inclusively.
 *
 * @return configured ID of the controller.
 * @see getControllerIdChars()
 */
uint8_t getControllerID() {
  uint8_t id = 0;

  // read the switch state from left to right and add value at position
  if (HIGH == digitalRead(ID_PIN1)) {
	id = 8;
  }
  if (HIGH == digitalRead(ID_PIN2)) {
	id += 4;
  }
  if (HIGH == digitalRead(ID_PIN3)) {
	id += 2;
  }
  if (HIGH == digitalRead(ID_PIN4)) {
	id += 1;
  }
  return id;
}

/**
 * Retrieve the (unique) ID of the controller board
 *
 * @return ID as const char*
 * @see getControllerID()
 */
const char* getControllerIdChars() {
  // ID as four bit, i.e. max value is 15, to 3 bytes (for NULL)
  char* str = (char*)malloc(3*sizeof(char));
  str[2] = '\n'; // just to be sure
  uint8_t id = getControllerID();
  sprintf(str,"%d",id);
  return str;
}

/**
 * A data structure to handle hardware related data and
 * communication of one ICM20948.
 *
 * @todo public/private visibility of members
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
  OSCMessage osc;              /**< OSC message with sensor values to send out */
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /**< internal quaternion storage */
  float q_r[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /**< internal rotated quaternion storage */
  unsigned long last_update = 0;  /**< timestamp of last quaternion update (in microseconds since boot) */
  float theta = 0.0f; /**< sensor angle to (reference) north */
  // https://en.wikipedia.org/wiki/Aircraft_principal_axes
  float roll = 0.0f; /**< rotation around center of gravity and and forward direction */
  float pitch = 0.0f; /**< rotation around lateral axis / center of gravity (directed) to the right (think wingtip to wingtip) */
  float yaw = 0.0f; /**< rotation around axis from the center of gravity (directed) towards the bottom of the sensor */

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
   * @see assembleOSCmessage()
   * @see update_quaternion()
   * @see update_angles()
   */
  bool update() {
	if (!selectI2cMultiplexerChannel(this->channel)) {
      return false;
    }
    if (this->usable) {
      this->sensor.getEvent(&this->accel_event, &this->gyro_event,
                            &this->temp_event, &this->mag_event);
      this->update_quaternion();
      this->update_angles();
      return true;
    }
    return false;
  }

  /**
   * Update the quaternion without any value filtering.
   * This includes the rotation.
   *
   * @see getQuaternionRX()
   * @see getQuaternionRY()
   * @see getQuaternionRZ()
   * @see getQuaternionRW()
   * @see update()
   * @see update_angles()
   */
  void update_quaternion() {
    // determine time passed since last update
    unsigned long now = micros();
    unsigned long deltaTus = now - this->last_update;
    this->last_update = now;
    float deltaT = fabs(deltaTus * 0.001 * 0.001);

    // adjust gyro values
    float gx = this->gyro_event.gyro.x * DEG_TO_RAD;
    float gy = -(this->gyro_event.gyro.y) * DEG_TO_RAD;
    float gz = -(this->gyro_event.gyro.z) * DEG_TO_RAD;

    // calculate quaternion
    this->q[0] += 0.5f * (-(this->q[1]) * gx - (this->q[2]) * gy - (this->q[3]) * gz) * deltaT;
    this->q[1] += 0.5f * ((this->q[0]) * gx + (this->q[2]) * gz - (this->q[3]) * gy) * deltaT;
    this->q[2] += 0.5f * ((this->q[0]) * gy - (this->q[1]) * gz + (this->q[3]) * gx) * deltaT;
    this->q[3] += 0.5f * ((this->q[0]) * gz + (this->q[1]) * gy - (this->q[2]) * gx) * deltaT;
    float recipNorm = 1.0 / sqrt((this->q[0] * this->q[0]) + (this->q[1] * this->q[1]) + (this->q[2] * q[2]) + (this->q[3] * this->q[3]));
    this->q[0] *= recipNorm;
    this->q[1] *= recipNorm;
    this->q[2] *= recipNorm;
    this->q[3] *= recipNorm;

    // rotate quaternion
    float halfcos = cos(theta/2);
    float halfsin = sin(theta/2);
    this->q_r[0] = this->q[0]*halfcos - this->q[3]*halfsin;
    this->q_r[1] = this->q[1]*halfcos - this->q[2]*halfsin;
    this->q_r[2] = this->q[2]*halfcos + this->q[1]*halfsin;
    this->q_r[3] = this->q[3]*halfcos + this->q[0]*halfsin;
  }

  /**
   * Update the internal values for roll, pitch, and yaw.
   *
   * @see update_quaternion()
   * @note make sure quaternions are up to date
   */
  void update_angles() {
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // use parts of the quaternion as Tait-Bryan angles
    // rotation matrix coefficients for Euler angles and gravity components
    float a12, a22, a31, a32, a33;
    float qx = this->getQuaternionX();
    float qy = this->getQuaternionY();
    float qz = this->getQuaternionZ();
    float qw = this->getQuaternionW();
    a12 = 2.0f * (qx * qy + qw * qz);
    a22 = qw * qw + qx * qx - qy * qy - qz * qz;
    a31 = 2.0f * (qw * qx + qy * qz);
    a32 = 2.0f * (qx * qz - qw * qy);
    a33 = qw * qw - qx * qx - qy * qy + qz * qz;
    this->roll = atan2f(a31, a33);
    this->pitch = -asinf(a32);
    this->yaw = atan2f(a12, a22);
    this->roll *= 180.0f / PI;
    this->pitch *= 180.0f / PI;
    this->yaw *= 180.0f / PI;
    this->yaw += MAGNETIC_DECLINATION;
    if (this->yaw >= +180.f) {
      this->yaw -= 360.f;
    } else {
      if (this->yaw < -180.f) {
        this->yaw += 360.f;
      }
    }
  }

  /**
   * @return x axis acceleration of the gyro (in m/s^2)
   * @see assembleOSCmessage()
   */
  float getGyroX() const { return this->gyro_event.gyro.x; }

  /**
   * @return y axis acceleration of the gyro (in m/s^2)
   * @see assembleOSCmessage()
   */
  float getGyroY() const { return this->gyro_event.gyro.y; }

  /**
   * @return z axis acceleration of the gyro (in m/s^2)
   * @see assembleOSCmessage()
   */
  float getGyroZ() const { return this->gyro_event.gyro.z; }

  /**
   * @return euler angle around x axis (in degree)
   *
   * @see assembleOSCmessage()
   * @see update_angles()
   */
  float getEulerX() const { return this->roll; }

  /**
   * @return euler angle around y axis (in degree)
   *
   * @see assembleOSCmessage()
   * @see update_angles()
   */
  float getEulerY() const { return -this->pitch; }

  /**
   * @return euler angle around z axis (in degree)
   *
   * @see assembleOSCmessage()
   * @see update_angles()
   */
  float getEulerZ() const { return -this->yaw; }

  /**
   * Retrieve x component of the quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionY()
   * @see getQuaternionZ()
   * @see getQuaternionW()
   */
  float getQuaternionX() const { return q[1]; }

  /**
   * Retrieve y component of the quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionX()
   * @see getQuaternionZ()
   * @see getQuaternionW()
   */
  float getQuaternionY() const { return q[2]; }

  /**
   * Retrieve z component of the quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionX()
   * @see getQuaternionY()
   * @see getQuaternionW()
   */
  float getQuaternionZ() const { return q[3]; }

  /**
   * Retrieve w component of the quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionX()
   * @see getQuaternionY()
   * @see getQuaternionZ()
   */
  float getQuaternionW() const { return q[0]; }

  /**
   * Retrieve x component of the rotated quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionRY()
   * @see getQuaternionRZ()
   * @see getQuaternionRW()
   */
  float getQuaternionRX() const { return q_r[1]; }

  /**
   * Retrieve y component of the rotated quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionRX()
   * @see getQuaternionRZ()
   * @see getQuaternionRW()
   */

  float getQuaternionRY() const { return q_r[2]; }
  /**
   * Retrieve z component of the rotated quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionRX()
   * @see getQuaternionRY()
   * @see getQuaternionRW()
   */

  float getQuaternionRZ() const { return q_r[3]; }
  /**
   * Retrieve w component of the rotated quaternion.
   *
   * @see update_quaternion()
   * @see getQuaternionRX()
   * @see getQuaternionRY()
   * @see getQuaternionRZ()
   */
  float getQuaternionRW() const { return q_r[0]; }

  /**
   * Assemble an OSC message based on current sensor values.
   *
   * @see printOSC()
   */
  void assembleOSCmessage() {
    // set the quaternion data
    this->osc.add(this->getQuaternionRX())
        .add(this->getQuaternionRY())
        .add(this->getQuaternionRZ())
        .add(this->getQuaternionRW());
    // set the euler angle data
    this->osc.add(this->getEulerX())
        .add(this->getEulerY())
        .add(this->getEulerZ());
    // set the gyro data
    this->osc.add(this->getGyroX()).add(this->getGyroY()).add(this->getGyroZ());
  }

  /**
   * Print the OSC data on the serial line.
   *
   * @see assembleOSCmessage()
   */
  void printOSC() {
    Serial.print(this->getQuaternionRX());
    Serial.print(" ");
    Serial.print(this->getQuaternionRY());
    Serial.print(" ");
    Serial.print(this->getQuaternionRZ());
    Serial.print(" ");
    Serial.print(this->getQuaternionRW());
    Serial.print(" # ");
    Serial.print(this->getEulerX());
    Serial.print(" ");
    Serial.print(this->getEulerY());
    Serial.print(" ");
    Serial.print(this->getEulerZ());
    Serial.print(" # ");
    Serial.print(this->getGyroX());
    Serial.print(" ");
    Serial.print(this->getGyroY());
    Serial.print(" ");
    Serial.println(this->getGyroZ());
  }
};

ICM20948socket socket[NUMBER_OF_SENSORS]; /**< a (global) list of sockets to bundle communication */

/**
 * This function establishes a connection to the preconfigured wifi network.
 *
 * @see #WIFI_SSID
 * @see #WIFI_PASS
 * @see startUdp()
 * @see setup()
 */
void connectWiFi() {
  Serial.print("Connecting to wifi network \"");
  Serial.print(WIFI_SSID);
  Serial.print("\" .");
  // Mode of the WiFi
  //   STA = STATION MODE (connect to access point),
  //   APM = Access Point Mode (create a network)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  long start = millis();
  // try for ten seconds to connect every 500 ms (i.e. make 10000/500 = 20 attempts)
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    Serial.print(".");
    delay(500);
  }

  // print result of connection attempt(s) on serial console
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" failed");
  } else {
	Serial.println(" succeeded");
    Serial.print("local IP address is ");
    Serial.println(WiFi.localIP());
  }
}

/**
 * Set up UDP communication locally.
 *
 * @param port local port to bind to
 * @see localPort
 * @see connectWiFi()
 */
void startUdp(int port) {
  Serial.print("Starting UDP connection to local port ");
  Serial.print(port);
  if (0 == Udp.begin(port)) {
	  // no socket available for use
	  Serial.println(" ... failed");
  }
  Serial.println(" ... succeeded");
}

/**
 * Bring the controller into a working state.
 *
 * @see loop()
 * @see selectI2cMultiplexerChannel(uint8_t channel)
 * @see ICM20948socket
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

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp(localPort);
  Serial.print("target OSC server is ");
  Serial.print(receiverIp);
  Serial.print(" port ");
  Serial.println(receiverPort);

  // socket label & channel
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

  // build OSC paths
  for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
    char path[100];
    strcpy(path, "/body/");
    strcat(path, getControllerIdChars());
    strcat(path, "/gyro/");
    strcat(path, socket[i].label);
    strcat(path, "/");
    socket[i].osc = OSCMessage(path);
  }

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
  delay(1000);

  // sequentially get all sensor data via each channel
  for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
    // skip processing unusable socket
    if (!socket[i].usable) {
      continue;
    }
    // update the sensor measurements
    if(!socket[i].update()) {
      // update failed, mark sensor as unusable
      socket[i].usable = false;
      continue;
    }
    // construct the OSC message
    socket[i].assembleOSCmessage();
    // send the OSC message
    Udp.beginPacket(receiverIp, receiverPort + getControllerID() - 1);
    socket[i].osc.send(Udp);
    Udp.endPacket();
    socket[i].osc.empty();
    socket[i].printOSC();
  }
}
