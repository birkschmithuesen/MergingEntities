/** @file
 *
 * This code is intended to run on an ESP32 (<a hfref="https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf">datasheet</a>)
 * with a TCA9548A I2C multiplexer and generic MPU9250 sensor boards.
 *
 * @note This code base is the leading one in terms of features and maturity.
 * @todo rework calibration routine(s)
 * @todo clean up setup/config code dependend on controller ID
 * @todo clean up OSC code and use new schema
 * @todo implement / fix manual/button press magnetometer calibration (around line 680)
 * @todo unify MPU data and socket structures
 * @todo clean up serial console messages
 * @todo implement (remote) OSC error logger
 * @todo fix LED identifiers w.r.t to new colors
 */
// Connections : Please always connect 2 hard mpu (builtin I2C bus) to your specified pins
// Indicate the numbers of hard, soft I2C, connect the soft I2C in the order of the specified pins
// Specify your IP address and Wifi
// Mag calibration desactivated right now, see if it's usefull
// Eventually speed up if TCA is fast enough
// The IMU is a MPU-9250, hence the reference in variable names etc.

// Attention : select the right channel with the TCA function (+2 woth prototype in wood, i regular without)

//-------LIBRARIES-------
// Library to use Arduino commands
#include <Arduino.h>
// Libraries for communication
#include <OSCMessage.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
// MPU and Bitbang library
#include "MPU9250.h"


//-------GENERAL SETTINGS-------
#define NUMBER_OF_MPU 6 /**< number of IMU (MPU) (boards) attached to the controller  */

// Define the number of the body : 1, 2 or 3
#define BODY_2  /**< indicate individal controler (and thus configuration) */

// Set the magnetic declination of the light
#define MAG_DECLINATION 2.53  /**< magnetic declination of stage light */

// Define the calibration mode : MANUAL_CALIBRATION for manual, AUTO_CALIBRATION
// otherwise
#define AUTO_CALIBRATION  /**< set (automatic) calibration mode */

// Define BUTTON to activate the button
#define BUTTON  /**< indicate existence of button (to trigger calibration procedure) */

//-------END GENERAL SETTINGS-------

//-------BEGIN WIFI SETTINGS--------
WiFiUDP Udp;                         /**< handler for UDP communication */
//#define WIFI_SSID "network name"     /**< SSID / name of the wifi network to use */
//#define WIFI_PASS "access password"  /**< password for the wifi network to use */
#define WIFI_SSID "ArtNet4Hans"     /**< SSID / name of the wifi network to use */
#define WIFI_PASS "kaesimira"  /**< password for the wifi network to use */
IPAddress outIp(192, 168, 0, 2);     /**< IP address of the (target) OSC server */
int localPort = 8888;                /**< source port for UDP communication on ESP32 */
//-------END WIFI SETTINGS--------

//-------MPU SETTINGS AND FUNCTIONS-------
// Parameters of the setup

// Addresses and pin of IMU (MPU-9250) and TCA9548A(=multiplexer)
#define MPU_ADDRESS_1 0x68           /**< address of the MPU-9250 when its pin AD0 is low */
#define MPU_ADDRESS_2 0x69           /**< address of the MPU-9250 when its pin AD0 is high */
#define TCA_ADDRESS_RIGHT_SIDE 0x70  /**< address of the "right side" 8 channel I2C switch */
#define TCA_ADDRESS_LEFT_SIDE 0x71   /**< address of the "left side" 8 channel I2C switch */

// SDA and SCL pin of the soft and hard wire mode
#define SDA_PIN 21   /**< I2C data pin (on ESP32) */
#define SCL_PIN 22   /**< I2C clock pin (on ESP32) */

// LED pin for info showing, BUTTON pin for communication
#define RED_PIN 32   /**< ESP pin number of red LED */
#define YEL_PIN 33   /**< ESP pin number of yellow LED */
#define BUTTON_PIN 5 /**< ESP pin number of (callibration) button */
#define ID_PIN1 27   /**< 1st bit pin of ID DIP switch (D27) */
#define ID_PIN2 14   /**< 2nd bit pin of ID DIP switch (D14) */
#define ID_PIN3 12   /**< 3rd bit pin of ID DIP switch (D12) */
#define ID_PIN4 13   /**< 4rd bit pin of ID DIP switch (D13) */

#define MPU_NORTH 1 /**< MPU used to set the north */
float theta = 0;    /**< angle to the north */

int state = HIGH;        /**< last state of the button */
int state_button = LOW;  /**< current state of the button */

/**
 * A data structure to handle hardware related data of one MPU9250.
 *
 * @todo Also move data read from sensor here (?)
 * @see MPU9250data
 */
struct MPU9250socket {
	String label;        /**< human readable identification of the sensor (used on OSC path) */
	uint8_t multiplexer; /**< I2C address of the responsible I2C multiplexer */
	uint8_t channel;     /**< channel used on the I2C multiplexer */
	MPU9250 mpu;         /**< software handler/abstraction for MPU at given channel of given multiplexer */
	bool usable = false; /**< indicate that sensor (data) is present and no errors occured */
};

/**
 * This data structure models a quaternion for easier access to its component data.
 *
 * @see MPU9250data
 * @see EulerAngle
 * @see GyroValue
 */
struct Quaternion {
	float x = 0.0; /**< The x value of the quaternion */
	float y = 0.0; /**< The y value of the quaternion */
	float z = 0.0; /**< The z value of the quaternion */
	float w = 0.0; /**< The w value of the quaternion */
};

/**
 * This data structure models an Euler angle for easier access to its component data.
 *
 * @see MPU9250data
 * @see Quaternion
 * @see GyroValue
 */
struct EulerAngle {
	float x = 0.0; /**< The x-axis value of the Euler angle */
	float y = 0.0; /**< The y-axis value of the Euler angle */
	float z = 0.0; /**< The z-axis value of the Euler angle */
};

/**
 * This data structure models gyroscope data for easier access to its component data.
 *
 * @see MPU9250data
 * @see Quaternion
 * @see EulerAngle
 */
struct GyroValue {
	float x = 0.0; /**< The x-axis value of the gyroscope */
	float y = 0.0; /**< The y-axis value of the gyroscope */
	float z = 0.0; /**< The z-axis value of the gyroscope */
};

/**
 * A model of the data retrieved from a MPU9250.
 * 
 * @note This is not a model for the sensor board itself.
 * @see MPU9250socket
 */
struct MPU9250data {
	Quaternion quaternion; /**< gyroscope data as quaternion */
	EulerAngle eulerangle; /**< gyroscope data as Euler angle */
	GyroValue gyrovalue;   /**< gyroscope data along axis */
};

// manually create indexes to emulate a hashmap with an array
#define LEFT_UPPER_ARM_INDEX 0  /**< index for the sensor at the left upper arm (brachium) */
#define RIGHT_UPPER_ARM_INDEX 1 /**< index for the sensor at the right upper arm (brachium) */
#define LEFT_FOOT_INDEX 2       /**< index for the sensor at the left foot */
#define RIGHT_FOOT_INDEX 3      /**< index for the sensor at the right foot */
#define BACK_INDEX 4            /**< index for the sensor at the back */
#define HEAD_INDEX 5            /**< index for the sensor at the head (cranium) */
#define LEFT_LOWER_ARM_INDEX 6  /**< index for the sensor at the left lower arm (antebrachium) */
#define RIGHT_LOWER_ARM_INDEX 7 /**< index for the sensor at the right lower arm (antebrachium) */
#define LEFT_UPPER_LEG_INDEX 8  /**< index for the sensor at the left thigh (femur) */
#define RIGHT_UPPER_LEG_INDEX 9 /**< index for the sensor at the right thigh (femur) */

// Instance to store data on ESP32, name of the preference
Preferences preferences;  /**< container for preferences on ESP32 */
char mpuPref[10];         /**< preferences of each MPU stored on ESP32 */

// MPU9250 settings and data storage
MPU9250socket sensors[NUMBER_OF_MPU];   /**< communication abstractions for all MPUs */
MPU9250data mpuData[NUMBER_OF_MPU]; /**< bundled MPU9250 data to be accessed by index */
float accbias[6][3];     /**< bias/drift/offset profile for the accelerator */
float gyrobias[6][3];    /**< bias/drift/offset profile for the gyroscope */
MPU9250Setting setting;  /**< configuration settings of the MPU9250 stored in memory */

/**
 * Switch to the given channel on the multiplexer for I2C communication.
 *
 * This function updates the control register in the switch to select one
 * of the eight I2C devices (numbered 0..7) attached to it.
 *
 * @param address The I2C address of the multiplexer to use
 * @param channel The channel to select to communicate with I2C client
 * @return true if selection was successful, false if not
 * @see countI2cDevices()
 * @see countMultiplexer()
 * @see checkAndConfigureGyros()
 * @see fetchData()
 * @todo limit processing to valid values (0..7)
 */
bool selectI2cMultiplexerChannel(uint8_t address, uint8_t channel) {
  bool result = false;
  // select the multiplexer by its hardware address
  Wire.beginTransmission(address);
  // select a channel on the multiplexer
  if (1 == Wire.write(1 << channel)) {
    result = true;
  }
  Wire.endTransmission();
  return result;
}

/**
 * Count I2C devices (IMU) connected to ESP and
 * print the result to the serial output.
 *
 * @note Select a multiplexer and channel first.
 *
 * @see selectI2cMultiplexerChannel(uint8_t address, uint8_t channel)
 * @see countMultiplexer()
 * @see checkAndConfigureGyros()
 * @todo select switch channel to scan via function argument?
 */
uint8_t countI2cDevices() {
  byte error;
  uint8_t deviceCount = 0;

  Serial.println("Scanning...");

  // go through each possible address
  for (uint8_t address = 1; address < 127; address++) {
    // try to initiate a conversation
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      // if talking was successful, we found a
      // device at the current address and print that
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        // pad leading 0
        Serial.print("0");
      }

      Serial.print(address, HEX);
      Serial.println("  !");

      // and we increase the number of devices found
      deviceCount++;
    } else {
      // if not, there was no suitable device
      if (error == 4) {
        // but we can still mention specific errors
        Serial.print("Unknown error at address 0x");
        if (address < 16) {
          Serial.print("0");
        }

        Serial.println(address, HEX);
      }
    }
  }

  // we to some final reporting
  if (deviceCount == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("done");
  }

  //delay(5000); // wait 5 seconds for next scan - TODO: remove?
  return deviceCount;
}

//-------WIFI SETTINGS AND FUNCTIONS-------
#ifdef BODY_1
int outPort = 8000;  /**< UDP server port on OSC receiver (i.e. central server) */
/** OSC messages with senor values for given part */
OSCMessage body[] = {
    OSCMessage("/body/1/gyro/left_upper_arm/"), OSCMessage("/body/1/gyro/right_upper_arm/"),
    OSCMessage("/body/1/gyro/left_foot/"), OSCMessage("/body/1/gyro/right_foot/"),
    OSCMessage("/body/1/gyro/back/"), OSCMessage("/body/1/gyro/head/")};
OSCMessage calibration("/calibration/1");  /**< OSC endpoint for calibration messages */
/** scaling factors for magnetometer */
float magscale[6][3] = {{1.01, 1.06, 0.94}, {1.01, 1.00, 0.99},
                        {1.04, 0.98, 0.98}, {1.06, 0.97, 0.98},
                        {0.99, 1.00, 1.01}, {1.02, 1.03, 0.95}};
/** bias factors for magnetometer */
float magbias[6][3] = {{-40.82, -108.61, -405.33}, {128.62, 104.29, -164.14},
                       {103.32, 249.40, -116.79},  {-15.87, 157.95, -46.02},
                       {-3.55, 46.14, -403.94},    {-17.57, 327.23, -390.66}};
#endif

#ifdef BODY_2
int outPort = 8001;  /**< UDP server port on OSC receiver (i.e. central server) */
/** OSC messages with senor values for given part */
OSCMessage body[] = {
    OSCMessage("/body/2/gyro/left_upper_arm/"), OSCMessage("/body/2/gyro/right_upper_arm/"),
    OSCMessage("/body/2/gyro/left_foot/"), OSCMessage("/body/2/gyro/right_foot/"),
    OSCMessage("/body/2/gyro/back/"), OSCMessage("/body/2/gyro/head/")};
OSCMessage calibration("/calibration/2");  /**< OSC endpoint for calibration messages */
/** scaling factors for magnetometer */
float magscale[6][3] = {{0.99, 1.01, 1.00}, {0.98, 1.00, 1.02},
                        {0.98, 1.03, 0.98}, {1.03, 0.99, 0.99},
                        {1.02, 0.99, 1.00}, {1.01, 0.98, 1.01}};
/** bias factors for magnetometer */
float magbias[6][3] = {{98.40, -5.27, -345.30}, {399.67, 242.51, -126.99},
                       {-48.23, -92.89, 67.16}, {8.90, -89.03, -82.37},
                       {5.31, 188.74, -324.95}, {183.41, 101.35, -152.56}};
#endif

#ifdef BODY_3
int outPort = 8002;  /**< UDP server port on OSC receiver (i.e. central server) */
/** OSC messages with senor values for given part */
OSCMessage body[] = {
    OSCMessage("/body/3/gyro/left_upper_arm/"), OSCMessage("/body/3/gyro/right_upper_arm/"),
    OSCMessage("/body/3/gyro/left_foot/"), OSCMessage("/body/3/gyro/right_foot/"),
    OSCMessage("/body/3/gyro/back/"), OSCMessage("/body/3/gyro/head/")};
OSCMessage calibration("/calibration/3");  /**< OSC endpoint for calibration messages */
#endif

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
  // try for ten seconds to connect every 100 ms (i.e. make 100 attempts)
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    Serial.print(".");
    delay(100);
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
 * Retrieve the (unique) ID configured for this controller.
 * This ID is used in the OSC messages to identify the sender.
 * Its value is between 0 and 15 inclusively.
 *
 * @return configured ID of the controller.
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
 * Calibrate the magnetometer (i.e. set north) by manually
 * fiddling with the sensor.
 *
 * @see automaticMagnetometerCalibration()
 * @see buttonBasedCalibration()
 * @note This code has been extracted manually and not changed/adjusted
 */
void manualMagnetometerCalibration() {
  Serial.println("manual calibration of magnetometers");

  // go through each sensor socket
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
	// blink LEDs to indicate calibration going on
    if (state == HIGH) {
      digitalWrite(YEL_PIN, state);
      state = LOW;
    } else {
      digitalWrite(YEL_PIN, state);
      state = HIGH;
    }
    calibration.add("Calibration of ").add(i + 1);

    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();

    Serial.println(i);

    if (!selectI2cMultiplexerChannel(sensors[i].multiplexer, sensors[i].channel)) {
      Serial.print("could not select channel ");
      Serial.print(sensors[i].channel);
      Serial.print(" on multiplexer at address ");
      Serial.println(sensors[i].multiplexer);
    }
    sensors[i].mpu.setMagneticDeclination(MAG_DECLINATION);
    sensors[i].mpu.calibrateMag();

    calibration.empty();
  }
}


/**
 * Calibrate the magnetometer (i.e. set north) automatically. This requires
 * detailed knowledge of the stage (layout / setup) and being diligent
 * with the gyro wiring.
 *
 * @see manualMagnetometerCalibration()
 * @see buttonBasedCalibration()
 * @note This code has been extracted manually and not changed/adjusted
 */
void automaticMagnetometerCalibration() {
  calibration.add("Automatic calibration of the magnetometer gyro");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    sensors[i].mpu.setMagneticDeclination(MAG_DECLINATION);
    sensors[i].mpu.setMagBias(magbias[i][0], magbias[i][1], magbias[i][2]);
    sensors[i].mpu.setMagScale(magscale[i][0], magscale[i][1], magscale[i][2]);
  }
  Serial.println("Mag calibration done");
}


/**
 * Count the number of I2C multiplexer attached to the controller.
 *
 * This routine is useful to guess the number of TCA9548A I2C multiplexer
 * attached to the controller and thus select a (sub-)set of OSC paths
 * and data to be transmitted.
 *
 * @return Number of TCA9548A I2C multiplexer
 * @see countI2cDevices()
 * @see selectI2cMultiplexerChannel(uint8_t address, uint8_t channel)
 */
uint8_t countMultiplexer(){
  uint8_t count = 0;
  if (selectI2cMultiplexerChannel(TCA_ADDRESS_RIGHT_SIDE, 1)) {
    count = 1;
  }
  if (selectI2cMultiplexerChannel(TCA_ADDRESS_LEFT_SIDE, 1)) {
    count = 2;
  }
  return count;
}

/**
 * Check which MPU9250 sensor board is there and do the initial
 * configuration.
 *
 * @see setup()
 * @see selectI2cMultiplexerChannel(uint8_t address, uint8_t channel)
 * @see countI2cDevices()
 * @todo send channel selection error via OSC
 */
void checkAndConfigureGyros() {
  // MPU parameters (sensitivity, etc)
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  // filter for measurement data
  QuatFilterSel sel{QuatFilterSel::MADGWICK};

  // Lauch communication with the MPUs
  // go through list of (expected) sensors and see if they are there
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    // skip sensors that are already configured (i.e. usable)
    if (sensors[i].usable) {
      continue;
    }

    // select channel on multiplexer
    if (!selectI2cMultiplexerChannel(sensors[i].multiplexer,
                                     sensors[i].channel)) {
      // selection failed
      sensors[i].usable = false;
      Serial.print("could not select channel ");
      Serial.print(sensors[i].channel);
      Serial.print(" on multiplexer at address ");
      Serial.println(sensors[i].multiplexer);
      continue;
    }

    // sanity check: there should only be one device and the MPU should be
    // available
    if (!((1 == countI2cDevices()) && (sensors[i].mpu.available()))) {
      sensors[i].usable = false;
      continue;
    }

    // try to initialize the multiplexer with the (global) settings
    if (!sensors[i].mpu.setup(sensors[i].multiplexer, setting, Wire)) {
      // somehow it failed
      sensors[i].usable = false;
      continue;
    }

    // configure the filter for the measured data
    sensors[i].mpu.selectFilter(sel);
    sensors[i].mpu.setFilterIterations(10);

    // everything is done and now the senor is usable
    sensors[i].usable = true;
  }
}

/**
 * Calibrate the Accelerometers and store calibration data.
 *
 * @note The device should not be moved during the calibration procedure.
 *
 * @see buttonBasedCalibration()
 * @see passiveMagnetometerCalibration()
 */
void passiveAccelerometerCalibration() {
  Serial.println("Calibration of acceleration : don't move devices");
  calibration.add("Calibration of acceleration : don't move devices");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();
  digitalWrite(RED_PIN, HIGH);
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    Serial.println(i);
    if (!selectI2cMultiplexerChannel(sensors[i].multiplexer,
                                     sensors[i].channel)) {
      Serial.print("could not select channel ");
      Serial.print(sensors[i].channel);
      Serial.print(" on multiplexer at address ");
      Serial.println(sensors[i].multiplexer);
    }
    sensors[i].mpu.calibrateAccelGyro();
  }
  digitalWrite(RED_PIN, LOW);
  Serial.println("Acceleration calibration done.");

  // Acceleration: store calibration data
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    itoa(i, mpuPref, 10); // Key names = number of mpu
    preferences.begin(mpuPref, false);

    preferences.putFloat("accbiasX", sensors[i].mpu.getAccBiasX());
    preferences.putFloat("accbiasY", sensors[i].mpu.getAccBiasY());
    preferences.putFloat("accbiasZ", sensors[i].mpu.getAccBiasZ());

    preferences.putFloat("gyrobiasX", sensors[i].mpu.getGyroBiasX());
    preferences.putFloat("gyrobiasY", sensors[i].mpu.getGyroBiasY());
    preferences.putFloat("gyrobiasZ", sensors[i].mpu.getGyroBiasZ());

    preferences.end();
  }
}

/**
 * Calibrate the Magnetometer and store calibration data.
 *
 * @note The device should not be moved during the calibration procedure.
 *
 * @see buttonBasedCalibration()
 * @see passiveMagnetometerCalibration()
 */
void passiveMagnetometerCalibration() {
  Serial.println("Calibration of magnetometers");

  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    if (state == HIGH) {
      digitalWrite(YEL_PIN, state);
      state = LOW;
    } else {
      digitalWrite(YEL_PIN, state);
      state = HIGH;
    }
    calibration.add("Calibration of ").add(i + 1);

    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();

    Serial.println(i);

    // selectI2cMultiplexerChannel(TCA_ADDRESS_RIGHT_SIDE, i);
    if (!selectI2cMultiplexerChannel(sensors[i].multiplexer,
                                     sensors[i].channel)) {
      Serial.print("could not select channel ");
      Serial.print(sensors[i].channel);
      Serial.print(" on multiplexer at address ");
      Serial.println(sensors[i].multiplexer);
    }
    sensors[i].mpu.setMagneticDeclination(MAG_DECLINATION);
    sensors[i].mpu.calibrateMag();

    calibration.empty();
  }
  Serial.println("Calibration of magnetometers done");

  // Magnetometer : store calibration data
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    itoa(i, mpuPref, 10); // Key names = number of mpu
    preferences.begin(mpuPref, false);

    preferences.putFloat("magbiasX", sensors[i].mpu.getMagBiasX());
    preferences.putFloat("magbiasY", sensors[i].mpu.getMagBiasY());
    preferences.putFloat("magbiasZ", sensors[i].mpu.getMagBiasZ());

    preferences.putFloat("magscaleX", sensors[i].mpu.getMagScaleX());
    preferences.putFloat("magscaleY", sensors[i].mpu.getMagScaleY());
    preferences.putFloat("magscaleZ", sensors[i].mpu.getMagScaleZ());

    preferences.end();
  }
}


/**
 * Do the calibration with button choices.
 *
 * @warning This is just a copy of the code from the setup routine. It has not
 * been refactored/reviewed yet.
 * @see passiveAccelerometerCalibration()
 * @see automaticMagnetometerCalibration()
 * @see manualMagnetometerCalibration()
 */
void buttonBasedCalibration() {
  float time_passed = 0;  // tracking time passed

  //-------FIRST CHOICE-------
  // Two LEDs are on: you have 6 seconds to press
  // or not to press the button, to launch a calibration process
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, HIGH);
  delay(6000);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);
  state_button = digitalRead(BUTTON_PIN);

  // state_button = LOW;
  if (state_button == HIGH) {
    Serial.println("HIGH : Launch calibration sequence");
    // Acceleration: get data calibration + calibrate
    passiveAccelerometerCalibration();
    // Magnetometer : get data calibration + calibrate
    passiveMagnetometerCalibration();
  } else {
	// Button not pushed : we read the stored calibration data and calibrate
    Serial.println("LOW : Load calibration data");
    for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
      itoa(i, mpuPref, 10); // Key names = number of mpu
      preferences.begin(mpuPref, false);

      // Set acceleration calibration data

      sensors[i].mpu.setAccBias(preferences.getFloat("accbiasX", 0),
                        preferences.getFloat("accbiasY", 0),
                        preferences.getFloat("accbiasZ", 0));
      sensors[i].mpu.setGyroBias(preferences.getFloat("gyrobiasX", 0),
                         preferences.getFloat("gyrobiasY", 0),
                         preferences.getFloat("gyrobiasZ", 0));

      // Test to see what the fuck
      /*
      digitalWrite(RED_PIN, HIGH);
      for(uint8_t i=0; i<NUMBER_OF_MPU; i++) {
        Serial.println(i);
        selectI2cMultiplexerChannel(sensors[i].multiplexer, sensors[i].channel);
        mpu[i].calibrateAccelGyro();
      }
      digitalWrite(RED_PIN, LOW);
      Serial.println("Acceleration calibration done.");*/

      // Set magnetometer calibration data
      sensors[i].mpu.setMagBias(preferences.getFloat("magbiasX", 0),
                        preferences.getFloat("magbiasY", 0),
                        preferences.getFloat("magbiasZ", 0));
      sensors[i].mpu.setMagScale(preferences.getFloat("magscaleX", 0),
                         preferences.getFloat("magscaleY", 0),
                         preferences.getFloat("magscaleZ", 0));
      sensors[i].mpu.setMagneticDeclination(MAG_DECLINATION);
      preferences.end();
    }
    Serial.println("Calibration loaded");
  }

  // Two leds are blinking, saying calibration is over
  for (uint8_t i = 0; i < 20; i++) {
    digitalWrite(RED_PIN, state);
    digitalWrite(YEL_PIN, state);
    if (state == HIGH) {
      state = LOW;
    } else {
      state = HIGH;
    }
    delay(200);
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  //-------SECOND CHOICE-------
  // Two leds are on: you have 10 seconds to
  // orientate the MPU 1 over the new north and press the button
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, HIGH);

  // TODO: fix this
  // We let the mpu run for 10 seconds to have the good yaw if we need it
  if (!selectI2cMultiplexerChannel(TCA_ADDRESS_RIGHT_SIDE, MPU_NORTH - 1)) {
    Serial.print("could not select channel ");
    Serial.print(MPU_NORTH - 1);
    Serial.print(" on multiplexer at address ");
    Serial.println(TCA_ADDRESS_RIGHT_SIDE);
  }

  time_passed = millis();
  while (millis() - time_passed < 10000) {
    sensors[MPU_NORTH - 1].mpu.update();
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  state_button = digitalRead(BUTTON_PIN);
  // state_button = HIGH;
  if (state_button == HIGH) {
    Serial.println("HIGH : setting north");
    // We save the north direction and send it to the library
    theta = sensors[MPU_NORTH - 1].mpu.getYaw() * (-1);

    preferences.begin("setNorth", false);
    preferences.putFloat("north", theta);
    preferences.end();
  } else {
    Serial.println("LOW : Load former north");
  }

  // We get the north and set it
  preferences.begin("setNorth", false);
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    sensors[i].mpu.setNorth(preferences.getFloat("north", 0));
  }
  preferences.end();
  Serial.println("North set");

  // Two leds are blinking, saying north is set
  for (uint8_t i = 0; i < 20; i++) {
    digitalWrite(RED_PIN, state);
    digitalWrite(YEL_PIN, state);
    if (state == HIGH) {
      state = LOW;
    } else {
      state = HIGH;
    }
    delay(200);
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);
}

/**
 * Fetch data from all MPU9250 boards and update global value storage.
 * 
 * @see selectI2cMultiplexerChannel(uint8_t address, uint8_t channel)
 * @see setup()
 * @see loop()
 * @todo indicate errors
 */
void fetchData() {
  // fetch data from each MPU
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    if (!selectI2cMultiplexerChannel(sensors[i].multiplexer,
                                     sensors[i].channel)) {
      Serial.print("could not select channel ");
      Serial.print(sensors[i].channel);
      Serial.print(" on multiplexer at address ");
      Serial.println(sensors[i].multiplexer);
    }
    if (sensors[i].usable) {
      if(!sensors[i].mpu.update()) {
        // too harsh?
        sensors[i].usable = false;
      }
    } else {
      // TODO: log errors remotely
      Serial.print("sensor for ");
      Serial.print(sensors[i].label);
      Serial.println(" is not usable");
    }
  }

  // Store sensor values
  for (int i = 0; i < NUMBER_OF_MPU; i++) {
    mpuData[i].quaternion.x = sensors[i].mpu.getQuaternionX();
    mpuData[i].quaternion.y = sensors[i].mpu.getQuaternionY();
    mpuData[i].quaternion.z = sensors[i].mpu.getQuaternionZ();
    mpuData[i].quaternion.w = sensors[i].mpu.getQuaternionW();

    mpuData[i].eulerangle.x = sensors[i].mpu.getEulerX();
    mpuData[i].eulerangle.y = sensors[i].mpu.getEulerY();
    mpuData[i].eulerangle.z = sensors[i].mpu.getEulerZ();

    mpuData[i].gyrovalue.x = sensors[i].mpu.getGyroX();
    mpuData[i].gyrovalue.y = sensors[i].mpu.getGyroY();
    mpuData[i].gyrovalue.z = sensors[i].mpu.getGyroZ();
  }
}

/**
 * Main setup / initialisation routine.
 *
 * @see connectWiFi()
 * @see startUdp()
 * @see loop()
 */
void setup() {
  //-------HARDWARE SETUP-------
  Serial.begin(115200);
  Serial.flush(); // Clean buffer

  Serial.print("setting up LED pins .");
  // LED initialization
  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(ID_PIN1, INPUT);
  pinMode(ID_PIN2, INPUT);
  pinMode(ID_PIN3, INPUT);
  pinMode(ID_PIN4, INPUT);
  Serial.print(".");
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);
  Serial.println(". done");

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp(localPort);
  Serial.print("target OSC server is ");
  Serial.print(outIp);
  Serial.print(" port ");
  Serial.println(outPort);


  //-------MPU SETUP------
  // Launch comm with multiplexer
  Serial.print("setting up I2C pins .");
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(2000);
  Serial.println(".. done");

  // all senor adressing
  Serial.print("setting up sensor sockets .");
  switch (countMultiplexer()) {
  case 1:
    if ((1 == countMultiplexer()) && (10 == NUMBER_OF_MPU)) {
      Serial.println("");
      Serial.println("---------------------------------------------------");
      Serial.println("10 sensors incompatible with single I2C multiplexer");
      Serial.println("---------------------------------------------------");
      // put ESP32 into deep sleep (closest to shutdown)
      esp_deep_sleep_start();
    }
    sensors[LEFT_UPPER_ARM_INDEX].label = "left_upper_arm";
    sensors[LEFT_UPPER_ARM_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[LEFT_UPPER_ARM_INDEX].channel = 0;
    sensors[RIGHT_UPPER_ARM_INDEX].label = "right_upper_arm";
    sensors[RIGHT_UPPER_ARM_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[RIGHT_UPPER_ARM_INDEX].channel = 1;
    sensors[LEFT_FOOT_INDEX].label = "left_foot";
    sensors[LEFT_FOOT_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[LEFT_FOOT_INDEX].channel = 2;
    sensors[RIGHT_FOOT_INDEX].label = "right_foot";
    sensors[RIGHT_FOOT_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[RIGHT_FOOT_INDEX].channel = 3;
    sensors[BACK_INDEX].label = "back";
    sensors[BACK_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[BACK_INDEX].channel = 4;
    sensors[HEAD_INDEX].label = "head";
    sensors[HEAD_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[HEAD_INDEX].channel = 5;
    break;
  case 2:
    // first do the minimal setup
    sensors[RIGHT_UPPER_ARM_INDEX].label = "right_upper_arm";
    sensors[RIGHT_UPPER_ARM_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[RIGHT_UPPER_ARM_INDEX].channel = 2;
    sensors[RIGHT_FOOT_INDEX].label = "right_foot";
    sensors[RIGHT_FOOT_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[RIGHT_FOOT_INDEX].channel = 5;
    sensors[BACK_INDEX].label = "hip";
    sensors[BACK_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
    sensors[BACK_INDEX].channel = 7;
    sensors[LEFT_UPPER_ARM_INDEX].label = "left_upper_arm";
    sensors[LEFT_UPPER_ARM_INDEX].multiplexer = TCA_ADDRESS_LEFT_SIDE;
    sensors[LEFT_UPPER_ARM_INDEX].channel = 2;
    sensors[LEFT_FOOT_INDEX].label = "left_foot";
    sensors[LEFT_FOOT_INDEX].multiplexer = TCA_ADDRESS_LEFT_SIDE;
    sensors[LEFT_FOOT_INDEX].channel = 5;
    sensors[HEAD_INDEX].label = "head";
    sensors[HEAD_INDEX].multiplexer = TCA_ADDRESS_LEFT_SIDE;
    sensors[HEAD_INDEX].channel = 7;
    // add the additional sensors if build is configured that way
    if (10 == NUMBER_OF_MPU) {
      sensors[RIGHT_LOWER_ARM_INDEX].label = "right_lower_arm";
      sensors[RIGHT_LOWER_ARM_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
      sensors[RIGHT_LOWER_ARM_INDEX].channel = 3;
      sensors[RIGHT_UPPER_LEG_INDEX].label = "right_upper_leg";
      sensors[RIGHT_UPPER_LEG_INDEX].multiplexer = TCA_ADDRESS_RIGHT_SIDE;
      sensors[RIGHT_UPPER_LEG_INDEX].channel = 4;
      sensors[LEFT_LOWER_ARM_INDEX].label = "left_lower_arm";
      sensors[LEFT_LOWER_ARM_INDEX].multiplexer = TCA_ADDRESS_LEFT_SIDE;
      sensors[LEFT_LOWER_ARM_INDEX].channel = 3;
      sensors[LEFT_UPPER_LEG_INDEX].label = "left_upper_leg";
      sensors[LEFT_UPPER_LEG_INDEX].multiplexer = TCA_ADDRESS_LEFT_SIDE;
      sensors[LEFT_UPPER_LEG_INDEX].channel = 4;
    }
    break;
  default:
    Serial.println("");
    Serial.println("---------------------------------------------------");
    Serial.println("can not find a reasonable number of I2C multiplexer");
    Serial.println("---------------------------------------------------");
    // put ESP32 into deep sleep (closest to shutdown)
    esp_deep_sleep_start();
    break;
  }
  Serial.println(".. done");

  // check which MPU9250 board is there and configure them
  checkAndConfigureGyros();

  // ------- CALIBRATION AND SET THE NORTH-------
  // CURRENT PROCESS WITHOUT BUTTON CHOICE

#ifndef BUTTON
  // Calibration of acceleration
  Serial.println("Calibration of acceleration : don't move devices");
  calibration.add("Calibration of acceleration : don't move devices");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  digitalWrite(RED_PIN, HIGH); // Calibrate one by one
  for (uint8_t i = 0; i < NUMBER_OF_MPU; i++) {
    Serial.println(i);
    if (!selectI2cMultiplexerChannel(sensors[i].multiplexer, sensors[i].channel)) {
      Serial.print("could not select channel ");
      Serial.print(sensors[i].channel);
      Serial.print(" on multiplexer at address ");
      Serial.println(sensors[i].multiplexer);
    }
    mpu[i].calibrateAccelGyro();
  }
  digitalWrite(RED_PIN, LOW);
  Serial.println("Acceleration calibration done.");

// Calibration of magnetometer
#ifdef MANUAL_CALIBRATION
  manualMagnetometerCalibration();
#endif

#ifdef AUTO_CALIBRATION
  automaticMagnetometerCalibration();
#endif
  // Blinking light = calibration over
  digitalWrite(YEL_PIN, LOW);
  delay(200);
  digitalWrite(YEL_PIN, HIGH);
  delay(200);
  digitalWrite(YEL_PIN, LOW);
#endif

#ifdef BUTTON
  buttonBasedCalibration();
#endif

  /*
  //Print/send the calibration of magnetometer - USE IT TO AUTO CALIB AGAIN
  Serial.println("Calibration data :");
  for(int i=0; i<NUMBER_OF_MPU; i++) {
    selectI2cMultiplexerChannel(sensors[i].multiplexer, sensors[i].channel);

    //Send calibration data to TouchDesigner
    calibration.add("MAGSCALE").add(i).add(mpu[i].getMagScaleX()).add(mpu[i].getMagScaleY()).add(mpu[i].getMagScaleZ());
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();

    calibration.add("MAGBIAS").add(i).add(mpu[i].getMagBiasX()).add(mpu[i].getMagBiasY()).add(mpu[i].getMagBiasZ());
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();
  }*/
}

/**
 * This main processing loop periodically retrieves the sensor data,
 * cleans it, and passes it on via OSC.
 * 
 * @see setup()
 * @todo check for errors in MPU data update
 * @todo log error remotely
 */
void loop() {

  //--------MPU recording--------
  fetchData();

  // Print values for debugging (with 100ms pauses)
  static unsigned long last_print = 0;
  if (millis() - last_print > 100) {
    for (int i = 0; i < NUMBER_OF_MPU; i++) {
      Serial.print(sensors[i].mpu.getYaw());
      Serial.print("// ");
      Serial.print(sensors[i].mpu.getYaw_r());
      Serial.print("// ");
      Serial.print(sensors[i].mpu.getNorth());
      Serial.print("// ");
      /*
                preferences.begin("0", false);
                Serial.print(preferences.getFloat("accbiasX", 0));
         Serial.print("/"); Serial.print(mpu[i].getAccBiasX());
         Serial.print("_"); Serial.print(preferences.getFloat("accbiasY", 0));
         Serial.print("/"); Serial.print(mpu[i].getAccBiasY());
         Serial.print("_"); Serial.print(preferences.getFloat("accbiasZ", 0));
         Serial.print("/"); Serial.print(mpu[i].getAccBiasZ());
         Serial.print("_"); preferences.end();*/
    }

    Serial.println();
    last_print = millis();
  }

  //-------OSC communication--------
  // Send data in separate message per sensor
  for (size_t i = 0; i < NUMBER_OF_MPU; i++) {
	// skip sesors with problems
	if (!sensors[i].usable) {
       continue;
    }
    // Fill OSC message with data
    body[i].add(mpuData[i].quaternion.x).add(mpuData[i].quaternion.y).add(mpuData[i].quaternion.z).add(mpuData[i].quaternion.w);
    body[i].add(mpuData[i].eulerangle.x).add(mpuData[i].eulerangle.y).add(mpuData[i].eulerangle.z);
    body[i].add(mpuData[i].gyrovalue.x).add(mpuData[i].gyrovalue.y).add(mpuData[i].gyrovalue.z);

    // send data out
    Udp.beginPacket(outIp, outPort);
    body[i].send(Udp);
    Udp.endPacket();

    // clear up message cache
    body[i].empty();
  }
}
