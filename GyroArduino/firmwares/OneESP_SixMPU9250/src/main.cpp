/** @file
 *
 * This code is intended to run on an ESP32 (<a hfref="https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf">datasheet</a>)
 * with a TCA9548A I2C multiplexer and generic MPU9250 sensor boards.
 *
 * @note This code base is the leading one in terms of features and maturity.
 * @todo implement backwards compatibility to 6 MPU version
 * @todo implement I2C switch selection
 * @todo support 10 MPUs
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
#define nbrMpu 6 /**< number of IMU (MPU) (boards) attached to the controller  */

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
#define WIFI_SSID "network name"     /**< SSID / name of the wifi network to use */
#define WIFI_PASS "access password"  /**< password for the wifi network to use */
IPAddress outIp(192, 168, 0, 2);     /**< IP address of the ESP32 */
int localPort = 8888;                /**< source port for UDP communication on ESP32 */
//-------END WIFI SETTINGS--------

//-------MPU SETTINGS AND FUNCTIONS-------
// Parameters of the setup

// Addresses and pin of IMU (MPU-9250) and TCA9548A(=multiplexer)
#define MPU_SWITCH_RIGHT_SIDE 0x68 /**< address of the MPU-9250 when its pin AD0 is low */
#define MPU_SWITCH_LEFT_SIDE 0x69  /**< address of the MPU-9250 when its pin AD0 is high */
#define TCA_ADDRESS 0x70   /**< address of the 8 channel I2C switch */

// SDA and SCL pin of the soft and hard wire mode
#define SDA_PIN 21   /**< I2C data pin (on ESP32) */
#define SCL_PIN 22   /**< I2C clock pin (on ESP32) */

// LED pin for info showing, BUTTON pin for communication
#define RED_PIN 32   /**< ESP pin number of red LED */
#define YEL_PIN 33   /**< ESP pin number of yellow LED */
#define BUTTON_PIN 5 /**< ESP pin number of (callibration) button */

#define MPU_NORTH 1 /**< MPU used to set the north */
float theta = 0;    /**< angle to the north */

int state = HIGH;        /**< last state of the button */
int state_button = LOW;  /**< current state of the button */

// Instance to store data on ESP32, name of the preference
Preferences preferences;  /**< container for preferences on ESP32 */
char mpuPref[10];         /**< preferences of each MPU stored on ESP32 */

// Hardware I2c
MPU9250 mpu[nbrMpu];  /**< software handler/abstraction for each MPU */

// Setting variable
MPU9250Setting setting;  /**< configuration settings of the MPU9250 stored in memory */

// Store data from MPU
float qX[nbrMpu] = {0};  /**< quaternion X value for MPU referenced by index */
float qY[nbrMpu] = {0};  /**< quaternion Y value for MPU referenced by index */
float qZ[nbrMpu] = {0};  /**< quaternion Z value for MPU referenced by index */
float qW[nbrMpu] = {0};  /**< quaternion W value for MPU referenced by index */

float oX[nbrMpu] = {0};  /**< euler angle X axis for MPU referenced by index */
float oY[nbrMpu] = {0};  /**< euler angle Y axis for MPU referenced by index */
float oZ[nbrMpu] = {0};  /**< euler angle Z axis for MPU referenced by index */

float gX[nbrMpu] = {0};  /**< gyroscope value X axis for MPU referenced by index */
float gY[nbrMpu] = {0};  /**< gyroscope value Y axis for MPU referenced by index */
float gZ[nbrMpu] = {0};  /**< gyroscope value Z axis for MPU referenced by index */

float accbias[6][3];     /**< bias/drift/offset profile for the accelerator */
float gyrobias[6][3];    /**< bias/drift/offset profile for the gyroscope */

/**
 * Switch to the given channel on the multiplexer for I2C communication.
 *
 * This function updates the control register in the switch to select one
 * of the eight I2C devices (numbered 0..7) attached to it.
 *
 * @param channel The channel to select to communicate with I2C client
 * @see scanI2C()
 * @todo limit processing to valid values (0..7)
 */
void selectI2cSwitchChannel(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

/**
 * Check to see if I2C device (IMU) can be found / is connected and
 * print the result to the serial output.
 *
 * @see selectI2cSwitchChannel(uint8_t channel)
 * @todo return number of devices found?
 * @todo select switch channel to scan via function argument?
 */
void scanI2C() {
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

  delay(5000); // wait 5 seconds for next scan
}

//-------WIFI SETTINGS AND FUNCTIONS-------
#ifdef BODY_1
int outPort = 8000;  /**< UDP server port on OSC receiver (i.e. central server) */
/** OSC messages with senor values for given part */
OSCMessage body[] = {
    OSCMessage("/body/1/gyro/1/"), OSCMessage("/body/1/gyro/2/"),
    OSCMessage("/body/1/gyro/3/"), OSCMessage("/body/1/gyro/4/"),
    OSCMessage("/body/1/gyro/5/"), OSCMessage("/body/1/gyro/6/")};
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
    OSCMessage("/body/2/gyro/1/"), OSCMessage("/body/2/gyro/2/"),
    OSCMessage("/body/2/gyro/3/"), OSCMessage("/body/2/gyro/4/"),
    OSCMessage("/body/2/gyro/5/"), OSCMessage("/body/2/gyro/6/")};
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
    OSCMessage("/body/3/gyro/1/"), OSCMessage("/body/3/gyro/2/"),
    OSCMessage("/body/3/gyro/3/"), OSCMessage("/body/3/gyro/4/"),
    OSCMessage("/body/3/gyro/5/"), OSCMessage("/body/3/gyro/6/")};
OSCMessage calibration("/calibration/3");  /**< OSC endpoint for calibration messages */
#endif

/**
 * This function establishes a connection to the preconfigured wifi network.
 * 
 * @see #WIFI_SSID
 * @see #WIFI_PASS
 * @see startUdp()
 */
void connectWiFi() {
  Serial.print("Connecting to wifi ..");
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
 * Calibrate the magnetometer (i.e. set north) by manually
 * fiddling with the sensor.
 *
 * @see automaticMagnetometerCalibration()
 * @note This code has been extracted manually and not changed/adjusted
 */
void manualMagnetometerCalibration() {
  Serial.println("Calibration of mag");

  for (uint8_t i = 0; i < nbrMpu; i++) {
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

    selectI2cSwitchChannel(i);
    mpu[i].setMagneticDeclination(MAG_DECLINATION);
    mpu[i].calibrateMag();

    calibration.empty();
  }
}


/**
 * Calibrate the magnetometer (i.e. set north) automatically. This requires
 * detailed knowledge of the stage (layout / setup) and being diligent
 * with the gyro wiring.
 *
 * @see manualMagnetometerCalibration()
 * @note This code has been extracted manually and not changed/adjusted
 */
void automaticMagnetometerCalibration() {
  calibration.add("Automatic calibration of the magnetometer gyro");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  for (uint8_t i = 0; i < nbrMpu; i++) {
    mpu[i].setMagneticDeclination(MAG_DECLINATION);
    mpu[i].setMagBias(magbias[i][0], magbias[i][1], magbias[i][2]);
    mpu[i].setMagScale(magscale[i][0], magscale[i][1], magscale[i][2]);
  }
  Serial.println("Mag calibration done");
}


/**
 * Main setup / initialisation routine.
 * 
 * @see loop()
 */
void setup() {
  float time_passed = 0;  /**< tracking time passed */

  //-------HARDWARE SETUP-------
  Serial.begin(115200);
  Serial.flush(); // Clean buffer

  // LED initialization
  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp(localPort);

  //-------MPU SETUP------
  // Launch comm with multiplexer

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(2000);

  // MPU parameters (sensitivity, etc)
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  // Lauch communication with the 6 MPUs - Switch to channel i and lauch comm
  // with mpu number i
  for (uint8_t i = 0; i < nbrMpu; i++) {
    selectI2cSwitchChannel(i);
    // scanI2C();
    mpu[i].setup(MPU_ADDRESS_1, setting, Wire);
  }

  // Selection of filters
  QuatFilterSel sel{QuatFilterSel::MADGWICK};
  for (uint8_t i = 0; i < nbrMpu; i++) {
    mpu[i].selectFilter(sel);
    mpu[i].setFilterIterations(10);
  }

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
  for (uint8_t i = 0; i < nbrMpu; i++) {
    Serial.println(i);
    selectI2cSwitchChannel(i);
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
  //-------FIRST CHOICE-------Two leds are lighted : you have 6 seconds to press
  //or not to press the button, to launch a calibration process
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, HIGH);
  delay(6000);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  state_button = digitalRead(BUTTON_PIN);
  // state_button = LOW;
  if (state_button == HIGH) {
    Serial.println("HIGH : Launch calibration sequence");
    // Acceleration : get data calibration + calibrate
    Serial.println("Calibration of acceleration : don't move devices");
    calibration.add("Calibration of acceleration : don't move devices");
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();

    digitalWrite(RED_PIN, HIGH);
    for (uint8_t i = 0; i < nbrMpu; i++) {
      Serial.println(i);
      selectI2cSwitchChannel(i);
      mpu[i].calibrateAccelGyro();
    }
    digitalWrite(RED_PIN, LOW);
    Serial.println("Acceleration calibration done.");

    // Acceleration : store calibration data
    for (uint8_t i = 0; i < nbrMpu; i++) {
      itoa(i, mpuPref, 10); // Key names = number of mpu
      preferences.begin(mpuPref, false);

      preferences.putFloat("accbiasX", mpu[i].getAccBiasX());
      preferences.putFloat("accbiasY", mpu[i].getAccBiasY());
      preferences.putFloat("accbiasZ", mpu[i].getAccBiasZ());

      preferences.putFloat("gyrobiasX", mpu[i].getGyroBiasX());
      preferences.putFloat("gyrobiasY", mpu[i].getGyroBiasY());
      preferences.putFloat("gyrobiasZ", mpu[i].getGyroBiasZ());

      preferences.end();
    }

    // Magnetometer : get data calibration + calibrate
    Serial.println("Calibration of mag");

    for (uint8_t i = 0; i < nbrMpu; i++) {
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

      selectI2cSwitchChannel(i);
      mpu[i].setMagneticDeclination(MAG_DECLINATION);
      mpu[i].calibrateMag();

      calibration.empty();
    }
    Serial.println("Calibration of mag done");

    // Magnetometer : store calibration data
    for (uint8_t i = 0; i < nbrMpu; i++) {
      itoa(i, mpuPref, 10); // Key names = number of mpu
      preferences.begin(mpuPref, false);

      preferences.putFloat("magbiasX", mpu[i].getMagBiasX());
      preferences.putFloat("magbiasY", mpu[i].getMagBiasY());
      preferences.putFloat("magbiasZ", mpu[i].getMagBiasZ());

      preferences.putFloat("magscaleX", mpu[i].getMagScaleX());
      preferences.putFloat("magscaleY", mpu[i].getMagScaleY());
      preferences.putFloat("magscaleZ", mpu[i].getMagScaleZ());

      preferences.end();
    }
  }

  // Button not pushed : we read the stored calibration data and calibrate
  else {
    Serial.println("LOW : Load calibration data");
    for (uint8_t i = 0; i < nbrMpu; i++) {
      itoa(i, mpuPref, 10); // Key names = number of mpu
      preferences.begin(mpuPref, false);

      // Set acceleration calibration data

      mpu[i].setAccBias(preferences.getFloat("accbiasX", 0),
                        preferences.getFloat("accbiasY", 0),
                        preferences.getFloat("accbiasZ", 0));
      mpu[i].setGyroBias(preferences.getFloat("gyrobiasX", 0),
                         preferences.getFloat("gyrobiasY", 0),
                         preferences.getFloat("gyrobiasZ", 0));

      // Test to see what the fuck
      /*
      digitalWrite(RED_PIN, HIGH);
      for(uint8_t i=0; i<nbrMpu; i++) {
        Serial.println(i);
        selectI2cSwitchChannel(i);
        mpu[i].calibrateAccelGyro();
      }
      digitalWrite(RED_PIN, LOW);
      Serial.println("Acceleration calibration done.");*/

      // Set magnetometer calibration data
      mpu[i].setMagBias(preferences.getFloat("magbiasX", 0),
                        preferences.getFloat("magbiasY", 0),
                        preferences.getFloat("magbiasZ", 0));
      mpu[i].setMagScale(preferences.getFloat("magscaleX", 0),
                         preferences.getFloat("magscaleY", 0),
                         preferences.getFloat("magscaleZ", 0));
      mpu[i].setMagneticDeclination(MAG_DECLINATION);
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

  //-------SECOND CHOICE------- Two leds are lighted : you have 10 seconds to
  //orientate the MPU 1 over the new north and press the button
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, HIGH);

  // We let the mpu run for 10 seconds to have the good yaw if we need it
  selectI2cSwitchChannel(MPU_NORTH - 1);

  time_passed = millis();
  while (millis() - time_passed < 10000) {
    mpu[MPU_NORTH - 1].update();
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  state_button = digitalRead(BUTTON_PIN);
  // state_button = HIGH;
  if (state_button == HIGH) {
    Serial.println("HIGH : setting north");
    // We save the north direction and send it to the library
    theta = mpu[MPU_NORTH - 1].getYaw() * (-1);

    preferences.begin("setNorth", false);
    preferences.putFloat("north", theta);
    preferences.end();
  } else {
    Serial.println("LOW : Load former north");
  }

  // We get the north and set it
  preferences.begin("setNorth", false);
  for (uint8_t i = 0; i < nbrMpu; i++) {
    mpu[i].setNorth(preferences.getFloat("north", 0));
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
#endif

  /*
  //Print/send the calibration of magnetometer - USE IT TO AUTO CALIB AGAIN
  Serial.println("Calibration data :");
  for(int i=0; i<nbrMpu; i++) {
    selectI2cSwitchChannel(i);

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
 */
void loop() {

  //--------MPU recording--------

  // fetch data from each MPU
  for (uint8_t i = 0; i < nbrMpu; i++) {
    selectI2cSwitchChannel(i);
    mpu[i].update();
  }

  // Print values for debugging (with 100ms pauses)
  static unsigned long last_print = 0;
  if (millis() - last_print > 100) {
    for (int i = 0; i < nbrMpu; i++) {
      Serial.print(mpu[i].getYaw());
      Serial.print("// ");
      Serial.print(mpu[i].getYaw_r());
      Serial.print("// ");
      Serial.print(mpu[i].getNorth());
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

  // Store values
  for (int i = 0; i < nbrMpu; i++) {
    qX[i] = mpu[i].getQuaternionX();
    qY[i] = mpu[i].getQuaternionY();
    qZ[i] = mpu[i].getQuaternionZ();
    qW[i] = mpu[i].getQuaternionW();

    oX[i] = mpu[i].getEulerX();
    oY[i] = mpu[i].getEulerY();
    oZ[i] = mpu[i].getEulerZ();

    gX[i] = mpu[i].getGyroX();
    gY[i] = mpu[i].getGyroY();
    gZ[i] = mpu[i].getGyroZ();
  }

  //-------OSC communication--------
  // Send data in 6 separated messages, working okay

  for (int i = 0; i < nbrMpu; i++) {
    body[i].add(qX[i]).add(qY[i]).add(qZ[i]).add(qW[i]); // Fill OSC message with data
    body[i].add(oX[i]).add(oY[i]).add(oZ[i]);
    body[i].add(gX[i]).add(gY[i]).add(gZ[i]);

    Udp.beginPacket(outIp, outPort);
    body[i].send(Udp);
    Udp.endPacket();

    body[i].empty();
  }
}
