/** @file */
/*
 * This code is intended to run on an ESP32 (<a hfref="https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf">datasheet</a>)
 */
// Connections : Please always connect 2 hard mpu (builtin I2C bus) to your specified pins
// Indicate the numbers of hard, soft I2C, connect the soft I2C in the order of the specified pins
// Specify your IP address and Wifi
// Mag calibration desactivated right now, see if it's usefull Eventually speed up if TCA is fast enough

// Attention : select the right channel with the TCA function (+2 woth prototype in wood, i regular without)

//-------LIBRARIES-------
// Library to use Arduino cmd
#include <Arduino.h>
// Libraries for communication
#include <OSCMessage.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
// MPU library
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Arduino.h>

//-------GENERAL SETTINGS-------
#define nbrMpu 6 /**< number of IMU (MPU?) (boards) attached to the controller */
// Select network to connnect
#define KAESIMIRA
// #define THEATER

// Define the number of the body : 1, 2 or 3
#define BODY_1

// Define the calibration mode : MANUAL_CALIBRATION for manual, AUTO_CALIBRATION otherwise
#define AUTO_CALIBRATION

// Define BUTTON to activate the button
#define BUTTON
//-------END GENERAL SETTINGS-------

//-------MPU SETTINGS AND FUNCTIONS-------
// Parameters of the setup

// Addresses and pin of MPU and TCA9548A(=multiplexer)
#define MPU_ADDRESS_1 0x68 // Set 0x68 or 0x69
#define MPU_ADDRESS_2 0x69 // Set 0x68 or 0x69

#define TCA_ADDRESS 0x70 /**< address of the 8 channel I2C switch */

// Rate of filtering
#define FILTER_UPDATE_RATE_HZ 25  /**< update rate of the IMU sensor data filter */

// LED pin for info showing, BUTTON pin for communication
#define RED_PIN 32   /**< ESP pin number of red LED */
#define YEL_PIN 33   /**< ESP pin number of yellow LED */
#define BUTTON_PIN 5 /**< ESP pin number of (callibration) button */

#define MPU_NORTH 1 /**< MPU used to set the north */

// angle to the north
float theta = 0;
float north = 0;
float halfcos = 0;
float halfsin = 0;

float time_converge = 0;
float last_update = 0;

uint32_t timestamp;

int state = HIGH;
int state_button = LOW;

// Instance to store data on ESP32, name of the preference
Preferences preferences;
char mpuPref[10];

// Hardware I2C: MPU data containers
Adafruit_Sensor *accelerometer[nbrMpu], *gyroscope[nbrMpu], *magnetometer[nbrMpu];
#include "NXP_FXOS_FXAS.h" // NXP 9-DoF breakout //We declare it here to allow the .h to use the previous variables

Adafruit_NXPSensorFusion filter[nbrMpu];  /**< filters for the sensor data, one for each sensor */
Adafruit_Sensor_Calibration_EEPROM cal[nbrMpu];  /**< calibration settings for each IMU sensor */
sensors_event_t accel[nbrMpu];  /**< last acceleration event for each IMU */
sensors_event_t gyro[nbrMpu];   /**< last gyro event for each IMU */
sensors_event_t mag[nbrMpu];    /**< last magnetic event for each IMU */

// To change sensibility, call "cal.<rightfunction>

// Store data from MPU
float qX[nbrMpu] = {0};
float qY[nbrMpu] = {0};
float qZ[nbrMpu] = {0};
float qW[nbrMpu] = {0};

float qX_r[nbrMpu] = {0};
float qY_r[nbrMpu] = {0};
float qZ_r[nbrMpu] = {0};
float qW_r[nbrMpu] = {0};

float oX[nbrMpu] = {0};
float oY[nbrMpu] = {0};
float oZ[nbrMpu] = {0};

float gX[nbrMpu] = {0};
float gY[nbrMpu] = {0};
float gZ[nbrMpu] = {0};

// Useless ?
float accbias[6][3];
float gyrobias[6][3];

/**
 * Switch to the given channel on the multiplexer for I2C communication.
 *
 * This function updates the control register in the switch to select one
 * of the eight I2C devices (numbered 0..7) attached to it.
 *
 * @param channel The channel to select to communicate with I2C client
 * @todo limit processing to valid values (0..7)
 */
void selectI2cSwitchChannel(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

/**
 * Get update events from the sensors and update (global) variables.
 *
 * @param i ID/number/index of the senor
 * @todo adjust type of i
 * @todo limit i to sensible values
 */
void fetchSensorUpdate(int i) {
  selectI2cSwitchChannel(i);

  // get raw data
  accelerometer[i]->getEvent(&accel[i]);
  gyroscope[i]->getEvent(&gyro[i]);
  magnetometer[i]->getEvent(&mag[i]);

  // substract the offset - CALIBRATION
  cal[i].calibrate(mag[i]);
  cal[i].calibrate(accel[i]);
  cal[i].calibrate(gyro[i]);

  // Gyroscope needs to be converted from rad/s to degree/s
  // the rest are not unit-important
  gX[i] = gyro[i].gyro.x * SENSORS_RADS_TO_DPS;
  gY[i] = gyro[i].gyro.y * SENSORS_RADS_TO_DPS;
  gZ[i] = gyro[i].gyro.z * SENSORS_RADS_TO_DPS;

  // update the SensorFusion filter - QUATERNION
  filter[i].update(gX[i], gY[i], gZ[i], accel[i].acceleration.x,
                   accel[i].acceleration.y, accel[i].acceleration.z,
                   mag[i].magnetic.x, mag[i].magnetic.y, mag[i].magnetic.z);
}

/**
 * Set proper global offsets to correctly indicate north.
 *
 * @param n angle to true north in degrees
 */
void setNorth(const float n) {
  // update global variables
  north = n * DEG_TO_RAD;
  halfcos = cos(north / 2);
  halfsin = sin(north / 2);
}

/**
 * Check to see if I2C device (IMU) can be found / is connected and
 * print the result to the serial output.
 *
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

// Compute angles from quaternions
float rpy[3];
void update_rpy(float qw, float qx, float qy, float qz) {
  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll. For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
  float a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler
                                 // angles and gravity components
  a12 = 2.0f * (qx * qy + qw * qz);
  a22 = qw * qw + qx * qx - qy * qy - qz * qz;
  a31 = 2.0f * (qw * qx + qy * qz);
  a32 = 2.0f * (qx * qz - qw * qy);
  a33 = qw * qw - qx * qx - qy * qy + qz * qz;
  rpy[0] = atan2f(a31, a33);
  rpy[1] = -asinf(a32);
  rpy[2] = atan2f(a12, a22);
  rpy[0] *= 180.0f / PI;
  rpy[1] *= 180.0f / PI;
  rpy[2] *= 180.0f / PI;
  if (rpy[2] >= +180.f)
    rpy[2] -= 360.f;
  else if (rpy[2] < -180.f)
    rpy[2] += 360.f;
}

//-------WIFI SETTINGS AND FUNCTIONS-------
// Settings to connect to WiFi

WiFiUDP Udp;

#ifdef KAESIMIRA
#define WIFI_SSID "ArtNet4Hans"
#define WIFI_PASS "kaesimira"
IPAddress outIp(192, 168, 0, 2); // IP of the computer
int localPort = 8888;            // Port of ESP
#endif

#ifdef THEATER
#define WIFI_SSID "TheaterDo-GAST"
#define WIFI_PASS "theaterdortmund"
IPAddress outIp(192, 168, 193, 221); // IP of the computer
int localPort = 8888;                // Port of ESP
#endif

#ifdef BODY_1
int outPort = 8000; // Port on PC
OSCMessage body[] = {
    OSCMessage("/body/1/gyro/1/"), OSCMessage("/body/1/gyro/2/"),
    OSCMessage("/body/1/gyro/3/"), OSCMessage("/body/1/gyro/4/"),
    OSCMessage("/body/1/gyro/5/"), OSCMessage("/body/1/gyro/6/")};
OSCMessage calibration("/calibration/1");
#endif

#ifdef BODY_2
int outPort = 8001; // Port on PC
OSCMessage body[] = {
    OSCMessage("/body/2/gyro/1/"), OSCMessage("/body/2/gyro/2/"),
    OSCMessage("/body/2/gyro/3/"), OSCMessage("/body/2/gyro/4/"),
    OSCMessage("/body/2/gyro/5/"), OSCMessage("/body/2/gyro/6/")};
OSCMessage calibration("/calibration/2");
#endif

#ifdef BODY_3
int outPort = 8002; // Port on PC
OSCMessage body[] = {
    OSCMessage("/body/3/gyro/1/"), OSCMessage("/body/3/gyro/2/"),
    OSCMessage("/body/3/gyro/3/"), OSCMessage("/body/3/gyro/4/"),
    OSCMessage("/body/3/gyro/5/"), OSCMessage("/body/3/gyro/6/")};
OSCMessage calibration("/calibration/3");
#endif

// Function to connect WiFi
void connectWiFi() // Let's connect a WiFi
{
  Serial.print("Starting WiFi connection ...");

  WiFi.mode(WIFI_STA); // Mode of the WiFi, STA = STATION MODE (connect to stg),
                       // APM = Access Point Mode (create a network)
  WiFi.begin(WIFI_SSID, WIFI_PASS); // Connecting the ESP

  long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    Serial.print(".");
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("__Not connected__");
  } else {
    Serial.print("__Connected. IP adress : ");
    Serial.println(WiFi.localIP());
  }
}

/**
 * Set up UDP communication locally.
 */
void startUdp() {
  Serial.print("Starting UDP connection to local port ");
  Udp.begin(localPort); // set the connection with the computer
  Serial.println(localPort);
}

//-------SETUP-------
void setup() {
  //-------HARDWARE SETUP-------
  Serial.begin(115200);
  Serial.flush(); // clear buffer

  // LED initialization
  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  //-------WIFI SETUP-------
  connectWiFi();
  startUdp();

  //-------MPU SETUP------
  // Initialize all gyros
  for (int i = 0; i < nbrMpu; i++) {
    selectI2cSwitchChannel(i);
    delay(100);

    if (!init_sensors(i)) {
      Serial.print("Desoo");
    }

    else {
      Serial.print("Strrange success");
    }
    filter[i].begin(FILTER_UPDATE_RATE_HZ);
  }

  // ------- CALIBRATION AND SET THE NORTH--------
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
  for (int i = 0; i < nbrMpu; i++) {
    Serial.println(i);
    selectI2cSwitchChannel(i);
    mpu[i].calibrateAccelGyro();
  }
  digitalWrite(RED_PIN, LOW);
  Serial.println("Acceleration calibration done.");

// Calibration of magnetometer

// Manual calibration : get ready to turn the mpu
#ifdef MANUAL_CALIBRATION
  Serial.println("Calibration of mag");

  for (int i = 0; i < nbrMpu; i++) {
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
#endif

#ifdef AUTO_CALIBRATION
  // Automatic calibration : With data of the stage. Respect the gyro wiring
  calibration.add("Automatic calibration of the magnetometer gyro");
  Udp.beginPacket(outIp, outPort);
  calibration.send(Udp);
  Udp.endPacket();
  calibration.empty();

  for (int i = 0; i < nbrMpu; i++) {
    mpu[i].setMagneticDeclination(MAG_DECLINATION);
    mpu[i].setMagBias(magbias[i][0], magbias[i][1], magbias[i][2]);
    mpu[i].setMagScale(magscale[i][0], magscale[i][1], magscale[i][2]);
  }
  Serial.println("Mag calibration done.");
#endif
  // Blinking light= calib over
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
  /*
  if(state_button == HIGH){
    Serial.println("HIGH : Launch calibration sequence");
    //Acceleration : get data calibration + calibrate
    Serial.println("Calibration of acceleration : don't move devices");
    calibration.add("Calibration of acceleration : don't move devices");
    Udp.beginPacket(outIp, outPort);
    calibration.send(Udp);
    Udp.endPacket();
    calibration.empty();

    digitalWrite(RED_PIN, HIGH);
    for(int i=0; i<nbrMpu; i++) {
      Serial.println(i);
      selectI2cSwitchChannel(i);
      mpu[i].calibrateAccelGyro();
    }
    digitalWrite(RED_PIN, LOW);
    Serial.println("Acceleration calibration done.");

    // Acceleration : store calibration data
    for(int i=0; i<nbrMpu; i++){
      itoa(i,mpuPref,10); //Key names = number of mpu
      preferences.begin(mpuPref, false);

      preferences.putFloat("accbiasX", mpu[i].getAccBiasX());
      preferences.putFloat("accbiasY", mpu[i].getAccBiasY());
      preferences.putFloat("accbiasZ", mpu[i].getAccBiasZ());

      preferences.putFloat("gyrobiasX", mpu[i].getGyroBiasX());
      preferences.putFloat("gyrobiasY", mpu[i].getGyroBiasY());
      preferences.putFloat("gyrobiasZ", mpu[i].getGyroBiasZ());

      preferences.end();
    }
  }*/

  // Button not pushed -> in any case here : we read the stored calibration data and calibrate
  Serial.println("LOW : Load calibration data");
  for (int i = 0; i < nbrMpu; i++) {

    // For the future : each imu got his own calibration
    itoa(i, mpuPref, 10); // Key names = number of mpu

    preferences.begin(mpuPref, false);

    // Set acceleration & gyro calibration data

    cal[i].accel_zerog[0] = float(preferences.getChar("accbiasX", 0)) / 100;
    cal[i].accel_zerog[1] = float(preferences.getChar("accbiasY", 0)) / 100;
    cal[i].accel_zerog[2] = float(preferences.getChar("accbiasZ", 0)) / 100;
    Serial.println(cal[i].accel_zerog[0]);
    Serial.println(cal[i].accel_zerog[1]);
    Serial.println(cal[i].accel_zerog[2]);

    cal[i].gyro_zerorate[0] =
        float(preferences.getChar("gyrobiasX", 0.0001)) / 100;
    cal[i].gyro_zerorate[1] =
        float(preferences.getChar("gyrobiasY", 0.0001)) / 100;
    cal[i].gyro_zerorate[2] =
        float(preferences.getChar("gyrobiasZ", 0.0001)) / 100;
    Serial.println(cal[i].gyro_zerorate[0]);
    Serial.println(cal[i].gyro_zerorate[1]);
    Serial.println(cal[i].gyro_zerorate[2]);

    // Set mag calibration data
    cal[i].mag_field = preferences.getFloat("magfield", 0);

    cal[i].mag_hardiron[0] = preferences.getFloat("maghard0", 0);
    cal[i].mag_hardiron[1] = preferences.getFloat("maghard1", 0);
    cal[i].mag_hardiron[2] = preferences.getFloat("maghard2", 0);

    cal[i].mag_softiron[0] = preferences.getFloat("magsoft0", 0);
    cal[i].mag_softiron[1] = preferences.getFloat("magsoft1", 0);
    cal[i].mag_softiron[2] = preferences.getFloat("magsoft2", 0);
    cal[i].mag_softiron[3] = preferences.getFloat("magsoft3", 0);
    cal[i].mag_softiron[4] = preferences.getFloat("magsoft4", 0);
    cal[i].mag_softiron[5] = preferences.getFloat("magsoft5", 0);
    cal[i].mag_softiron[6] = preferences.getFloat("magsoft6", 0);
    cal[i].mag_softiron[7] = preferences.getFloat("magsoft7", 0);
    cal[i].mag_softiron[8] = preferences.getFloat("magsoft8", 0);

    preferences.end();
  }
  Serial.println("Calibration loaded");

  // Two leds are blinking, saying calibration is over
  for (int i = 0; i < 20; i++) {
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

  time_converge = millis();
  last_update = millis();
  while (millis() - time_converge < 10000) {
    if (millis() - last_update > 1000 / FILTER_UPDATE_RATE_HZ) {
      fetchSensorUpdate(MPU_NORTH - 1);
      last_update = millis();
    }
  }

  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);

  state_button = digitalRead(BUTTON_PIN);
  if (state_button == HIGH) {
    Serial.println("HIGH : setting north");
    // We save the north direction and send it to the library
    theta = filter[MPU_NORTH - 1].getYaw() * (-1);

    preferences.begin("setNorth", false);
    preferences.putFloat("north", theta);
    preferences.end();
  } else {
    Serial.println("LOW : Load former north");
  }

  // We get the north and set it for everyone
  preferences.begin("setNorth", false);

  setNorth(preferences.getFloat("north", 0));

  preferences.end();
  Serial.println("North set");

  // Two leds are blinking, saying north is set
  for (int i = 0; i < 20; i++) {
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

  Wire.setClock(400000);
}

void loop() {

  //--------MPU recording--------

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();

  for (int i = 0; i < nbrMpu; i++) {
    // Update data
    fetchSensorUpdate(i);

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
  }

  Serial.println();

  // Store values

  for (int i = 0; i < nbrMpu; i++) {
    // Quaternions storage
    filter[i].getQuaternion(&qW[i], &qX[i], &qY[i], &qZ[i]);

    // Rotated quaternions storage
    qW_r[i] = qW[i] * halfcos - qZ[i] * halfsin;
    qX_r[i] = qX[i] * halfcos - qY[i] * halfsin;
    qY_r[i] = qY[i] * halfcos + qX[i] * halfsin;
    qZ_r[i] = qZ[i] * halfcos + qW[i] * halfsin;

    // Angles
    oX[i] = filter[i].getPitch();
    oY[i] = filter[i].getRoll();
    oZ[i] = filter[i].getYaw();

    // Gyro speeds are already stored right in update
  }

  // TO CHECK IF ROTATION WORK : we look the angles from the rotated quat :
  // update_rpy(qW[0], qX[0], qY[0], qZ[0]);

  //-------OSC communication--------
  // Send data in 6 separated messages

  for (int i = 0; i < nbrMpu; i++) {
    body[i].add(qX[i]).add(qY[i]).add(qZ[i]).add(
        qW[i]); // Fill OSC message with data
    body[i].add(oX[i]).add(oY[i]).add(oZ[i]);
    body[i].add(gX[i]).add(gY[i]).add(gZ[i]);

    Udp.beginPacket(outIp, outPort);
    body[i].send(Udp);
    Udp.endPacket();

    body[i].empty();
  }
}
