/** @file
 * This firmware uses one TCA9548A to read data from 8 ICM20948 sensors
 * via I2C. This data is slightly prepocessed passed on via OSC.
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
 * A data structure to handle hardware related data and
 * communication of one ICM20948.
 */
struct ICM20948socket {
  const char *label; /**< human readable identification of the sensor (for OSC path) */
  uint8_t channel; /**< channel used on the I2C multiplexer */
  Adafruit_ICM20948 sensor; /**< software handler/abstraction for ICM20948 at given channel */
  bool usable = false; /**< indicate that sensor (data) is present and no errors occured */
};

// IOBundle iobundle[NUMBER_OF_MPU];
ICM20948socket socket[NUMBER_OF_SENSORS]; /**< a (global) list of sockets to bundle communication */

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
 * Bring the controller into a working state.
 *
 * @see loop()
 * @see selectI2cMultiplexerChannel(uint8_t channel)
 */
void setup(void) {
  uint8_t result = 0;     // track results/error codes
  uint8_t nonworking = 0; // track number of non-working sensors
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
  for (uint8_t ch = 0; ch < NUMBER_OF_SENSORS; ch++) {
    Serial.print("checking for serial communication for sensor on channel ");
    Serial.print(ch);
    Serial.print(" .");
    if (!selectI2cMultiplexerChannel(ch)) {
      Serial.println(".. failed (channel selection)");
      nonworking += 1;
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
    Serial.print("found ");
    Serial.print(NUMBER_OF_SENSORS - nonworking);
    Serial.println(" sensors");

    // check for correct sensor communication
    nonworking = 0;
    Serial.print("checking sensor on channel ");
    Serial.print(ch);
    Serial.print(" (");
    Serial.print(socket[ch].label);
    Serial.print(") .");
    if (socket[ch].sensor.begin_I2C()) {
      socket[ch].usable = true;
      Serial.println(".. works");
    } else {
      nonworking += 1;
      Serial.println(".. failed");
    }
    Serial.print("can communicate with ");
    Serial.print(NUMBER_OF_SENSORS - nonworking);
    Serial.println(" sensors");
  }
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
}
