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

/**
 * Bring the controller into a working state.
 *
 * @see loop()
 */
void setup(void) {
  uint8_t result = 0; // track results/error codes
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

  // set up I2C communication
  Serial.print("setting up I2C pins .");
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);
  Serial.println(".. done");

  // check of one TCA9548A
  Serial.print("searching TCA9548A .");
  Wire.beginTransmission(TCA_ADDRESS);
  result = Wire.endTransmission(true);
  switch (result) {
  case 0:
    Serial.println(".. found");
    break;
  case 5:
    Serial.println(".. failed (I2C bus timeout)");
    break;
  default:
    Serial.println(".. failed");
    break;
  }
}

/**
 * The main processing loop.
 *
 * @see setup()
 */
void loop() {
  Serial.print(".");
  delay(1000);
}
