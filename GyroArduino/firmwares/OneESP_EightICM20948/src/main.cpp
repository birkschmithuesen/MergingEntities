/** @file
 * This firmware uses one TCA to read data from 8 ICM20948 sensors via
 * I2C. This data is slightly prepocessed passed on via OSC.
 */

#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// SDA and SCL pin of the soft and hard wire mode
#define SDA_PIN 21 /**< I2C data pin (on ESP32) */
#define SCL_PIN 22 /**< I2C clock pin (on ESP32) */

/**
 * Bring the controller into a working state.
 *
 * @see loop()
 */
void setup(void) {
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
