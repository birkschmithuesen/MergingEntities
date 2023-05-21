#include "I2CMultiplexer.hpp"
#include <HardwareSerial.h>
bool I2CMultiplexer::testConnection()
{
    // see if (expected) multiplexer is there
    Serial.print("searching TCA9548A .");
    Wire.beginTransmission(TCA_ADDRESS);
    int result = Wire.endTransmission(true);
    switch (result)
    {
    case 0:
        Serial.println(".. found");
        return true;
        break;
    case 5:
        Serial.println(".. failed (I2C bus timeout)");
        // put ESP32 into deep sleep (closest to shutdown)
        // esp_deep_sleep_start();
        break;
    default:
        Serial.print(".. failed (error ");
        Serial.print(result);
        Serial.println(")");
        // put ESP32 into deep sleep (closest to shutdown)
        // esp_deep_sleep_start();
        break;
    }
    return false;
}

bool I2CMultiplexer::selectI2cMultiplexerChannel(uint8_t channel)
{
    bool result = false;
    if (channel > 7)
    {
        // channel out of (hardware) range
        return false;
    }
    // select the multiplexer by its hardware address
    Wire.beginTransmission(TCA_ADDRESS);
    // select a channel on the multiplexer
    if (1 == Wire.write(1 << channel))
    {
        result = true;
    }
    Wire.endTransmission();
    return result;
}
