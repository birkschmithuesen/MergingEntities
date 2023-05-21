#pragma once
#include <Wire.h>
#include <Stream.h>
#define TCA_ADDRESS 0x70 /**< I2C address of the TCA9548A (I2C multiplexer) */
class I2CMultiplexer
{
    public:
    static bool testConnection();
    static bool selectI2cMultiplexerChannel(uint8_t channel);
};