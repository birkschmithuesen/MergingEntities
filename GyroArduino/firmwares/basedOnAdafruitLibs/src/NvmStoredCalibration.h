#pragma once
#include <Adafruit_Sensor_Calibration.h>
// Uses data from Esp32 NVM
#include "Preferences.h"
#include <nvs_flash.h>

class NvmStoredCalibration : public Adafruit_Sensor_Calibration
{
public:
    bool begin(Preferences *nvm, int controllerIndex, int sensorIndex);

    bool saveCalibration(void); /**< accesses NVM to set Values */
    bool loadCalibration(void); /**< accesses NVM to get Values */
    bool printSavedCalibration(void); // prints values in NVM
    bool printCurrentCalibration(void); // prints values in RAM
    // Infrastructure for receiving Motioncal Packets from Adafruit imucal example
    bool receiveCalibration(); // returns true if Motioncal packet was received

private:
    int controllerIndex = 0;
    int sensorIndex = 0;

    char presetName[10];
    Preferences *nvm;

    // Infrastructure for receiving Motioncal Packets from Adafruit imucal example
    uint16_t crc16_update(uint16_t crc, uint8_t a);
    byte caldata[68]; // buffer to receive magnetic calibration data from motioncal
    byte calcount = 0;

};