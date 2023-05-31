#pragma once
#include "UserInterface.hpp"
#include "NvmStoredCalibration.h"
#include "HardCodedCalibration.hpp"
#include "I2CMultiplexer.hpp"
#include "Preferences.h"

//#define USE_HARD_CODED_CALIBRATION


class Imu;// prototype
class MultiplexedImus;
class CalibrationManager{
public:
    void setup(Preferences* nvm,int controllerId,MultiplexedImus* imus);
    MultiplexedImus* imus;
#ifdef USE_HARD_CODED_CALIBRATION
    HardCodedCalibration calibrations[N_SENSORS] ; // these are loaded on setup
#else
    NvmStoredCalibration calibrations[N_SENSORS] ; // these are loaded from flash

    void calibrateSequence();
    void calibrateGyros();
    void calibrateMagnetometerInteractive(int sensorId, Imu* imu);
    void calibrateMagnetometer(int sensorId, Imu* imu);
Preferences* nvm;
    // delete complete NVS if you have unknown namespaces (brings back around 629 free key pairs)
    void resetNVS()
    {
        nvs_flash_erase(); // erase the NVS partition and...
        delay(200); // weird nvm storage bug otherwise
        nvs_flash_init();  // initialize the NVS partition.
        delay(200);        // weird nvm storage bug otherwise
    }
    #endif
};