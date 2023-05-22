#pragma once
#include "UserInterface.hpp"
#include "NvmStoredCalibration.h"
#include "HardCodedCalibration.hpp"
#include "I2CMultiplexer.hpp"
#include "Preferences.h"

#define USE_HARD_CODED_CALIBRATION


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
    void calibrateSensor(int sensorId, Imu* imu);


    #endif
};