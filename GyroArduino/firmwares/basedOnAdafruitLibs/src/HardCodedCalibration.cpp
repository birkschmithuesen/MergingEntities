#include "HardCodedCalibration.hpp"
#include "CalibData.hpp"

bool HardCodedCalibration::begin(int controllerIndex, int sensorIndex)
{
    this->controllerIndex = controllerIndex;
    this->sensorIndex = sensorIndex;
    return true;
}

bool HardCodedCalibration::loadCalibration()
{
    // look for a matching preset
    for (int i = 0; i < CALIB_N_PRESETS; i++)
    {
        if (calibrationSets[i].controllerIndex == controllerIndex && calibrationSets[i].sensorIndex == sensorIndex)
        {
            Serial.print("Found matching calibration set for controller#");
            Serial.print(controllerIndex);
            Serial.print(" sensor# ");
            Serial.println(sensorIndex);
            for (int o = 0; o < 3; o++)
                accel_zerog[o] = calibrationSets[i].accel_zerog[o];
            for (int o = 0; o < 3; o++)
                gyro_zerorate[o] = calibrationSets[i].gyro_zerorate[o];
            for (int o = 0; o < 3; o++)
                mag_hardiron[o] = calibrationSets[i].mag_hardiron[o];
            for (int o = 0; o < 9; o++)
                mag_softiron[o] = calibrationSets[i].mag_softiron[o];
            mag_field = calibrationSets[i].mag_field;
            return true;
        }
    }
    return false;
}