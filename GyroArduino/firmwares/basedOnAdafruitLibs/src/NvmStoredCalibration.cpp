#include "NvmStoredCalibration.h"
#include "Preferences.h"
#include "SensorNames.hpp"

bool NvmStoredCalibration::begin(Preferences *nvm, int controllerIndex, int sensorIndex)
{
    // build namespace name
    sprintf(presetName, "sensor%d", sensorIndex);

    this->nvm = nvm;
    return true;
}

bool NvmStoredCalibration::loadCalibration()
{
    // open NVM as read-only
    if (!nvm->begin(presetName, true))
    {
        nvm ->end();
        return false ;
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "accel_zerog_%d", o);
        accel_zerog[o] = nvm->getFloat(topic, 0.0);
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "gyro_zerorate_%d", o);
        gyro_zerorate[o] = nvm->getFloat(topic, 0.0);
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "mag_hardiron_%d", o);
        mag_hardiron[o] = nvm->getFloat(topic, 0.0);
    }

    for (int o = 0; o < 9; o++)
    {
        char topic[40];
        sprintf(topic, "mag_softiron_%d", o);
        // default diagonal entries to 1
        if (o == 0 || o == 4 || o == 8)
        {
            mag_softiron[o] = nvm->getFloat(topic, 1.0);
        }
        else
        {
            mag_softiron[o] = nvm->getFloat(topic, 0.0);
        }
    }

    mag_field = nvm->getFloat("mag_field", 50);
    return true;
}


bool NvmStoredCalibration::saveCalibration()
{
    // open NVM as read-write
    if (!nvm->begin(presetName, false))
    {
        nvm ->end();
        return false;
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "accel_zerog_%d", o);
        nvm->putFloat(topic, accel_zerog[o]);
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "gyro_zerorate_%d", o);
        nvm->putFloat(topic, gyro_zerorate[o]);
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "mag_hardiron_%d", o);
        nvm->putFloat(topic, mag_hardiron[o]);
    }

    for (int o = 0; o < 9; o++)
    {
        char topic[40];
        sprintf(topic, "mag_softiron_%d", o);
        nvm->putFloat(topic, mag_softiron[o]);
    }

    nvm->putFloat("mag_field", mag_field);
    return true;
}