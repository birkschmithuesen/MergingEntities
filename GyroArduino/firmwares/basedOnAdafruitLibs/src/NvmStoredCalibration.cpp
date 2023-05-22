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
        Serial.print("!!! nvm.begin failed  in load with preset Name ");
        Serial.println(presetName);
        nvm->end();
        return false;
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
    nvm->end();
    return true;
}

bool NvmStoredCalibration::saveCalibration()
{
    // open NVM as read-write
    if (!nvm->begin(presetName, false))
    {
        Serial.print("!!! nvm.begin failed  in save with preset Name ");
        Serial.println(presetName);
        nvm->end();
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
    nvm->end();
    return true;
}

bool NvmStoredCalibration::printSavedCalibration()
{
    // open NVM as read-write
    if (!nvm->begin(presetName, true))
    {
        Serial.print("!!! nvm.begin failed  in print with preset Name ");
        Serial.println(presetName);
        nvm->end();
        return false;
    }
    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "accel_zerog_%d", o);
        Serial.print(topic);
        Serial.print(":");
        Serial.println(nvm->getFloat(topic, 666.666));
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "gyro_zerorate_%d", o);
        Serial.print(topic);
        Serial.print(":");
        Serial.println(nvm->getFloat(topic, 666.666));
    }

    for (int o = 0; o < 3; o++)
    {
        char topic[40];
        sprintf(topic, "mag_hardiron_%d", o);
        Serial.print(topic);
        Serial.print(":");
        Serial.println(nvm->getFloat(topic, 666.666));
    }

    for (int o = 0; o < 9; o++)
    {
        char topic[40];
        sprintf(topic, "mag_softiron_%d", o);
        Serial.print(topic);
        Serial.print(":");
        Serial.println(nvm->getFloat(topic, 666.666));
    }

    Serial.print("mag_field");
    Serial.print(":");
    Serial.println(nvm->getFloat("mag_field", 666.666));
    nvm->end();
    return true;
}

bool NvmStoredCalibration::receiveCalibration()
{
    uint16_t crc;
    byte b, i;

    while (Serial.available())
    {
        b = Serial.read();
        if (calcount == 0 && b != 117)
        {
            // first byte must be 117
            return false;
        }
        if (calcount == 1 && b != 84)
        {
            // second byte must be 84
            calcount = 0;
            return false;
        }
        // store this byte
        caldata[calcount++] = b;
        if (calcount < 68)
        {
            // full calibration message is 68 bytes
            return false;
        }
        // verify the crc16 check
        crc = 0xFFFF;
        for (i = 0; i < 68; i++)
        {
            crc = crc16_update(crc, caldata[i]);
        }
        if (crc == 0)
        {
            // data looks good, use it
            float offsets[16];
            memcpy(offsets, caldata + 2, 16 * 4);
            accel_zerog[0] = offsets[0];
            accel_zerog[1] = offsets[1];
            accel_zerog[2] = offsets[2];

            gyro_zerorate[0] = offsets[3];
            gyro_zerorate[1] = offsets[4];
            gyro_zerorate[2] = offsets[5];

            mag_hardiron[0] = offsets[6];
            mag_hardiron[1] = offsets[7];
            mag_hardiron[2] = offsets[8];

            mag_field = offsets[9];

            mag_softiron[0] = offsets[10];
            mag_softiron[1] = offsets[13];
            mag_softiron[2] = offsets[14];
            mag_softiron[3] = offsets[13];
            mag_softiron[4] = offsets[11];
            mag_softiron[5] = offsets[15];
            mag_softiron[6] = offsets[14];
            mag_softiron[7] = offsets[15];
            mag_softiron[8] = offsets[12];

            if (!saveCalibration())
            {
                Serial.println("**WARNING** Couldn't save calibration");
            }
            else
            {
                Serial.println("Wrote calibration");
            }
            printSavedCalibration();
            calcount = 0;
            return true;
        }
        // look for the 117,84 in the data, before discarding
        for (i = 2; i < 67; i++)
        {
            if (caldata[i] == 117 && caldata[i + 1] == 84)
            {
                // found possible start within data
                calcount = 68 - i;
                memmove(caldata, caldata + i, calcount);
                return false;
            }
        }
        // look for 117 in last byte
        if (caldata[67] == 117)
        {
            caldata[0] = 117;
            calcount = 1;
        }
        else
        {
            calcount = 0;
        }
    }
    return false;
}

uint16_t NvmStoredCalibration::crc16_update(uint16_t crc, uint8_t a)
{
    int i;
    crc ^= a;
    for (i = 0; i < 8; i++)
    {
        if (crc & 1)
        {
            crc = (crc >> 1) ^ 0xA001;
        }
        else
        {
            crc = (crc >> 1);
        }
    }
    return crc;
}
