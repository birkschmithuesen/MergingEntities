#include "CalibrationManager.hpp"
#include "MultiplexedImu.hpp"
#include "HardwareSerial.h"
void CalibrationManager::setup(Preferences *nvm, int controllerId, MultiplexedImus *imus)
{
    this->imus = imus;
    this->nvm=nvm;
    for (int i = 0; i < N_SENSORS; i++)
    {
#ifdef USE_HARD_CODED_CALIBRATION
        // get calibration info
        HardCodedCalibration *curCalib = &(calibrations[i]);
        curCalib->begin(controllerId, i);
#else
        Serial.print("setting up Calib Set no.");
        Serial.println(i);
        NvmStoredCalibration *curCalib = &(calibrations[i]);
        curCalib->begin(nvm, controllerId, i);
#endif
        // get calibration info

        curCalib->loadCalibration();
    }
}
#ifndef USE_HARD_CODED_CALIBRATION

void CalibrationManager::calibrateSequence()
{
    Serial.println("Old calib values:");
    for (int curSensorId = 0; curSensorId < N_SENSORS; curSensorId++)
    {
        calibrations[curSensorId].printCurrentCalibration();
    }

    Serial.println("Starting calibration sequence.");
    Serial.println("Calibrating Gyro bias - don't move!");


    resetNVS(); // clear storage to make space for saving nvs data
    calibrateGyros();
    digitalWrite(PIN_BLUELED, HIGH); // Blue LED flash: ready to calibrate next sensor

    // wait until button is pressed again
    uint32_t nCycles=0;
    while (!UserInterface::getCalibrationButtonState())
    {
        delay(10);
        nCycles++;
        if(nCycles>200) return;
    }
    delay(200); // debounce
    // wait until button is released again
    nCycles=0;
    while (UserInterface::getCalibrationButtonState())
    {
        delay(10);
        nCycles++;
    }
    // if button was pressed less than one second, skip magnetometer calib
    if(nCycles<100)return;
    delay(200); // debounce
    
    digitalWrite(PIN_BLUELED, LOW);
    for (int curSensorId = 0; curSensorId < N_SENSORS; curSensorId++)
    {

        calibrateMagnetometerInteractive(curSensorId, &imus->imus[curSensorId]);
    }
    digitalWrite(PIN_BLUELED, LOW);
}

void CalibrationManager::calibrateMagnetometerInteractive(int sensorId, Imu *imu)
{
    Serial.print("Calibrating Sensor no");
    Serial.println(sensorId);

    bool calibrationFinished = false;

    // blue LED off, green flashing: calibrating sensor no (nGreenFlashes)
    digitalWrite(PIN_BLUELED, LOW);
    UserInterface::blinkFlashes = sensorId + 1;
    UserInterface::currentBlinkMode = UserInterface::BlinkModeCalibrate;

    // wait for button to be released
    while (UserInterface::getCalibrationButtonState())
    {
        delay(10);
    }
    delay(500); // wait for button to no longer bounce.

    NvmStoredCalibration *curCalib = &(calibrations[sensorId]);
    I2CMultiplexer::selectI2cMultiplexerChannel(sensorId);

    // run calibration until either button is pressed, or calibration info has been received
    while (!calibrationFinished && !UserInterface::getCalibrationButtonState())
    {
        imu->sendMotionCal();
        calibrationFinished = curCalib->receiveCalibration();
    }
    digitalWrite(PIN_BLUELED, HIGH); // Blue LED continous== calibration received;

    // Blue LED flash== calibration aborted, skipping to next;
    if (!calibrationFinished)
    {
        Serial.println("Calibration aborted.");
        delay(500);
        digitalWrite(PIN_BLUELED, LOW);
    }
    else
    {
        Serial.println("Calibration data received and saved.");
        // if calibration was not aborted, wait for button to be pressed
        while (!UserInterface::getCalibrationButtonState())
        {
            delay(10);
        }
        delay(200); // debounce
    }
    Serial.println("Waiting for button to be released.");
    // wait for button to be released
    while (UserInterface::getCalibrationButtonState())
    {
        delay(10);
    }
    delay(200); // debounce
    UserInterface::currentBlinkMode = UserInterface::BlinkModeNormal;
}
void CalibrationManager::calibrateMagnetometer(int sensorId, Imu *imu)
{
    // run calibration until calibration info has been received
    NvmStoredCalibration *curCalib = &(calibrations[sensorId]);
    I2CMultiplexer::selectI2cMultiplexerChannel(sensorId);
    bool calibrationFinished = false;
    while (!calibrationFinished)
    {
        imu->sendMotionCal();
        calibrationFinished = curCalib->receiveCalibration();
    }
}

void CalibrationManager::calibrateGyros()
{
    // update gyro zero rate with an average value of a few reads
    float meanGx[N_SENSORS];
    float meanGy[N_SENSORS];
    float meanGz[N_SENSORS];

    for (int sensorIndex = 0; sensorIndex < N_SENSORS; sensorIndex++)
    {
        meanGx[sensorIndex] = 0;
        meanGy[sensorIndex] = 0;
        meanGz[sensorIndex] = 0;
    }

    int nAverageCycles = 32;
    for (int i = 0; i < nAverageCycles; i++)
    {
        for (int sensorIndex = 0; sensorIndex < N_SENSORS; sensorIndex++)
        {
            I2CMultiplexer::selectI2cMultiplexerChannel(sensorIndex);
            imus->imus[sensorIndex].readSensor();
            meanGx[sensorIndex] += imus->imus[sensorIndex].gyro_event.gyro.x;
            meanGy[sensorIndex] += imus->imus[sensorIndex].gyro_event.gyro.y;
            meanGz[sensorIndex] += imus->imus[sensorIndex].gyro_event.gyro.z;
        }
        delay(10); // 100Hz update rate
    }

    for (int sensorIndex = 0; sensorIndex < N_SENSORS; sensorIndex++)
    {
        calibrations[sensorIndex].gyro_zerorate[0] = meanGx[sensorIndex] / nAverageCycles;
        calibrations[sensorIndex].gyro_zerorate[1] = meanGy[sensorIndex] / nAverageCycles;
        calibrations[sensorIndex].gyro_zerorate[2] = meanGz[sensorIndex] / nAverageCycles;
        calibrations[sensorIndex].saveCalibration();
        Serial.print("Gyro bias:");
         for (int i = 0; i < 3; i++)
        {
            Serial.print(calibrations[sensorIndex].gyro_zerorate[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
    }
}

#endif
