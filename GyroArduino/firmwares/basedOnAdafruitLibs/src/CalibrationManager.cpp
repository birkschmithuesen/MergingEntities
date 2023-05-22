#include "CalibrationManager.hpp"
#include "MultiplexedImu.hpp"
#include "HardwareSerial.h"
void CalibrationManager::setup(Preferences *nvm, int controllerId, MultiplexedImus *imus)
{
    this->imus = imus;
    for (int i = 0; i < N_SENSORS; i++)
    {
#ifdef USE_HARD_CODED_CALIBRATION
        // get calibration info
        HardCodedCalibration *curCalib = &(calibrations[i]);
        curCalib->begin(controllerId, i);
#else
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
    Serial.println("Starting calibration sequence.");
    digitalWrite(PIN_BLUELED, HIGH); // Blue LED flash: ready to calibrate next sensor
    delay(500);
    digitalWrite(PIN_BLUELED, LOW);
    for (int curSensorId = 0; curSensorId < N_SENSORS; curSensorId++)
    {

        calibrateSensorInteractive(curSensorId, &imus->imus[curSensorId]);
    }
}

void CalibrationManager::calibrateSensorInteractive(int sensorId, Imu *imu)
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
        Serial.println("Calibration data recieved and saved.");
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
void CalibrationManager::calibrateSensor(int sensorId, Imu *imu)
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
#endif
