#include "MultiplexedImu.hpp"
#include "I2CMultiplexer.hpp"
#include "SensorNames.hpp"
void MultiplexedImus::setupAll(int controllerId, CalibrationManager *calibrationManager)
{
    int nImusWorking = 0;

    char oscNameBuffer[200];
    for (int i = 0; i < N_SENSORS; i++)
    {
        // set multiplexer channel
        Serial.print("Setting multiplexer to channel ");
        Serial.println(i);
        I2CMultiplexer::selectI2cMultiplexerChannel(i);

        // generate OSC name
        buildOscName(controllerId, i, oscNameBuffer);

        // let the imu do its setup
        imus[i].setup(oscNameBuffer, i, &(calibrationManager->calibrations[i]));

        if (imus[i].lastErrorState == Imu::sensorConfigSuccess)
            nImusWorking++;
    }
    Serial.print(nImusWorking);
    Serial.print(" out of ");
    Serial.print(N_SENSORS);
    Serial.println(" set up successfully!");

    char statusAdress[50];
    strcpy(statusAdress, "/body/");
    strcat(statusAdress, controllerNames[controllerId]);
    strcat(statusAdress, "/status/");
    statusMessage.init(statusAdress, 8);
}

void MultiplexedImus::updateAll()
{
    highestErrorCode=0;
    for (int i = 0; i < N_SENSORS; i++)
    {
        // set multiplexer channel
        I2CMultiplexer::selectI2cMultiplexerChannel(i);
        // get data and update sensor fusion
        imus[i].update();
        if(highestErrorCode<imus[i].lastErrorCode)highestErrorCode=imus[i].lastErrorCode;
        statusMessage.setFloat(i, imus[i].errorCounter);
    }
}
void MultiplexedImus::sendOscAll(WiFiUDP &Udp, IPAddress &receiverIp, int receiverPort)
{
    for (int i = 0; i < N_SENSORS; i++)
    {
        imus[i].sendOsc(Udp, receiverIp, receiverPort);
    }
    vTaskDelay(2); // to avoid starving other tasks;
}

void MultiplexedImus::printSerialAll()
{
    for (int i = 0; i < N_SENSORS; i++)
    {
        imus[i].printSerial();
    }
}

void MultiplexedImus::buildOscName(int controllerId, int sensorIndex, char *dst)
{
    if (true)
    {
        strcpy(dst, "/body/");
        strcat(dst, controllerNames[controllerId]);
        strcat(dst, "/gyro/");
        strcat(dst, sensorNames[sensorIndex]);
        strcat(dst, "/");
    }
    else
    {
        // a short version to reduce data load
        strcpy(dst, "/b/");
        strcat(dst, controllerNames[controllerId]);
        strcat(dst, "/g/");
        strcat(dst, controllerNames[sensorIndex]);
    }
}
