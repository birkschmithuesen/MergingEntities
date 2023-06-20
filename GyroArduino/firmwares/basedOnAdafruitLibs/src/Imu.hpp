#pragma once
#include "PinsAndAddresses.hpp"

#define ICM_ADDRESS 0x69 /**< I2C address of the ICM20948 sensor */

// libraries for local sensor communication
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>

#include <Wire.h>

#include <WiFiUdp.h>
#include <OSCMessage.h>

#include "CalibData.hpp"
#include "HardCodedCalibration.hpp"
#include "ImuOscMessage.h"
#include "Adafruit_AHRS_NXPFusion_flex_deltaT.hpp"
#include <mutex> 

class Imu
{
public:
    Imu();
    void setup(const char *oscName, int id,Adafruit_Sensor_Calibration* calibration);
    void update(); ///< read sensor, process data, prepare Osc-Message
    void readSensor(); ///< read sensor only, don't do any processing
    void prepareOscMessage(); ///< put current values into osc message
    void processCurrentData(); ///< process last data set read from Sensor (altering values in place)

    void checkReconnect(); ///< if the controller has forgotten one of its settings, reinitialize it.
    void printSerial();
    void sendMotionCal();

    void sendOsc(WiFiUDP& Udp,IPAddress& receiverIp,int receiverPort);
    enum ImuErrorStates
    {
        sensorConfigFailed,
        sensorConfigSuccess
    };
    ImuErrorStates lastErrorState;
    int lastErrorCode = 0;
    uint32_t updatesSinceLastReconnectCheck=0;
    sensors_event_t mag_event, gyro_event, temp_event, accel_event;
private:
    void configureSensor();

    char oscName[200];
    int sensorId=0;
    std::mutex oscMessageMutex; /**< controls access to the content of @oscMessage */
    ImuOscMessage oscMessage; /**< a buffer for a pre formed oscMessage, ready to be sent by the WiFi-send-task. lock @oscMessageMutex during access*/

    Adafruit_ICM20948 sensor; /**< software handler/abstraction for ICM20948 at given channel */
    // pick your filter! slower == better quality output
    //Adafruit_NXPSensorFusionFlexDeltaT filter; // slowest
    Adafruit_Madgwick filter;  // faster than NXP
    //Adafruit_Mahony filter; // fastest/smallest

    Adafruit_Sensor_Calibration* calibration;

    float gx, gy, gz; // for gyro, as these need to be converted for filtering
    uint32_t lastUpdateMicros=0;
    int reconnectCount=0; // for limiting amount of reconnection attempts:
};