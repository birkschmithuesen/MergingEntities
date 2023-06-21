#pragma once
#include "I2CMultiplexer.hpp"
#include "Imu.hpp"
#include "CalibrationManager.hpp"

#include <WiFiUdp.h>
#include <OSCMessage.h>



class MultiplexedImus
{
public:
    MultiplexedImus(){};
    void setupAll(int controllerId,CalibrationManager* calibrationManager); //< used to find preset and build OSC addresses);
    void updateAll();
    void printSerialAll();
    void sendOscAll(WiFiUDP& Udp,IPAddress& receiverIp,int receiverPort);
    Imu imus[N_SENSORS];
    HardCodedCalibration calibrations[N_SENSORS] ; // these are loaded on setup
    ImuOscMessage statusMessage; // used to send error counters of all imus
    int highestErrorCode=0; // highest error code of all imus in last update
private:
 void buildOscName(int controllerId,int sensorIndex, char*dst);    
};