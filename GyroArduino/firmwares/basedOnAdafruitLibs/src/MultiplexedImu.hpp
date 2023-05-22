#pragma once
#include "I2CMultiplexer.hpp"
#include "Imu.hpp"

#include <WiFiUdp.h>
#include <OSCMessage.h>

#define N_SENSORS 8

class MultiplexedImus
{
public:
    MultiplexedImus(){};
    void setupAll(int controllerId); //< used to find preset and build OSC addresses);
    void updateAll();
    void printSerialAll();
    void sendOscAll(WiFiUDP& Udp,IPAddress& receiverIp,int receiverPort);
    Imu imus[N_SENSORS];
    HardCodedCalibration calibrations[N_SENSORS] ; // these are loaded on setup
private:
 void buildOscName(int controllerId,int sensorIndex, char*dst);    
};