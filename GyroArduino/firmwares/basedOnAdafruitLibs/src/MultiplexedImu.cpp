#include "MultiplexedImu.hpp"
#include "I2CMultiplexer.hpp"
#include "SensorNames.hpp"
void MultiplexedImus::setupAll(int controllerId)
{
    int nImusWorking=0;

    char oscNameBuffer[200];
    for(int i=0;i<N_SENSORS;i++){
        // set multiplexer channel
        Serial.print("Setting multiplexer to channel ");
        Serial.println(i);
        I2CMultiplexer::selectI2cMultiplexerChannel(i);

        // generate OSC name
        buildOscName(controllerId,i,oscNameBuffer);

        // get calibration info
        HardCodedCalibration* curCalib=&(calibrations[i]);
        curCalib->begin(controllerId,i);
        curCalib->loadCalibration();
        
        //let the imu do its setup 
        imus[i].setup(oscNameBuffer, curCalib);
        if(imus[i].lastErrorState==Imu::sensorConfigSuccess)nImusWorking++;
    }
    Serial.print (nImusWorking);
    Serial.print (" out of ");
    Serial.print (N_SENSORS);
    Serial.println (" set uo successfully!");
}

void MultiplexedImus::updateAll()
{
    for(int i=0;i<N_SENSORS;i++){
        // set multiplexer channel
        I2CMultiplexer::selectI2cMultiplexerChannel(i);
        imus[i].update();
    }
}
void MultiplexedImus::sendOscAll(WiFiUDP& Udp,IPAddress& receiverIp,int receiverPort){
    for(int i=0;i<N_SENSORS;i++){
         imus[i].sendOsc( Udp, receiverIp, receiverPort);
    } 
}
    

void MultiplexedImus::printSerialAll(){
    for(int i=0;i<N_SENSORS;i++){
         imus[i].printSerial();
    } 
}

void MultiplexedImus::buildOscName(int controllerId, int sensorIndex, char *dst)
{
    strcpy(dst, "/body/");
    strcat(dst, controllerNames[controllerId]);
    strcat(dst, "/gyro/");
    strcat(dst, sensorNames[sensorIndex]);
    strcat(dst, "/");
}
