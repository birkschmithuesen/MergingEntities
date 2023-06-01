#pragma once
// a class to build osc messages with imu data more quickly than with the OSC library
#define ImuOscMessageBufferLength 1024
#
extern unsigned long debugPackageIndex; // for keeping track of dropped packages

class ImuOscMessage
{
public:
    void init(const char *address, int nArgs);
    void setFloat(int pos, float value);
    void setData(float qw, float qx, float qy, float qz, float roll, float pitch, float yaw, float gx,float gy, float gz);
    char buffer[ImuOscMessageBufferLength];
    unsigned int dataStartOffset = 0;
    unsigned int messageLength = 0;
    bool hasBeenSent=false;
    void writeBigEndianFloat(char* bufStart,float value);
    int nArgsAllocated=0;
};