#pragma once
// a class to build osc messages with imu data more quickly than with the OSC library
#define ImuOscMessageBufferLength 1024
class ImuOscMessage
{
public:
    void init(const char *address, int nArgs);
    void setData(float qw, float qx, float qy, float qz, float roll, float pitch, float yaw, float gx,float gy, float gz);
    char buffer[ImuOscMessageBufferLength];
    unsigned int dataStartOffset = 0;
    unsigned int messageLength = 0;
    void writeBigEndienFloat(char* bufStart,float value);
};