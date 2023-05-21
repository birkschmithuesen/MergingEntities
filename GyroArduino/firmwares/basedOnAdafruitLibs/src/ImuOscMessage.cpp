#include "ImuOscMessage.h"
#include <cstring>
#include <HardwareSerial.h>
void ImuOscMessage::init(const char *address, int nArgs)
{
    // set buffer to zero first
    std::memset(buffer, 0, ImuOscMessageBufferLength);
    // copy address
    strcpy(buffer, address);
    int curOffset= strlen(address);
    if((curOffset/4)*4<curOffset)curOffset=(curOffset/4+1)*4; //padding

    // add type string (",fffffffff")
    buffer[curOffset] = ',';
    curOffset++;
    std::memset(buffer + curOffset, 'f', nArgs);
    curOffset += nArgs;

    if((curOffset/4)*4<curOffset)curOffset=(curOffset/4+1)*4; //padding

    // data starts after typestring
    dataStartOffset = curOffset;
    messageLength =dataStartOffset+ nArgs * 4;
    // pad message length to next multiple of 4
    if ((messageLength/4) * 4 < messageLength)
        messageLength = (messageLength/4 + 1) * 4;

    Serial.println(messageLength);
}

void ImuOscMessage::setData(float qw, float qx, float qy, float qz, float roll, float pitch, float yaw, float gx, float gy, float gz)
{
    float *bufAsFloat = (float *)(buffer + dataStartOffset);
    *bufAsFloat = qw;
    bufAsFloat++;
    *bufAsFloat = qx;
    bufAsFloat++;
    *bufAsFloat = qy;
    bufAsFloat++;
    *bufAsFloat = qz;
    bufAsFloat++;

    *bufAsFloat = roll;
    bufAsFloat++;
    *bufAsFloat = pitch;
    bufAsFloat++;
    *bufAsFloat = yaw;
    bufAsFloat++;


    *bufAsFloat = gx;
    bufAsFloat++;
    *bufAsFloat = gy;
    bufAsFloat++;
    *bufAsFloat = gz;
    bufAsFloat++;
}