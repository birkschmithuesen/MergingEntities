#include "ImuOscMessage.h"
#include <cstring>
#include <HardwareSerial.h>
void ImuOscMessage::init(const char *address, int nArgs)
{
    // set buffer to zero first
    std::memset(buffer, 0, ImuOscMessageBufferLength);
    // copy address
    strcpy(buffer, address);
    int curOffset= strlen(address)+1; //null termination
    if((curOffset/4)*4<curOffset)curOffset=(curOffset/4+1)*4; //padding

    // add type string (",fffffffff")
    buffer[curOffset] = ',';
    curOffset++;

    std::memset(buffer + curOffset, 'f', nArgs);
    curOffset += nArgs;
    curOffset += 1; //null termination

    if((curOffset/4)*4<curOffset)curOffset=(curOffset/4+1)*4; //padding

    // data starts after typestring
    dataStartOffset = curOffset;
    messageLength =dataStartOffset+ nArgs * 4;
    // pad message length to next multiple of 4
    if ((messageLength/4) * 4 < messageLength)
        messageLength = (messageLength/4 + 1) * 4;

    Serial.println(messageLength);
}
void ImuOscMessage::writeBigEndienFloat(char* bufStart,float value){
    char* inBufStart=((char*) &value)+3;
    for(int i=0;i<4;i++){
        *bufStart=*inBufStart;
        bufStart++;
        inBufStart--;
    }
}

void ImuOscMessage::setData(float qw, float qx, float qy, float qz, float roll, float pitch, float yaw, float gx, float gy, float gz)
{
    char* curDataStart=&buffer[dataStartOffset];

    writeBigEndienFloat(curDataStart,qw);
    curDataStart+=4;

    writeBigEndienFloat(curDataStart,qx);
    curDataStart+=4;

    writeBigEndienFloat(curDataStart,qy);
    curDataStart+=4;

    writeBigEndienFloat(curDataStart,qz);
    curDataStart+=4;



    writeBigEndienFloat(curDataStart,roll);
    curDataStart+=4;

    writeBigEndienFloat(curDataStart,pitch);
    curDataStart+=4;

    writeBigEndienFloat(curDataStart,yaw);
    curDataStart+=4;


    writeBigEndienFloat(curDataStart,gx);
    curDataStart+=4;

    writeBigEndienFloat(curDataStart,gy);
    curDataStart+=4;

    writeBigEndienFloat(curDataStart,gz);
    curDataStart+=4;
}