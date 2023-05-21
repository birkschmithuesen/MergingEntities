
// libraries for wireless communication
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

// hardware support
#include <Wire.h>
#include <EEPROM.h>

#include "PinsAndAddresses.hpp"
#include "UserInterface.hpp"
#include "NetworkConfig.hpp"
#include "MultiplexedImu.hpp"

WiFiUDP Udp;                            /**< handler for UDP communication */
IPAddress receiverIp(192, 168, 0, 104); /**< IP address of the (target) OSC server */
int receiverPortStart = 8000;           /**< default UDP server port on OSC receiver (i.e. central server), gets offset by controller ID */
int receiverPort;                       // set later according to controller ID

int controllerID; // set later according to DIP-switch

MultiplexedImus imuCollection;

// for limiting the rate of reports
uint32_t lastReportMillis = 0;
uint32_t reportInterval = 10; // ms
void setup()
{
    Serial.begin(115200);
    // pause until serial line is available
    while (!Serial)
    {
        delay(10);
    }
    Serial.flush(); // clear serial buffer
    delay(500);

    // determine controller ID from DIP-switch
    controllerID = UserInterface::getControllerID();
    Serial.print("This is controller no ");
    Serial.println(controllerID);
    if (controllerID > 7)
    {
        Serial.print("ID>7, but only 8 channels supported!");
        controllerID = 7;
    }

    /////////////////ATTENTION!///////////////////
    controllerID = 0; // for debug purposes only
                      /////////////////ATTENTION!///////////////////

    receiverPort = receiverPortStart + controllerID;

    ///////////set up Wifi
    Serial.println("setting up Wifi ");
    setupWifi(Udp);
    Serial.print("target OSC server is ");
    Serial.print(receiverIp);
    Serial.print(" port ");
    Serial.println(receiverPort);

    // set up I2C communication
    Serial.print("setting up I2C pins .");
    Wire.begin(SDA_PIN, SCL_PIN);
    I2CMultiplexer::testConnection();

    /////////////set up sensors
    Serial.println("setting up sensors");
    imuCollection.setupAll(controllerID);
}

void loop()
{
    uint32_t timestamp = micros();
    imuCollection.updateAll();

    Serial.print("Update took ");
    Serial.print(micros() - timestamp);
    Serial.println("mus");

    if ((millis() - lastReportMillis) > (reportInterval))
    {
        lastReportMillis = millis();
        // imuCollection.printSerialAll();
        timestamp = micros();
        imuCollection.sendOscAll(Udp, receiverIp, receiverPort);
        Serial.print("print/send took ");
        Serial.print(micros() - timestamp);
        Serial.println("mus");
    }
}