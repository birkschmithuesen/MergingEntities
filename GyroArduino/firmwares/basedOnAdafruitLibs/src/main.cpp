
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
TaskHandle_t oscSendTask;               /**< sends  osc updates parallel to computation*/
IPAddress receiverIp(192, 168, 0, 104); /**< IP address of the (target) OSC server */
int receiverPortStart = 8000;           /**< default UDP server port on OSC receiver (i.e. central server), gets offset by controller ID */
int receiverPort;                       // set later according to controller ID

int controllerID; // set later according to DIP-switch

MultiplexedImus imuCollection;

// for limiting the rate of reports
uint32_t lastReportMillis = 0;
uint32_t reportInterval = 10; // ms

// Task1code: blinks an LED every 1000 ms
void oscSendFun(void *pvParameters)
{
    while (true)
    {
        imuCollection.sendOscAll(Udp, receiverIp, receiverPort);
        vTaskDelay(1); // to avoid starving other tasks;
    }
}

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

    // start OSC send task

    if (true)
    {
        xTaskCreatePinnedToCore(
            oscSendFun,   /* Task function. */
            "OscSender",  /* name of task. */
            20000,        /* Stack size of task */
            NULL,         /* parameter of the task */
            1,            /* priority of the task */
            &oscSendTask, /* Task handle to keep track of created task */
            0);           /* pin task to core 0 */
    }
}

void loop()
{
    vTaskDelay(1); // to avoid starving other tasks;
    uint32_t timestamp = micros();
    imuCollection.updateAll();

    Serial.print("Update took ");
    Serial.print(micros() - timestamp);
    Serial.println("mus");

    // imuCollection.sendOscAll(Udp, receiverIp, receiverPort); // this is now done by a separate task

    // this is (painfully) slow...
    if (false)
    {
        timestamp = micros();

        imuCollection.printSerialAll();

        Serial.print("print/send took ");
        Serial.print(micros() - timestamp);
        Serial.println("mus");
    }
}