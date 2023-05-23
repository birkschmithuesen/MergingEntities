


// libraries for wireless communication
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

// hardware support
#include <Wire.h>
#include <EEPROM.h>
#include "Preferences.h"

#include "PinsAndAddresses.hpp"
#include "UserInterface.hpp"
#include "NetworkConfig.hpp"
#include "MultiplexedImu.hpp"
#include "NvmStoredCalibration.h"




//////network related stuff
WiFiUDP Udp;                            /**< handler for UDP communication */
TaskHandle_t oscSendTask;               /**< sends  osc updates parallel to computation*/

//#define WIFI_SSID "syntheticwire"     /**< SSID / name of the wifi network to use */
//#define WIFI_PASS "doesnotmatter"  /**< password for the wifi network to use */

#define WIFI_SSID "ArtNet4Hans"     /**< SSID / name of the wifi network to use */
#define WIFI_PASS "kaesimira"  /**< password for the wifi network to use */


IPAddress receiverIp(192, 168, 0, 2); /**< IP address of the (target) OSC server */
//IPAddress receiverIp(255, 255, 255, 255); /**< IP address of the (target) OSC server */
int receiverPortStart = 8000;           /**< default UDP server port on OSC receiver (i.e. central server), gets offset by controller ID */
int receiverPort;                       // set later according to controller ID
#define  UDP_localPort  8888  

int controllerID; // set later according to DIP-switch
CalibrationManager calibrationManager;
MultiplexedImus imuCollection;
Preferences nvm;
TaskHandle_t blinkTask;               /**< sends  osc updates parallel to computation*/

// Task1code: blinks an LED every 1000 ms
void oscSendFun(void *pvParameters)
{
    uint32_t timestamp=micros();
    while (true)
    {   
        //Serial.print("sending osc package. Rate is (Hz)=");
        //Serial.println(1000000.0f/(micros()-timestamp));
        timestamp=micros();
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
    UserInterface::setup();
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
    setupWifi(Udp,WIFI_SSID, WIFI_PASS, UDP_localPort);

    Serial.print("target OSC server is ");
    Serial.print(receiverIp);
    Serial.print(" port ");
    Serial.println(receiverPort);

    // set up I2C communication
    Serial.print("setting up I2C pins .");


    Wire.begin(SDA_PIN, SCL_PIN,(uint32_t)1000000);
    I2CMultiplexer::testConnection();

    // load calibrations
    calibrationManager.setup(&nvm,controllerID,&imuCollection);

    /////////////set up sensors
    Serial.println("setting up sensors");
    imuCollection.setupAll(controllerID, &calibrationManager);

    // start OSC send task
    if (false) // can be disabled for test purposes
    {
        xTaskCreatePinnedToCore(
            oscSendFun,   /* Task function. */
            "OscSender",  /* name of task. */
            20000,        /* Stack size of task */
            NULL,         /* parameter of the task */
            2,            /* priority of the task */
            &oscSendTask, /* Task handle to keep track of created task */
            0);           /* pin task to core 0 */
    }
if(true){
    // start blink Task
        xTaskCreatePinnedToCore(
            UserInterface::blinkFunction,   /* Task function. */
            "Blink",  /* name of task. */
            2000,        /* Stack size of task */
            NULL,         /* parameter of the task */
            1,            /* priority of the task */
            &blinkTask, /* Task handle to keep track of created task */
            1);           /* pin task to core 0 */
}
// test calibration
//calibrationManager.calibrateSensor(0,&imuCollection.imus[0]);
}

uint32_t globalTimestamp = 0;
    
void loop()
{
    uint32_t timestamp = micros();
    
    ///update frequency
    Serial.print(1000000/(timestamp - globalTimestamp));
    Serial.println("Hz");
    globalTimestamp=timestamp;

    vTaskDelay(1); // to avoid starving other tasks;
    
    timestamp = micros(); 
    imuCollection.updateAll();
    
        Serial.print("Update took ");
        Serial.print(micros() - timestamp);
        Serial.println("mus");
    
    
    timestamp = micros(); 
    imuCollection.sendOscAll(Udp, receiverIp, receiverPort); // this is now done by a separate task
    
        Serial.print("Update took ");
        Serial.print(micros() - timestamp);
        Serial.println("mus");
    
    
    //support for interactive calibration
    #ifndef USE_HARD_CODED_CALIBRATION
    if(UserInterface::getCalibrationButtonState())calibrationManager.calibrateSequence();
    #endif



    // Serial printing is (painfully) slow...
    if (false)
    {
        timestamp = micros();        
        imuCollection.printSerialAll();
    
        Serial.print("print/send took ");
        Serial.print(micros() - timestamp);
        Serial.println("mus");
    }
    
}