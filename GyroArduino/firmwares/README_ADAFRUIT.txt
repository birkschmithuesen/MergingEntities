----- 27/06/2022 -----
----- Instructions to properly run the code -----

PARAMETERS OF THE CODE :

- At the beginning of the code stands the comment : //-------GENERAL SETTINGS-----
It follows a number of parameters to change according to the situation :

- You have to set the north of your stage (See EXECUTION OF THE CODE part)

- When using a new wifi, you can follow this example and add #define NEW_WIFI_NAME at the beginning of the code to set it up

#ifdef NEW_WIFI_NAME
#define WIFI_SSID "The name of your wifi"
#define WIFI_PASS "The password of your wifi"
IPAddress outIp(192,168,0,2); //IP of the computer
int localPort = 8888; //Port of ESP
#endif

- When uploading the code in one of the three boxes, you have to define BODY_1, BODY_2 or BODY_3.

EXECUTION OF THE CALIBRATION CODE :

- Two LEDS on the side of the box are here to tell what is happening in the code.
- One button on the side is here to interact with the ESP

- When you power on the ESP, you wil get through one initalization sequence :

1. Choice of calibration sequence sequence :
.The red LED is on for 5 seconds, indicating the beginning of the first sequence

.If you press the button for the 5 seconds, the gyroscope and accelerometer calibration will begin : see A.
If you don't press the button, the magnetometer calibration will begin : see B.

A. Calibration of gyroscope :
.The red LED will be on until one gyroscope is calibrated.
You have to let all the MPU still until the red LED is off.

.Then, the yellow LED will be on until one accelerometer is calibrated.
You have to let the IMU as flat and still as possible until the yellow LEED is off.

.Finally, the same sequence A. will occur again for the 6 IMU


B. Calibration of magnetometers :
. PREREQUISITE : you have to close the Serial Monitor of VisualStudioCode and power the ESP through the USB port to allow SerialCommunication with MotionCal (=the calibration software)

.Open "MotionCal.exe"
.Power the ESP and enter the magnetometer calibration sequence
.In MotionCal, select the port COMx corresponding to the one of the USB communication.

.Round the first IMU in every direction until you drawed a nice sphere of red dots
.Click on "Send Cal". The two leds of the box should blink, indicating the first magnetometer calibration is received.

.Take the second IMU, click on "Clear" and re do the two previous steps for each IMU
.When you are done with the last one, just close MotionCal and power off the ESP

.You can now upload the regular code to get it working

EXECUTION OF THE CODE :

- Two LEDS on the side of the box are here to tell what is happening in the code.
- When you power on the ESP, you wil have the choice to set the north or not :

1. Choosing to set the north :
.The two LEDS are on for 6 seconds
.If you press the button for the 6 seconds, the north will be set in the direction pointed by the IMU 1

LIBRARY MODIFICATIONS :

.Adafruit  AHRS: Some modifications occured in the library, in comparizon to the original version.
Thus, you have to use this version only to get the code working fine.

.Others : No modifications from the original ones

ISSUES :

Event : the IMU fails to be initialized and the code stops
Details : when the IMU is not initialized, the pointers pointing on the accelerometer, gyroscope and magnetometer are not initialize.
Then, when we try to access the data, we get an error : GuruMeditation (LoadProhibited).
This error means that we are trying to read in unauthorized part of the ESP32 memory.
Thus, the ESP doesn't allow the reading and reboot endlessly.
The problem is in the "sensors.init()" function, failing to initalize the sensors.
Especially in this function, the bug occurs when trying to do the Wire.begin() function (which is not a function that would fail in general.
Maybe it has something to do with the multiplexer as well, since this error doesn't occur without.

Ideas : .Investigate the multiplexer functioning to see if it is a possible cause of the problem
.Investigate why the Wire.begin() function could fail (too much call ? call frequency ?)















