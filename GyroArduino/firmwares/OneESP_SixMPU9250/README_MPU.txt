----- 13/06/2022 -----
----- Instructions to properly run the code -----

PARAMETERS OF THE CODE :

- At the beginning of the code stands the comment : //-------GENERAL SETTINGS-----
It follows a number of parameters to change according to the situation :

- When using the code in a new location, you have to change the magnetic declination. It's the offset the magnetic earth in the location.
To find the one of your location, just go to this website : https://www.magnetic-declination.com/
Then, at the line #define MAG_DECLINATION xxx, change xxx by your nwe magnetic declination.

You also have to set the north of your stage (See EXECUTION OF THE CODE PART)

- When using a new wifi, you can follow this example and add #define NEW_WIFI_NAME at the beginning of the code to set it up

#ifdef NEW_WIFI_NAME
#define WIFI_SSID "The name of your wifi"
#define WIFI_PASS "The password of your wifi"
IPAddress outIp(192,168,0,2); //IP of the computer
int localPort = 8888; //Port of ESP
#endif

- When uploading the code in one of the three boxes, you have to define BODY_1, BODY_2 or BODY_3.

EXECUTION OF THE CODE :

- Two LEDS on the side of the box are here to tell what is happening in the code.
- One button on the side is here to interact with the ESP

- When you power on the ESP, you wil get through two initalization sequences :

1. Calibration sequence :
.The two LEDS are on for 6 seconds, indicating the beginning of the first sequence.

.If you press the button for the 6 seconds, the first calibration sequence will begin : see A. and B.
If you don't press the button, the last calibration data will be loaded.

A. Calibration of acceleration :
The red LED will be on until all the accelerometer and gyroscop are calibrated.
You have to let all the MPU STILL and FLAT until the red LED is off.

B. Calibration of magnetometers :
The yellow LED will turn on, meaning you have to round the first MPU in all direction.
Then, the yellow LED will turn off, meaning you have to round the second MPU in all direction.
Then, the yellow LED will turn on, meaning you have to round the third MPU in all direction.
Etc for the 6 MPUs

.The two LEDS are blinking, indicating the end of the first sequence.

2.North sequence :
.The two LEDS are on for 10 seconds, indicating the beginning of the second sequence.

.If you press the button for the 10 seconds and point your stage north with the first MPU, the north will be set in this direction.
If you don't press the button, the last stage north will be loaded.

.The two LEDS are blinking, indicating the end of the first sequence.

LIBRARY MODIFICATIONS :

.MPU9250 : Some modifications occured in the library, in comparizon to the original version.
Thus, you have to use this version only to get the code working fine.

.OSC : No modifications from the original ones


