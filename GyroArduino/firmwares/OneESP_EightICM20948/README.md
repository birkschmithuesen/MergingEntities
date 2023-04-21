This is the firmare for the board with one ESP32 and 8 ICM-20948 sensors.
The ICM-20948 was chosen because it is readily available and the MPU9250 is discontinued.
No support of other sensors is planned (as of 2023-04-21).

# development
The "-main" branch always contains working code. The "-dev" branch contains the
(potentially buggy) ongoing development state. Features are developed in dedicated branches.

# links
* [Adafruit ICM-20948 documentation](https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu)
* [Adafruit Arduino library for ICM20X ](https://github.com/adafruit/Adafruit_ICM20X)
