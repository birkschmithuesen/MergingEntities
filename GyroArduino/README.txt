Run the code properly : 

. You have to set the magnetic declination according to your position. To do so, find the lines mpu1.setMagneticDeclination
and mpu2.setMagneticDeclination and put the number you find on this website : https://www.magnetic-declination.com/

. You have to setup one parameter in the library :
- in the MPU9250.h library, you have to set line 85 : static constexpr uint8_t MAG_MODE {0x02}; : 0x02 instead of 0x08, the default value

