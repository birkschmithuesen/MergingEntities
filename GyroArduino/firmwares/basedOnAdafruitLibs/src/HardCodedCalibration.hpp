#pragma once
#include <Adafruit_Sensor_Calibration.h>
// Uses hard coded values from CalibData.cpp

class HardCodedCalibration: public Adafruit_Sensor_Calibration{
    public:
  bool begin(int controllerIndex,int sensorIndex);  

  bool saveCalibration(void){return false;}; // these don't do anything
  bool loadCalibration(void);/**< looks though list of saved presets to finda match of the IDs */
  bool printSavedCalibration(void){return false;};
private:
int controllerIndex=0;
int sensorIndex=0;
};