#include <Adafruit_Sensor_Calibration.h>
// Uses data from Esp32 NVM
#include "Preferences.h"
class NvmStoredCalibration : public Adafruit_Sensor_Calibration
{
public:
    bool begin(Preferences *nvm, int controllerIndex, int sensorIndex);

    bool saveCalibration(void); /**< accesses NVM to set Values */
    bool loadCalibration(void); /**< accesses NVM to get Values */
    bool printSavedCalibration(void) { return false; };

private:
    char presetName[10];
    Preferences *nvm;
};