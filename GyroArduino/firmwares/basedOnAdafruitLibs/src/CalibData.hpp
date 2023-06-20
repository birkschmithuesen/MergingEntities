#pragma once
#define CALIB_N_PRESETS 8*6

struct CalibData
{
    int controllerIndex = 0;
    int sensorIndex = 0;

    /**! XYZ vector of offsets for zero-g, in m/s^2 */
    float accel_zerog[3] = {0, 0, 0};

    /**! XYZ vector of offsets for zero-rate, in rad/s */
    float gyro_zerorate[3] = {0, 0, 0};

    /**! XYZ vector of offsets for hard iron calibration (in uT) */
    float mag_hardiron[3] = {0, 0, 0};

    /**! The 3x3 matrix for soft-iron calibration (unitless) */
    float mag_softiron[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    /**! The magnetic field magnitude in uTesla */
    float mag_field = 50;
};

extern CalibData calibrationSets[CALIB_N_PRESETS];