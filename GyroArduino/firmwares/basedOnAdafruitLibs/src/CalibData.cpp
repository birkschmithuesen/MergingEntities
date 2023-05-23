#include "CalibData.hpp"

CalibData calibrationSets[]{
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        0, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {-7.21, -28.69, -8.63},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1.022, -0.011, -0.001,
         0, 0.983, -0.004,
         0, 0, 0.996},

        /**! The magnetic field magnitude in uTesla */
        39.93},
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        1, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {16.43, -1.75, 33.61},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1.011, -0.009, -0.013,
         0, 0.983, 0.002,
         0, 0, 1.007},

        /**! The magnetic field magnitude in uTesla */
        39.59},
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        2, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {7.69, -4.59, -21.54},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1.016, -0.004, 0.006,
         0, 0.982, 0.003,
         0, 0, 1.003},

        /**! The magnetic field magnitude in uTesla */
        38.88},
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        3, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {0, 0, 0},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1, 0, 0,
         0, 1, 0,
         0, 0, 1},

        /**! The magnetic field magnitude in uTesla */
        50},
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        4, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {0, 0, 0},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1, 0, 0,
         0, 1, 0,
         0, 0, 1},

        /**! The magnetic field magnitude in uTesla */
        50},
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        5, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {0, 0, 0},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1, 0, 0,
         0, 1, 0,
         0, 0, 1},

        /**! The magnetic field magnitude in uTesla */
        50},
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        6, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {0, 0, 0},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1, 0, 0,
         0, 1, 0,
         0, 0, 1},

        /**! The magnetic field magnitude in uTesla */
        50},
    //////////////////// One block for each sensor:
    {
        0, // controllerIndex
        7, // sensorIndex

        /**! XYZ vector of offsets for zero-g, in m/s^2 */
        {0, 0, 0},

        /**! XYZ vector of offsets for zero-rate, in rad/s */
        {0, 0, 0},

        /**! XYZ vector of offsets for hard iron calibration (in uT) */
        {0, 0, 0},

        /**! The 3x3 matrix for soft-iron calibration (unitless) */
        {1, 0, 0,
         0, 1, 0,
         0, 0, 1},

        /**! The magnetic field magnitude in uTesla */
        50},
};