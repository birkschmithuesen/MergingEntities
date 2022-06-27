#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

bool init_sensors(int i) {
  if (!fxos.begin() || !fxas.begin()) {
    return false;
  }
  accelerometer[i] = fxos.getAccelerometerSensor();
  gyroscope[i] = &fxas;
  magnetometer[i] = fxos.getMagnetometerSensor();
  return true;
}

void setup_sensors(void) {}
