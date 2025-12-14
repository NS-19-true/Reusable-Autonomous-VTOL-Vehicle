/*
  BMP280 altitude module
  Provides a simple interface to the Adafruit BMP280 driver.
  Exposes `bmp_altitude` (meters) as the latest altitude estimate, updated by `bmp_read()`.
  - Call `bmp_setup()` once to initialize sensor and sampling.
  - Call `bmp_read()` to refresh `bmp_altitude` using sea-level pressure calibration.
*/
#include "Adafruit_BMP280.h"


extern float bmp_altitude;

void bmp_setup();
void bmp_read();
