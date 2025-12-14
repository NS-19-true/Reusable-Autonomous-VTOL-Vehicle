/*
  BMP280 altitude module implementation
  Initializes BMP280 and sets sampling configuration; `bmp_read()` updates `bmp_altitude`.
  Notes:
  - Sea-level pressure (SLP) is set to 1019.66 hPa; adjust for local SLP for accurate altitude.
  - Oversampling: Temp x2, Pressure x16; Filter OFF; Standby 1 ms.
*/
#include "bmp.h"

Adafruit_BMP280 bmp;

float bmp_altitude;

void bmp_setup()
{
    if (!bmp.begin()) 
    {
    //Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        while (1);
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

}

void bmp_read()
{
    bmp_altitude = bmp.readAltitude(1019.66);
}
