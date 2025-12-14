/*
  u-blox GPS NAV-PVT (UBX) parser implementation
  Streams UBX over `Serial1`, accumulates a NAV-PVT struct, and validates with Fletcher checksum.
  Updates globals for time-of-week, NED velocity, LLA, and MSL altitude.
  Note: `gps_setup()` sends UBX config at 9600 baud and waits for frames; ensure wiring and power.
*/
#include "gps.h"

NAV_PVT pvt;

unsigned long iTOW;
double velN, velE, gps_velD, latitude, longitude, gps_altitude;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  while ( gps.available() ) {
    byte c = gps.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {      
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

void gps_setup() 
{
  gps.begin(9600);
  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    gps.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  while(!processGPS);
}

void gps_read()
{
  if(processGPS())
  {
      iTOW = pvt.iTOW;
      velN = pvt.velN/1000.0f;
      velE = pvt.velE/1000.0f;
      gps_velD = pvt.velD/1000.0f;
      latitude = pvt.lat/10000000.0f;
      longitude = pvt.lon/10000000.0f;
      gps_altitude = pvt.hMSL/1000.0f;
  }
}
