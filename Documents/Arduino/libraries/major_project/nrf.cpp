/*
  NRF24L01 telemetry implementation
  Uses `RF24 radio(CE=9, CSN=10)` to transmit/receive `Data_Package` across two fixed pipes.
  AutoAck disabled; 2Mbps data rate; PA set to MAX. Listening mode is resumed after send.
*/
#include "nrf.h"

RF24 radio(9, 10); // CE, CSN
//unsigned long Time;


Data_Package msg;


void sendData(float m1=99.99, float m2=99.99, float m3=99.99, float m4=99.99, float m5=99.99, float m6=99.99, float m7=99.99, float m8=99.99)
{
  msg.m1 = m1;
  msg.m2 = m2;
  msg.m3 = m3;
  msg.m4 = m4;
  msg.m5 = m5;
  msg.m6 = m6;
  msg.m7 = m7;
  msg.m8 = m8;
  
  radio.stopListening();
  radio.write(&msg, sizeof(Data_Package));
  radio.startListening();
}

bool receiveData()
{
  if (radio.available())
  {
    radio.read(&msg, sizeof(Data_Package));
    return 1;
  }
  return 0;
}

void nrf_setup() {
  radio.begin();
  radio.openWritingPipe(addresses[1]);//efghi
  radio.openReadingPipe(1, addresses[0]);//abcde

  radio.setDataRate(RF24_2MBPS);
 
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.startListening();
}
