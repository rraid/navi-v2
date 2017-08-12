//////////////////////PINS//////////////////////
/*
  GPS: VIN: 3.3V  RX: 11  TX: 12
  Sonar Value: Top from l to r: 24,26,28,30
        botton from l to r: 22,23,25,27,29
*/
///////////////////////////////////////////////
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "compass.h"

//Compass
int heading = 0;

//GPS
static const int RXPin = 11, TXPin = 12;
static const uint32_t GPSBaud = 57600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
double latitude = 40.521788;
double longitude =  -74.4608355;

#define BUFSIZE 256
const int safesize = BUFSIZE / 2;
char write_buffer[BUFSIZE];

int delayGPS;

void setup() {
  Serial.begin(57600);
  ss.begin(GPSBaud);
  Wire.begin();
  compass_x_offset = -112.66;
  compass_y_offset = 992.60;
  compass_z_offset = 546.70;
  compass_x_gainError = 1.01;
  compass_y_gainError = 1.09;
  compass_z_gainError = 0.99;

  compass_init(2);
}

void loop()
{
  getGPS();
  //getsonar_value();
  //getCompass();
  writeSerial();
}

void getGPS() {
  while (ss.available()){
    gps.encode(ss.read());
  }
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
}

void getCompass(){
  compass_heading();
  heading = (int)(bearing + 260) % 360;
}

char ftos [safesize];
void writeSerial()
{
  memset(write_buffer, '\0', BUFSIZE);
  strcat(write_buffer, "[");
  dtostrf(latitude, 20, 10, ftos);
  strcat(write_buffer, ftos);
  strcat(write_buffer, ",");
  dtostrf(longitude, 20, 10, ftos);
  strcat(write_buffer, ftos);
  strcat(write_buffer, ",");
  dtostrf(heading, 20, 10, ftos);
  strcat(write_buffer, ftos);
  strcat(write_buffer, "]\n");
  Serial.write(write_buffer);
}
