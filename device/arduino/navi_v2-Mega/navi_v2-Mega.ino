//////////////////////PINS//////////////////////
/*
  GPS: VIN: 3.3V  RX: 11  TX: 12
  Sonar Value: Top from l to r: 24,26,28,30
        botton from l to r: 22,23,25,27,29
*/
///////////////////////////////////////////////
#include <Wire.h>
#include <NewPing.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Gyro - Compass
float heading = 0.0;

//GPS
static const int RXPin = 11, TXPin = 12;
static const uint32_t GPSBaud = 57600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
double latitude = 40.521788;
double longitude =  -74.4608355;

//Sonar
#define SONAR_NUM     9 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 29 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
unsigned int sonar_value[SONAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double lastPingTime;
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

#define BUFSIZE 256
const int safesize = BUFSIZE / 2;
char write_buffer[BUFSIZE];

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(24, 24, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(26, 26, MAX_DISTANCE),
  NewPing(28, 28, MAX_DISTANCE),
  NewPing(30, 30, MAX_DISTANCE),
  NewPing(22, 22, MAX_DISTANCE),
  NewPing(23, 23, MAX_DISTANCE),
  NewPing(25, 25, MAX_DISTANCE),
  NewPing(27, 27, MAX_DISTANCE),
  NewPing(29, 29, MAX_DISTANCE)
};

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  lastPingTime = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  gyroSetup();
}

void loop()
{
  getGPS();
  //getsonar_value();
  getCompass();
  writeSerial();
}

void getGPS() {
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
}

void getsonar_value() {
  if (millis() >= lastPingTime + PING_INTERVAL) {         // Is it this sensor's time to ping?
    sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
    ++currentSensor;                            // Sensor being accessed.
    if (currentSensor == SONAR_NUM) currentSensor = 0; // Sensor ping cycle complete, do something with the results.
    sonar_value[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
    sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    lastPingTime = millis();
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
<<<<<<< HEAD
    sonar_value[currentSensor] = (sonar[currentSensor].ping_result/2) * 3.4029;
}
void getCompass(){
  heading = gyroLoop() *180/PI;
=======
    sonar_value[currentSensor] = sonar[currentSensor].ping_cm;
>>>>>>> 083151196f88d81195e45896f6a03b8d95e695c3
}


char ftos [safesize];
void writeSerial()
{
  memset(write_buffer, '\0', BUFSIZE);
  strcat(write_buffer, "[mega,");
  /*for (int x = 0; x < SONAR_NUM; x++)
  {
    dtostrf(sonar_value[x] / 50.0 , 20, 10, ftos);
    strcat(write_buffer, ftos);
    strcat(write_buffer, ",");
  }*/
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
