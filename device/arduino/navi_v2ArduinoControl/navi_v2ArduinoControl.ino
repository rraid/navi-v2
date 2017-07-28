//////////////////////PINS//////////////////////
/*
Magnometer: VIN: 3.3V  SDA: 20  SCL: 21
Compass: VIN: 3.3V  RX: 11  TX: 12
Sonars: 

*/
///////////////////////////////////////////////
#include <Wire.h>
#include <FRCmotor.h>
#include <NewPing.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

int gamemode = 1; // Enables the FRCmotor library

//Magnometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
double compass_theta= 0;

//Compass
static const int RXPin = 11, TXPin = 12;
static const uint32_t GPSBaud = 57600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
double latitude = 40.521788;
double longitude =  -74.4608355;

//Sonar
#define SONAR_NUM     9 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
unsigned int sonars[SONAR_NUM];
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(22, 22, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(23, 23, MAX_DISTANCE),
  NewPing(24, 24, MAX_DISTANCE),
  NewPing(25, 25, MAX_DISTANCE),
  NewPing(26, 26, MAX_DISTANCE),
  NewPing(27, 27, MAX_DISTANCE),
  NewPing(28, 28, MAX_DISTANCE),
  NewPing(29, 29, MAX_DISTANCE),
  NewPing(30, 30, MAX_DISTANCE)
};

//SerialRead/Write
#define BUFSIZE 256
#define SPEED_LIMIT 100.f
#define RAMP_CONST 1 // Higher is faster

const int safesize = BUFSIZE / 2;
char buf[BUFSIZE];
char write_buffer[BUFSIZE];
int available_bytes = 0;

// Target and previous velocity arrays
static float target_vel[] = {0.f , 0.f};

//Motor Control
FRCmotor leftMotor; //DECLARE LEFT MOTOR CONTROLLER
FRCmotor rightMotor; //DECLARE RIGHT MOTOR CONTROLLER



void setup() {
  leftMotor.SetPort(10); //DECLARE ARDUINO PORT FOR MOTOR CONTROLLER SIGNAL
  rightMotor.SetPort(9);
  leftMotor.Set(0); //SET INITIAL MOTOR VALUES TO ZERO
  rightMotor.Set(0); //(100 MAX FORWARD, -100 MAX BACK)
  
  Serial.begin(9600);
  
  ss.begin(GPSBaud);

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop()
{
  readSerial();
  moveMotor();
  getGPS();
  getSonars();
  getCompass();
  writeSerial();
}

void readSerial()
{
  if ((available_bytes = Serial.available()))
  {
    // Read + attach null byte to read string
    int obytes = strlen(buf);
    Serial.readBytes(&buf[obytes], available_bytes);
    buf[available_bytes + obytes] = '\0';
    
    if(strlen(buf) > safesize){
      memmove(buf,&buf[strlen(buf) - safesize],safesize);
      buf[safesize] = '\0';
    }
    char *s, *e;
    if ((e = strchr(buf, '\n')))
    {
      e[0] = '\0';
      if ((s = strrchr(buf, '[')))
      {
        sscanf(s, "[%d,%d]\n", &target_vel[0], &target_vel[1]);
        target_vel[0] = constrain(target_vel[0],-SPEED_LIMIT,SPEED_LIMIT);
        target_vel[1] = constrain(target_vel[1],-SPEED_LIMIT,SPEED_LIMIT);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }
}

void moveMotor()
{
  leftMotor.Set(int(target_vel[0] * 100));
  rightMotor.Set(int(target_vel[1] * 100));
}

void getGPS(){
  if(gps.location.isValid()){
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
}

void getSonars(){
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
void oneSensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    sonars[i] = cm[i];
  }
}
void getCompass(){
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  //Calculate heading
  compass_theta = atan2(event.magnetic.y, event.magnetic.x);
  //Magnetic field error in Piscataway
  //https://www.ngdc.noaa.gov/geomag-web/
  float declinationAngle = -0.22;
  compass_theta += declinationAngle;
  //Convert to degrees
  compass_theta = compass_theta *180/M_PI;
}

char ftos [safesize];
void writeSerial()
{
  memset(write_buffer,'\0',BUFSIZE);
  for(int x = 0; x<SONAR_NUM;x++)
  {
    sprintf(ftos,"%f",sonars[x]);
    strcat(write_buffer, ftos);
    strcat(write_buffer, ",");
  }
  sprintf(ftos,"%f",latitude);
  strcat(write_buffer, ftos);
  strcat(write_buffer, ",");
  sprintf(ftos,"%f",longitude);
  strcat(write_buffer, ftos);
  strcat(write_buffer, ",");
  sprintf(ftos,"%f",compass_theta);
  strcat(write_buffer, ftos);
  strcat(write_buffer, "\n");
  Serial.write(write_buffer);
}

