//////////////////////PINS//////////////////////
/*
Magnometer: VIN: 3.3V  SDA: 20  SCL: 21
Compass: VIN: 3.3V  RX: 11  TX: 12
sonar_value: Top from l to r: 24,26,28,30
        botton from l to r: 22,23,25,27,29
Motor - R: 9  L: 10

*/
///////////////////////////////////////////////
#include <Wire.h>
#include <FRCmotor.h>
#include <NewPing.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>


int gamemode = 1; // Enables the FRCmotor library

//Magnometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float heading= 0;

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
#define PING_INTERVAL 29 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
unsigned int sonar_value[SONAR_NUM] = {0,0,0,0,0,0,0,0,0};
double lastPingTime;
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

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

//SerialRead/Write
#define BUFSIZE 256
#define SPEED_LIMIT 100
#define RAMP_CONST 1 // Higher is faster

const int safesize = BUFSIZE / 2;
char buf[BUFSIZE];
char write_buffer[BUFSIZE];
int available_bytes = 0;

// Target and previous velocity arrays
static int target_vel[] = {0 , 0};

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

  lastPingTime = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
}

int prevMillis;
void loop()
{
  prevMillis = millis();
  Serial.print("Read: ");
  readSerial();
  Serial.print(millis()-prevMillis);
  prevMillis = millis();
  Serial.print(" Move: ");
  moveMotor();
  Serial.print(millis() - prevMillis);
  prevMillis = millis();
  Serial.print(" GPS: ");
  getGPS();
  Serial.print(millis() - prevMillis);
  //prevMillis = millis();
  //Serial.print(" Sonar: ");
  getsonar_value();
  Serial.println();
  //getCompass();
  //writeSerial();
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
  leftMotor.Set(-1*target_vel[0]);
  rightMotor.Set(target_vel[1]);
  //Serial.print(target_vel[0]);
  //Serial.println(target_vel[1]);
}

void getGPS(){
  if(gps.location.isValid()){
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
}

void getsonar_value(){
  if (millis() >= lastPingTime + PING_INTERVAL) {         // Is it this sensor's time to ping?
    Serial.print(" Sonar Ping Time: ");
    prevMillis = millis();
    sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        Serial.print(millis()- prevMillis);

    ++currentSensor;                            // Sensor being accessed.
    if (currentSensor == SONAR_NUM) currentSensor = 0; // Sensor ping cycle complete, do something with the results.
    sonar_value[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
    sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    lastPingTime = millis();
    Serial.print(millis()- prevMillis);
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    sonar_value[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}



void getCompass(){
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  
  //Calculate heading
  heading = atan2(event.magnetic.y, event.magnetic.x);
  
  //Magnetic field error in Piscataway
  //https://www.ngdc.noaa.gov/geomag-web/
  float declinationAngle = -0.22;
  heading += declinationAngle;

  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  
  //Convert to degrees
  heading = heading *180/M_PI;
}

char ftos [safesize];
void writeSerial()
{
  memset(write_buffer,'\0',BUFSIZE);
  for(int x = 0; x<SONAR_NUM;x++)
  {
    dtostrf(sonar_value[x] / 50.0 ,20,10,ftos);
    strcat(write_buffer, ftos);
    strcat(write_buffer, ",");
  }
  dtostrf(latitude,20,10,ftos);
  strcat(write_buffer, ftos);
  strcat(write_buffer, ",");
  dtostrf(longitude,20,10,ftos);
  strcat(write_buffer, ftos);
  strcat(write_buffer, ",");
  dtostrf(heading,20,10,ftos);
  strcat(write_buffer, ftos);
  strcat(write_buffer, "\n");
  Serial.write(write_buffer);
}

