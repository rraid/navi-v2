//////////////////////PINS//////////////////////
/*
  Magnometer: VIN: 3.3V  SDA: 20  SCL: 21
  Motor - R: 9  L: 10
*/
///////////////////////////////////////////////
#include <FRCmotor.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

int gamemode = 1; // Enables the FRCmotor library

//SerialRead/Write
#define BUFSIZE 256
#define SPEED_LIMIT 100

//Gyro-Compass
float heading = 0.0;

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
  gyroSetup();
}

void loop() 
{
  //readSerial();
  moveMotor();
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
  leftMotor.Set(-1*target_vel[0]);
  rightMotor.Set(target_vel[1]);
  //Serial.print(target_vel[0]);
  //Serial.println(target_vel[1]);
}



void getCompass(){
  heading = gyroLoop() *180/M_PI;
}


char ftos [safesize];
void writeSerial()
{
  memset(write_buffer,'\0',BUFSIZE);
  strcat(write_buffer, "[uno,");
  dtostrf(heading,20,10,ftos);
  strcat(write_buffer, ftos);
  strcat(write_buffer, "]\n");
  Serial.write(write_buffer);
}

