//////////////////////PINS//////////////////////
/*
  Magnometer: VIN: 3.3V  SDA: 20  SCL: 21
  Motor - R: 9  L: 10
*/
///////////////////////////////////////////////
#include <FRCmotor.h>

int gamemode = 1; // Enables the FRCmotor library

//SerialRead/Write
#define BUFSIZE 256
#define SPEED_LIMIT 100


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
  leftMotor.SetPort(11); //DECLARE ARDUINO PORT FOR MOTOR CONTROLLER SIGNAL
  rightMotor.SetPort(10);
  leftMotor.Set(0); //SET INITIAL MOTOR VALUES TO ZERO
  rightMotor.Set(0); //(100 MAX FORWARD, -100 MAX BACK)
  
  Serial.begin(57600);
}

void loop() 
{
  readSerial();
  moveMotor();
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

