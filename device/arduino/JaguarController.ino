#include <FRCmotor.h>

int gamemode = 1; //TELLS CLASS THAT ROBOT IS ENABLED

FRCmotor leftMotor; //DECLARE LEFT MOTOR CONTROLLER
FRCmotor rightMotor; //DECLARE RIGHT MOTOR CONTROLLER

const int maxBuffSize = 4; //DETERMINING THE DIGITS BUFFER SIZE
char buffChar[maxBuffSize]; //DECLARE ARRAY TO TAKE INPUT FROM SERIAL
int valueLeft;
int valueRight;


void setup() {
  
  //-------------------MOTOR CONTROLLER-----------------------------
  leftMotor.SetPort(10); //DECLARE ARDUINO PORT FOR MOTOR CONTROLLER SIGNAL
  rightMotor.SetPort(9);

  leftMotor.Set(0); //SET INITIAL MOTOR VALUES TO ZERO
  rightMotor.Set(0); //(100 MAX FORWARD, -100 MAX BACK)
  //----------------------------------------------------------------
  
  Serial.begin(9600);
  
}

void loop() {

  if(Serial.available()>0){ //WAIT FOR SERIAL COMMAND
    Serial.println("hello");
    delay(50); //--> OPTIONAL WAIT FOR REMAINING BITS
    //8 BIT ASCII CHARACTERS COME THROUGH SERIAL INTERFACE
    //GROUP THESE CHARACTERS INTO CHAR ARRAY FOR PROCESSING

    if(Serial.peek() == '[') Serial.read();


    //GET VALUE OF LEFT MOTOR FROM SERIAL -----------------------------
    int counter = 0;
    while(Serial.peek() != ','){
      buffChar[counter] = Serial.read();
      counter++;
    } valueLeft = atoi(buffChar)*-1; //CONVERT CHAR ARRAY TO INTEGER VALUE
    //-----------------------------------------------------------------

    Serial.read(); //REMOVE COMMA DELIMETER
    
    //GET VALUE OF RIGHT MOTOR FROM SERIAL ----------------------------
    counter = 0;
    while(Serial.peek() != ']'){
      buffChar[counter] = Serial.read();
      counter++;
    } valueRight = atoi(buffChar); //CONVERT CHAR ARRAY TO INTEGER VALUE
    //-----------------------------------------------------------------

    while(Serial.available()){
      Serial.read();
    }

    
    //Serial.flush(); // WARNING!!!! -- MAY CAUSE PROBLEMS WITH ARDUINO 1.0 OR >

       
  }
  
  leftMotor.Set(valueLeft);
  rightMotor.Set(valueRight);
  Serial.print("Left: ");
  Serial.print(valueLeft);
  Serial.print("   |   Right: ");
  Serial.println(valueRight);

  
  //delay(2000);
  

}
