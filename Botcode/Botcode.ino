#include <Servo.h>
Servo myServoR;
Servo myServoL;

int Linter = 0;  // Left interrrupt -> digital pin 38
int Rinter = 2;  // Right Interupt -> digital pin 7
volatile int state = LOW; //state variable to be changed during an interrupt

// have distance sensor look left and right once it gets a
// certain distance from a wall to determine which way to turn

int speedZeroR = 1500;
int speedMinR = 1700;
//int speedMedR = 1400;
int speedMaxR = 1300;

int speedZeroL = 1500;
int speedMinL = 1300;
int speedMedL = 1600;
int speedMaxL = 1700;


unsigned long start_time;

enum StateMachineState {
  MAIN = 0,
  LEFT = 1,
  RIGHT = 2,
};



bool lstate = false;
bool rstate = false;

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING SETUP");

  state = MAIN; // initializes state so that bot starts out going forward

  myServoR.attach(30);  // attaches the servo on pin J16 to the servo object
  myServoL.attach(31);  // attaches the servo on pin J17 to the servo object

  attachInterrupt(Linter, runLeftState, FALLING); // attaches left state to pin 38 to listen on
  attachInterrupt(Rinter, runRightState, FALLING); // attaches right state to pin 7 to listen on
}

void loop() { // decides what action to take based on state

  switch (state)
  {
    case (MAIN):
      runMainState();
      break;

    case (LEFT): // may no longer be required; replace with other implementation++++++++++++++
      turnLeft();
      break;

    case (RIGHT): // may no longer be required; replace with other implementation++++++++++++++
      turnRight();
      break;

    default:
      Serial.println("ERROR - UNKNOWN STATE");
      break;
  }
}



// Begin the individal state functions


void runMainState() //need to modify; implementatio has changed++++++++++++++
{


  Serial.println("\nMAIN STATE: ");
  Serial.print(state);

  //code that makes bot go straight
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMaxR);

}


void runLeftState()
{
  state = LEFT;
}

void turnLeft() {
  Serial.println("\nLEFT STATE: ");
  Serial.print(state);

  // Goes backwards first
  myServoL.writeMicroseconds(speedMinL);
  myServoR.writeMicroseconds(speedMinR);

  start_time = millis();
   while(millis()<start_time+400){
  }//wait 1 second
  
  //code that makes bot go left
  myServoL.writeMicroseconds(speedMinL);
  myServoR.writeMicroseconds(speedMaxR);

  start_time = millis();
   while(millis()<start_time+400){
  }//wait 1 second

  state = MAIN;
}

void runRightState()
{
  state = RIGHT;
}

void turnRight(){
  Serial.println("\nRIGHT STATE: ");
  Serial.print(state);
  
  // Goes backwards first
  myServoL.writeMicroseconds(speedMinL);
  myServoR.writeMicroseconds(speedMinR);

  start_time = millis();
   while(millis()<start_time+400){
  }//wait 1 second
  
  //code that makes bot go right
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMinR);

  start_time = millis();
  while (millis() < start_time + 400) {
  }//wait 1 second

  state = MAIN;  
}
