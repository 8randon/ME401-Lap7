#include <Servo.h>
Servo myServoR;
Servo myServoL;

int Linter = 2;  // Left interrrupt
int Rinter = 3;  // Right Interupt
volatile int state = LOW; //state variable to be changed during an interrupt

int speedZeroR = 1500;
int speedMinR = 1475;
int speedMedR = 1400;
int speedMaxR = 1300;

int speedZeroL = 1500;
int speedMinL = 1525;
int speedMedL = 1600;
int speedMaxL = 1700;


unsigned long start_time;

enum StateMachineState {
  MAIN = 0,
  LEFT = 1,
  RIGHT = 2,
};

int state;


//int PIN_L = 6;
//int PIN_R = 7;


bool lstate = false;
bool rstate = false;

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING SETUP");
  
  state = MAIN;
  pinMode(PIN_R, INPUT);
  pinMode(PIN_L, INPUT);
  
  myServoR.attach(30);  // attaches the servo on pin J16 to the servo object
  myServoL.attach(31);  // attaches the servo on pin J17 to the servo object
  
  attachInterrupt(Linter,runLeftState, FALLING);
  attachInterrupt(Rinter,runRightState, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly: 
  //Serial.println("STARTING LOOP");
  //Serial.println(state);
  switch (state)
  {
    case(MAIN):
      runMainState();
      break; 
      
    case(LEFT):
      runLeftState();
      break;

    case(RIGHT):
      runRightState();
      break;
      
    default:
      Serial.println("ERROR - UNKNOWN STATE");
      break;
  }
}



// Begin the individal state functions
void runMainState()
{
  state = MAIN;
  start_time = millis();
  while(millis()<start_time+100){
}//wait 1 second

  //lstate = digitalRead(PIN_L);
  //rstate = digitalRead(PIN_R);
  
  Serial.println("\nMAIN STATE: ");
  Serial.print(state);
  if(rstate == false){
    state = RIGHT;
    runRightState();
  }
   if(lstate == false){
    state = LEFT;
    runLeftState();
  }  
  //code that makes bot go straight
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMaxR);
  

}


void runLeftState()
{

  //rstate = digitalRead(PIN_R);
  
  state = LEFT;
  Serial.println("\nLEFT STATE: ");
  Serial.print(state);
 
 if(rstate == false){
    state = RIGHT;
    runMainState();
  }

  //code that makes bot go left
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedZeroR);
  
  start_time = millis();
  while(millis()<start_time+1000){
}//wait 1 second
}


void runRightState()
{

  //lstate = digitalRead(PIN_L);
  
  
  Serial.println("\nRIGHT STATE: ");
  Serial.print(state);
 
 if(lstate == false){
    state = LEFT;
    runMainState();
  }
  
  //code that makes bot go right
  myServoL.writeMicroseconds(speedZeroL);
  myServoR.writeMicroseconds(speedMaxR);
  
  start_time = millis();
  while(millis()<start_time+1000){
}//wait 1 second
}
