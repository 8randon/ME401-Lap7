#include <Servo.h>
Servo myServoR;
Servo myServoL;

int Linter = 0;  // Left interrrupt -> digital pin 38
int Rinter = 2;  // Right Interupt -> digital pin 7
volatile int state = LOW; //state variable to be changed during an interrupt


//speed pulsewidths: Right Servo
int speedZeroR = 1500;
int speedMinR = 1700;
int speedMaxR = 1300;

//speed pulsewidths: Keft Servo
int speedZeroL = 1500;
int speedMinL = 1300;
int speedMaxL = 1700;

//count color strips
int redCounter = 0;
int blueCounter = 0;

unsigned long start_time;

//------------------------------------------------------------------------------------------------------
//colordetector setup
int sensorPin = 0;    // select the analog input pin for the photoresistor
int bPin = 12;  // select the digital output pin for blue LED
int rPin = 13; // select the digital output pin for red LED
int sensorValue = 0;  // variable to store the value coming from the photoresistor
int maxblack[]={320,380}; // the max reading that will be obtained from a black surface {RED,GREEN,BLUE}
int minwhite[]={195,193}; // the min reading that will be obtained from a white surface {RED,GREEN,BLUE}
int color[]={0,0}; // array for the readings
//------------------------------------------------------------------------------------------------------


enum StateMachineState {
  MAIN = 0,
  LEFT = 1,
  RIGHT = 2,
  RED = 3,
  BLUE = 4,
  WHITE = 5,
  BLACK = 6,
  BEACONDETECT = 7,
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

//------------------------------------------------------------------------------------------------------
  // declare the LED pins as an OUTPUT:
  pinMode(rPin, OUTPUT);
  pinMode(bPin, OUTPUT);
//------------------------------------------------------------------------------------------------------
}

void loop() { // decides what action to take based on state

  colorLoop();
  
  switch (state)
  {
    case (MAIN):
      runMainState();
      break;

    case (LEFT):
      turnLeft();
      break;

    case (RIGHT):
      turnRight();
      break;
      
    case (RED):
      detectingRed();
      break;

    case (BLUE):
      detectingBlue();
      break;

    case (BLACK):
      detectingBlack();
      break;
      
    case (WHITE):
      
      break;

    case (BEACONDETECT):

      break;    
      
    default:
      Serial.println("ERROR - UNKNOWN STATE");
      break;
  }
}

void colorLoop()
{
  // turn on the red led only
   digitalWrite(rPin, HIGH); //Set rPin to HIGH++++++++++++++++
   digitalWrite(bPin, LOW); //Set bPin to LOW+++++++++++++++
  
  start_time = millis();
   while(millis()<start_time+50){
  }//wait
  
  sensorValue = analogRead(sensorPin); // read the photoresistor value
  // record the red reading
  color[0]=sensorValue;
  
  //constrain the reading such that it is between the white and black values
  color[0]=constrain(sensorValue,minwhite[0],maxblack[0]);
  
  // map the reading between 0 and 100 such that black is 0, 100 is white
  color[0]=map(color[0],maxblack[0],minwhite[0],0,100);
  
  // output the reading
  Serial.print("Red: ");Serial.print(color[0]);Serial.print(" "); 

 
  digitalWrite(bPin, HIGH); //Set bPin to HIGH++++++++++++++++
  digitalWrite(rPin, LOW); //Set rPin to LOW+++++++++++++++
  
  start_time = millis();
   while(millis()<start_time+50){
  }//wait
  
  sensorValue = analogRead(sensorPin); 
  color[1]=sensorValue;
  color[1]=constrain(sensorValue,minwhite[1],maxblack[1]);
  color[1]=map(color[1],maxblack[1],minwhite[1],0,100);
  Serial.print("Blue: ");Serial.print(color[1]);Serial.print(" ");  
  Serial.println("");
  
  //red color detected
  if(color[0]>color[1])
  {
    Serial.println("RED!");
    state = RED;
  }
  //blue color detected
  else if(color[1]>color[0])
  {
    Serial.println("BLUE!");
    state = BLUE;
  }
}

// Begin the individal state functions
void detectingRed(){
 redCounter++;
}

// Begin the individal state functions
void detectingBlue(){
 blueCounter++;
}

// Begin the individal state functions
void detectingBlack(){

}

void runMainState() //need to modify; implementatio has changed++++++++++++++
{
//  Serial.println("\nMAIN STATE: ");
//  Serial.print(state);

  //code that makes bot go straight
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMaxR);
}


void runLeftState()
{
  state = LEFT;
}

void turnLeft() {
//  Serial.println("\nLEFT STATE: ");
//  Serial.print(state);

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
//  Serial.println("\nRIGHT STATE: ");
//  Serial.print(state);
  
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
