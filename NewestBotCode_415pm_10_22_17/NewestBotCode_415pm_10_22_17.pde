

#include <SoftPWMServo.h>
#include <Servo.h>

int irSensor1Pin1 = A7;//long; long limit is 17cm; if close range senses below 17, listen to close sensor
int irSensor1Pin2 = A8;//short
float irSensor1Value1 = 0;
float irSensor1Value2 = 0;
float f1=0.0;
float f2=0.0;

Servo myServoR;
Servo myServoL;

//array for angles
//signed int lookA[] = {-200,-150,-100,-50,0,50,100,150,200}; //for position
signed int lookA[] = {-200,-100,0,100,200}; //for position
//int lookD[] = {0,0,0,0,0,0,0,0,0}; //for corresponding distance values
float lookD[] = {0,0,0,0,0};
int i; // for counter
int p; // another counter
float largestDistance; //largest distance detected out of all of the looks taken
int numReadingsPerLook = 1; //takes 1 readings per look, then averages them
float botDist;

long newPosition;

// Global variables for keeping track of position
static volatile char lastLeftA;
static volatile char lastLeftB;
static volatile bool errorLeft;
volatile long position = 0;

String motorType="Forward"; //TODO: Change according to the type of the motor.
double Kp=17,Ki=0,Kd=0; // Controller parameters. For the project you can change all three. 
int targetAngle=0;
int botAngle = 0;

float encoderToDeg = 0.45; // conversion constants
float degToMs = 8.333;

int errorAngle=0;
float incomingByte = 0;
String inString= "";

//initialization of the variables for the PID calculation
double input=0, lastinput=0, output=0, setpoint=0;
double error=0, iterm=0,dinput=0;
double kp=0, ki=0, kd=0;
double outmax=2400;
double outmin=-2400;
int pidSampleTime = 10;
double SampleTimeInSec = ((double)pidSampleTime/1000);
long counter=1; // This counter is used to determine when the control loop should run


//_________________________________________________________________________________________________________Limit Switches and Light Sensor
int Linter = 4;  // Left interrrupt -> digital pin 35
int Rinter = 3;  // Right Interupt -> digital pin 8
volatile int state = LOW; //state variable to be changed during an interrupt
volatile int colorState = LOW; //state variable to be changed during an interrupt


//speed pulsewidths: Right Servo
int speedZeroR = 1500;
int speedMinR = 1700;
int speedMaxR = 1300;

//speed pulsewidths: Left Servo
int speedZeroL = 1500;
int speedMinL = 1300;
int speedMaxL = 1700;

//count color strips
int redCounter = 0;
int blueCounter = 0;

bool found = false;

unsigned long start_time;

//------------------------------------------------------------------------------------------------------
//colordetector setup
int sensorPin = 0;    // select the analog input pin for the photoresistor
int bPin = 12;  // select the digital output pin for blue LED
int rPin = 13; // select the digital output pin for red LED
int sensorValue = 0;  // variable to store the value coming from the photoresistor
int maxblack[]={708,717}; // the max reading that will be obtained from a black surface {RED,BLUE}
int minwhite[]={438,379}; // the min reading that will be obtained from a white surface {RED,BLUE}
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
  
  // Set up the serial port in case we want output or input
  Serial.begin(9600);
  
  state = MAIN; // initializes state so that bot starts out going forward
  attachInterrupt(Linter, runLeftState, FALLING); // attaches left state to pin 38 to listen on
  attachInterrupt(Rinter, runRightState, FALLING); // attaches right state to pin 7 to listen on

//------------------------------------------------------------------------------------------------------
  // declare the LED pins as an OUTPUT:
  pinMode(rPin, OUTPUT);
  pinMode(bPin, OUTPUT);
//------------------------------------------------------------------------------------------------------

  // Set up the quadrature inputs
  pinMode(2, INPUT);
  pinMode(20, INPUT);
  errorLeft = false;
  lastLeftA = digitalRead(2);
  lastLeftB = digitalRead(20);
  
  //timer interrupt is to read the encoder position and generate the PID output
  attachCoreTimerService(MyCallback);
  
  // Set up the motor outputs
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  
  digitalWrite(3,0);
  digitalWrite(4,0);
  
  myServoR.attach(30);  // attaches the servo on pin J16 to the servo object
  myServoL.attach(31);  // attaches the servo on pin J17 to the servo object
  
  SoftPWMServoPWMWrite(3, 0);

}

//___________________________________________________________________________________________________________________
void loop() {

     Serial.println("STATE:");
     Serial.println(state);
     start_time = millis();
   while(millis()<start_time+50){
  }//wait
  
  switch (state)
  {
    case (MAIN):
      colorLoop();
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
      irDetect();
      fwd();
      sensorZero();
      //look for color
      
      break;
      
    case (WHITE):
      state = MAIN;
      break;

    case (BEACONDETECT):
      
      break;    
      
    default:
      Serial.println("ERROR - UNKNOWN STATE");
      break;
  }
  
}

// Begin the individal state functions

//___________________________________________________________________________________________________________________detectingRed
void detectingRed(){
  
  if (colorState != RED){
    Serial.println("RED COUNTER: ");
    redCounter++;
    Serial.println(redCounter);
  }
 
 state = MAIN;
}
//___________________________________________________________________________________________________________________detectingBlue
// Begin the individal state functions
void detectingBlue(){
  
  if (colorState != BLUE){
    Serial.println("BLUE COUNTER: ");
    blueCounter++;
    Serial.println(blueCounter);
  }
 state = MAIN;
}
//___________________________________________________________________________________________________________________detectingBlack
// Begin the individal state functions
void detectingBlack(){
  Serial.println("DETECTED BLACK!");
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMaxR);
  
  delay(1000);
  
  myServoL.writeMicroseconds(speedMinL);
  myServoR.writeMicroseconds(speedMaxR);
  
  delay(3000);
   // will now run end-of-maze protocol
}
//___________________________________________________________________________________________________________________runLeftState
void runLeftState()
{
  state = LEFT;
}
//___________________________________________________________________________________________________________________turnLeft
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
   while(millis()<start_time+200){
  }//wait 1 second
  
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMaxR);
  
  state = MAIN;
}
//___________________________________________________________________________________________________________________runRightState
void runRightState()
{
  state = RIGHT;
}
//___________________________________________________________________________________________________________________turnRight
void turnRight(){
//  Serial.println("\nRIGHT STATE: ");
//  Serial.print(state);
  
  // Goes backwards first
  myServoL.writeMicroseconds(speedMinL);
  myServoR.writeMicroseconds(speedMinR);

  start_time = millis();
   while(millis()<start_time+400){
  }//wait
  
  //code that makes bot go right
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMinR);

  start_time = millis();
  while (millis() < start_time + 200) {
  }//wait
  myServoL.writeMicroseconds(speedMaxL);
  myServoR.writeMicroseconds(speedMaxR);
  state = MAIN;  
}

//___________________________________________________________________________________________________________________fwd
void fwd(){
   myServoL.writeMicroseconds(1700);
   myServoR.writeMicroseconds(1300);
  // Serial.println("\n\nMoving fowrard\n\n");
}
//___________________________________________________________________________________________________________________stopServos
void stopServos(){
   myServoL.writeMicroseconds(1500);
   myServoR.writeMicroseconds(1500);
  // Serial.println("\n\nStopped Servos\n\n");
}
//___________________________________________________________________________________________________________________sensorZero
void sensorZero(){
  targetAngle = 0;
}
//___________________________________________________________________________________________________________timer interrupt ISR


uint32_t MyCallback(uint32_t currentTime) {
  //read the enconder position
  char newLeftA = digitalRead(2);
  char newLeftB = digitalRead(20);
  
  position += (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB);
  
  if((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB))
    {
        errorLeft = true;
    }
  
  lastLeftA = newLeftA;
  lastLeftB = newLeftB;
    
  //run the PID calculation if the sampling time elapsed since the previous calculation.
  if (counter % 100*pidSampleTime == 0)
  {
   input = position;           
   setpoint=map(targetAngle,-360,360,-1200,1200);
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
   error=setpoint-input;
   errorAngle=map(error,0,1200,0,360);
   iterm +=ki*error;
   if (iterm>outmax) iterm=outmax;
   else if (iterm<outmin) iterm=outmin;
   dinput=input-lastinput;
   output=kp*error+iterm-kd*dinput;
   if(output > outmax) output = outmax;
   else if(output < outmin) output = outmin;
   lastinput=input;
   
    if ((motorType== "Forward" && output < 0) || (motorType== "Reverse" && output > 0))
    {
      digitalWrite(4,1);
      
    }
    else
    {
      digitalWrite(4,0);
    }  
    SoftPWMServoPWMWrite(3,abs(output));
    counter = 0;
  }
  counter++;
  
  return (currentTime + CORE_TICK_RATE/100);
}


