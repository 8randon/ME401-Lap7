

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
int Linter = 0;  // Left interrrupt -> digital pin 38
int Rinter = 2;  // Right Interupt -> digital pin 7
volatile int state = LOW; //state variable to be changed during an interrupt


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
   while(millis()<start_time+500){
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
//___________________________________________________________________________________________________________________
void colorLoop()
{
  // turn on the red led only
   digitalWrite(rPin, HIGH); //Set rPin to HIGH++++++++++++++++
   digitalWrite(bPin, LOW); //Set bPin to LOW+++++++++++++++
  
  start_time = millis();
   while(millis()<start_time+100){
  }//wait
  
  sensorValue = analogRead(sensorPin); // read the photoresistor value
  // record the red reading
  color[0]=sensorValue;
  
  //constrain the reading such that it is between the white and black values
  color[0]=constrain(sensorValue,minwhite[0],maxblack[0]);
  
  // map the reading between 0 and 100 such that black is 0, 100 is white
  //color[0]=map(color[0],maxblack[0],minwhite[0],0,100);
  
  // output the reading
  //Serial.print("Red: ");Serial.print(color[0]);Serial.print(" "); 


  digitalWrite(rPin, LOW); //Set rPin to LOW+++++++++++++++ 
  digitalWrite(bPin, HIGH); //Set bPin to HIGH++++++++++++++++

  
  start_time = millis();
   while(millis()<start_time+100){
  }//wait
  
  sensorValue = analogRead(sensorPin); 
  color[1]=sensorValue;
  color[1]=constrain(sensorValue,minwhite[1],maxblack[1]);
  //color[1]=map(color[1],maxblack[1],minwhite[1],0,100);
  //Serial.print("Blue: ");Serial.print(color[1]);Serial.print(" ");  
  //Serial.println("");
  
  
  
  //white color detected
  if(color[0] < 460 && color[1] < 485)
  {
    //Serial.println("WHITE!");
    state = WHITE;
  }
  //black color detected
  else if(color[1] > 710 && color[0] > 700)
  {
    //Serial.println("BLACK!");
    state = BLACK;
  }
   //Red color detected
  else if(color[0]<color[1])
  {
    //Serial.println("RED!");
    state = RED;
  }
  //Blue color detected
  else if(color[1]<color[0])
  {
    //Serial.println("BLUE!");
    state = BLUE;
  }

}
//___________________________________________________________________________________________________________________
// Begin the individal state functions
void detectingRed(){
 Serial.println("RED COUNTER: ");
 redCounter++;
 Serial.println(redCounter);
 state = MAIN;
}
//___________________________________________________________________________________________________________________
// Begin the individal state functions
void detectingBlue(){
 Serial.println("BLUE COUNTER: ");
 blueCounter++;
 Serial.println(blueCounter);
 state = MAIN;
}
//___________________________________________________________________________________________________________________
// Begin the individal state functions
void detectingBlack(){
  Serial.println("DETECTED BLACK!");
  state = MAIN;
}
//___________________________________________________________________________________________________________________
void runMainState() //need to modify; implementatio has changed++++++++++++++
{
//  Serial.println("\nMAIN STATE: ");
//  Serial.print(state);

  //code that makes bot go straight
    fwd();
    
  //READ FROM f1 (long distance)
//      start_time = millis();
//      while(millis()<start_time+50){
//      } //wait for the measurement to settle
      
      irSensor1Value1 = analogRead(irSensor1Pin1);
      f1 = -0.0612*irSensor1Value1 + 68.863;
      
      //READ FROM f2 (short distance)
      //delay(50);//wait for the measurement to settle
      irSensor1Value2 = analogRead(irSensor1Pin2);
      f2 = 4536.1*pow(irSensor1Value2, -1.048);
      
//      Serial.print("Sensor1:");
//      Serial.print("        ");
//      Serial.print("Sensor2:");
//      Serial.println("");
//      Serial.print(f1);
//      Serial.print("        ");
//      Serial.print(f2);
//      Serial.println("");
      
    if(f2 < 17){
      botDist = f2;
//      Serial.println("Printing from Short Range");
    }
    else{
      botDist = f1;
//       Serial.println("Printing from Long Range");
    }
    
//     Serial.print("        ");
//      Serial.println("Bot Distance");
//      Serial.println(botDist);
    
    if(botDist<17){
      stopServos();
      distance();
      stopServos();
      sensorZero();
      start_time = millis();
      while(millis()<start_time+50){
      }
    }
      else{
      fwd();
    }
//     Serial.println("Ran Main");
    
}
//___________________________________________________________________________________________________________________

void runLeftState()
{
  state = LEFT;
}
//___________________________________________________________________________________________________________________
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
//___________________________________________________________________________________________________________________
void runRightState()
{
  state = RIGHT;
}
//___________________________________________________________________________________________________________________
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
  while (millis() < start_time + 400) {
  }//wait

  state = MAIN;  
}

//___________________________________________________________________________________________________________________
void fwd(){
   myServoL.writeMicroseconds(1700);
   myServoR.writeMicroseconds(1300);
  // Serial.println("\n\nMoving fowrard\n\n");
}

void stopServos(){
   myServoL.writeMicroseconds(1500);
   myServoR.writeMicroseconds(1500);
  // Serial.println("\n\nStopped Servos\n\n");
}

void sensorZero(){
  targetAngle = 0;
}

void distance(){
//  Serial.println("\n\ntargetAngle:");
 // Serial.println("\n\nChecking Distance\n\n");
  newPosition = position;
  
  Kp = 17;
  Kd = 3.0;
  Ki = 0.5;
  
  
  for(i = 0; i<5;i++){
    
    // TODO:
    //write lookA[i] to motor
    targetAngle = lookA[i];
    start_time = millis();
    while (millis() < start_time + 500) {
  }//wait 1 second
//    Serial.println("targetAngle:");
//    Serial.println(targetAngle);
    
    //resetting sensor readings
    f1 = 0;
    f2 = 0;
    
    
    //TAKING THE AVERAGE DISTANCE READING AT A POSITION
    
    for(p = 0; p<numReadingsPerLook; p++){ //takes multiple readings at each position
      
      //READ FROM f1 (long distance)
      start_time = millis();
      while (millis() < start_time + 50) {
      }//wait 1 second
      //delay(50); //wait for the measurement to settle
      irSensor1Value1 = analogRead(irSensor1Pin1);
      f1 += -0.0612*irSensor1Value1 + 68.863;
      
      //READ FROM f2 (short distance)
      //delay(50);//wait for the measurement to settle
      irSensor1Value2 = analogRead(irSensor1Pin2);
      f2 += 4536.1*pow(irSensor1Value2, -1.048);
      
    }
    
    //takes average of readings to account for servo swivel
    f1 /= numReadingsPerLook;
    f2 /= numReadingsPerLook;
    
//          //READ FROM f1 (long distance)
//      delay(50); //wait for the measurement to settle
//      irSensor1Value1 = analogRead(irSensor1Pin1);
//      f1 += -0.0612*irSensor1Value1 + 68.863;
//      
//      //READ FROM f2 (short distance)
//      //delay(50);//wait for the measurement to settle
//      irSensor1Value2 = analogRead(irSensor1Pin2);
//      f2 += 4536.1*pow(irSensor1Value2, -1.048);
//    
    
    
    //decide which sensor to read from; (short distance sensor will never read below 17cm when out of accuracy range)
    if(f2 < 17){
      lookD[i] = f2;
//      Serial.println("Printing from Short Range:  ");
//      Serial.println(lookD[i]);
    }
    else{
      lookD[i] = f1;
//      Serial.println("Printing from Long Range:  ");
//      Serial.println(lookD[i]);
    }
    
  }
  
  // INITIALIZES THE LARGEST DISTANCE FOR THE LOOP
  largestDistance = lookD[0];
//  Serial.println("largestDistance:");

  
  if(largestDistance < lookD[1]){
    largestDistance = lookD[1];
//    Serial.println("1largestDistance:");

  }
  
  if(largestDistance < lookD[2]){
    largestDistance = lookD[2];
//    Serial.println("2largestDistance:");

  }
  
    if(largestDistance < lookD[3]){
    largestDistance = lookD[3];
//    Serial.println("3largestDistance:");

  }
  
  if(largestDistance < lookD[4]){
    largestDistance = lookD[4];
//    Serial.println("4largestDistance:");

  }
  //loops through whole distance value array to find the largest value
//  for(i = 0; i<2 ; i++){
//    if(lookD[i] > lookD[i+1]){
//    largestDistance = lookD[i+1];
//      Serial.println("largestDistance:");
//      Serial.println(largestDistance);
//    }
//  }


  
  
  // finds out where that value is in the array
  for(i = 0; lookD[i] != largestDistance; i++){
    //looking for largestDistance value in look array
  }
  
  // uses that index to find the corresponding angle at which that value was read
  botAngle = lookA[i];
  
//  Serial.println("botAngle:");
//  Serial.println(botAngle);
  
  start_time = millis();
    while (millis() < start_time + 100) {
  }//wait 1 second
  
  //determines which way to turn based on the ongle at which the greatest value was found
  
  if(botAngle<0){ // turn right
   myServoL.writeMicroseconds(1700);
   myServoR.writeMicroseconds(1700);
//   Serial.println("TURN RIGHT");
//   Serial.println("Delay: ");
//   Serial.println(int(abs(botAngle)*encoderToDeg*degToMs));
//   Serial.println("bot angle: ");
//   Serial.println(botAngle);
//   Serial.println("encoder to deg: ");
//   Serial.println(encoderToDeg);
//   Serial.println("deg to mss: ");
//   Serial.println(degToMs);
   //delay(int(abs(botAngle)*encoderToDeg*degToMs));
   start_time = millis();
   while(millis()<start_time+int(abs(botAngle)*encoderToDeg*degToMs)){
    }
  }
  else if(botAngle>0){// turn left
   myServoL.writeMicroseconds(1300);
   myServoR.writeMicroseconds(1300);
//   Serial.println("TURN LEFT");
//   Serial.println("Delay: ");
//   Serial.println((botAngle)*encoderToDeg*degToMs);
//   Serial.println("bot angle: ");
//   Serial.println(botAngle);
//   Serial.println("encoder to deg: ");
//   Serial.println(encoderToDeg);
//   Serial.println("deg to mss: ");
//   Serial.println(degToMs);
   //delay(int(botAngle*encoderToDeg*degToMs+200));
   start_time = millis();
   while(millis()<start_time+int(abs(botAngle)*encoderToDeg*degToMs+200)){
    }
  } 
}

//timer interrupt ISR
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





