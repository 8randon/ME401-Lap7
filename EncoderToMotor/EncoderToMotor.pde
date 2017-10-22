

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
signed int lookA[] = {-200,-150,-100,-50,0,50,100,150,200}; //for position
int lookD[] = {0,0,0,0,0,0,0,0,0}; //for corresponding distance values
int i; // for counter
int largestDistance; //

// Global variables for keeping track of position
static volatile char lastLeftA;
static volatile char lastLeftB;
static volatile bool errorLeft;
volatile long position = 0;

String motorType="Forward"; //TODO: Change according to the type of the motor.
double Kp=17,Ki=0,Kd=0; // Controller parameters. For the project you can change all three. 
int targetAngle=0;
int botAngle = 0;

float encoderToDeg = 90/200; // conversion constants
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



void setup() {
  
  // Set up the serial port in case we want output or input
  Serial.begin(9600);
  Serial.println("Timer Interrupt Encoder Test:");
  
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


void loop() {

  long newPosition = position;
  
  Kp = 17;
  Kd = 3.0;
  Ki = 0.5;
  
  
  for(i = 0; i<9;i++){
    
    // TODO:
    //write lookA[i] to motor
    targetAngle = lookA[i];
    delay(1000);
    Serial.println("targetAngle:");
    Serial.println(targetAngle);
    
    //lookD[i] = read in sensor value
    
    //READ FROM ONE OF THE SENSORS
    irSensor1Value1 = analogRead(irSensor1Pin1);
    f1 = -0.0612*irSensor1Value1 + 68.863;
    delay(100); //wait for the measurement to settle

    //IMPLEMENT READING TO DISTANCE CONVERSION FOR THE FIRST SENSOR
    //READ FROM THE OTHER SENSOR
    irSensor1Value2 = analogRead(irSensor1Pin2);
    f2 = 4536.1*pow(irSensor1Value2, -1.048);
    delay(100);//wait for the measurement to settle
    
    //decide which sensor to read from; short distance sensor will never read below 17cm when out of accuracy range
    if(f2 < 17){
      lookD[i] = f2;
    }
    else{
      lookD[i] = f1;
    }
    
  }
  
  largestDistance = lookD[0];
  Serial.println("largestDistance:");
  Serial.println(largestDistance);
  
  for(i = 0; i<8 ; i++){
    if(lookD[i] > lookD[i+1]){
    largestDistance = lookD[i+1];
    }
  }

  
  for(i = 0; lookD[i] != largestDistance; i++){
    //looking for largestDistance value in look array
  }
  
  
  botAngle = lookA[i];
  Serial.println("botAngle:");
  Serial.println(botAngle);
  
  
  if(botAngle<0){
   myServoL.writeMicroseconds(1300);
   myServoR.writeMicroseconds(1300);
   delay(int((0.0-targetAngle)*encoderToDeg*degToMs));
  }
  else if(botAngle>0){
   myServoL.writeMicroseconds(1700);
   myServoR.writeMicroseconds(1700);
   delay(int(targetAngle*encoderToDeg*degToMs-200));
  }
  else{
   myServoL.writeMicroseconds(1700);
   myServoR.writeMicroseconds(1300);
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





