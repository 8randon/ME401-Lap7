#include <SoftPWMServo.h>

//array for angles
//angle calibration: 2000/90 (reading/degrees)
int i; // for counter
int largestDistance;

// Global variables for keeping track of position
static volatile char lastLeftA;
static volatile char lastLeftB;
static volatile bool errorLeft;
volatile long position = 0;

String motorType="Forward"; //TODO: Change according to the type of the motor.
double Kp=17,Ki=0,Kd=0; // Controller parameters. For the project you can change all three. 
int targetAngle=0;
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
  
  SoftPWMServoPWMWrite(3, 0);

}


void loop() {

  long newPosition = position;
  
  Kp = 17;
  Kd = 3.0;
  Ki = 0.5;
//  
//
//  for(i = 0; i<9;i++){
//    
//    // TODO:
//    //look[i][0] = read in sensor value
//  }
//  
//  largestDistance = max(look[0][1],look[1][1],look[2][1],look[3][1],look[4][1],look[5][1],look[6][1],look[7][1],look[8][1]);
//
//  for(i = 0; look[i][1] != largestDistance; i++){
//    //looking for largestDistance value in look array
//  };
//  
//  targetAngle = look[i][0];
  
  targetAngle += 60;
  delay(1000);
   targetAngle += 60;
  delay(1000);
   targetAngle += 60;
  delay(1000);
  targetAngle += 60;
  delay(1000);
  targetAngle -= 420;
  delay(1000);
  Serial.print("Target Angle: ");Serial.print(targetAngle); Serial.print(" ");
  Serial.print("Angle Error: ");Serial.print(errorAngle); Serial.print(" ");
  Serial.print("Kp: ");Serial.print(Kp); Serial.print(" ");
  Serial.print("Kd: ");Serial.print(Kd); Serial.print(" ");
  Serial.print("Ki: ");Serial.print(Ki); Serial.print(" ");
  Serial.println("");
  
    targetAngle += 60;
  delay(1000);
   targetAngle += 60;
  delay(1000);
   targetAngle += 60;
  delay(1000);
  Serial.print("Target Angle: ");Serial.print(targetAngle); Serial.print(" ");
  Serial.print("Angle Error: ");Serial.print(errorAngle); Serial.print(" ");
  Serial.print("Kp: ");Serial.print(Kp); Serial.print(" ");
  Serial.print("Kd: ");Serial.print(Kd); Serial.print(" ");
  Serial.print("Ki: ");Serial.print(Ki); Serial.print(" ");
  Serial.println("");
  
  
//  if (Serial.available() > 0) 
//  {
//    // read the incoming byte:
//    incomingByte = Serial.read();
//
//    if (incomingByte == 'a') // "a" to vary the target angle in one direction by 90 degrees.
//    {
//      targetAngle += 180; 
//    }
//    else if (incomingByte == 'z')// "z" to vary the target angle in one direction by 90 degrees.
//    {
//      targetAngle -= 180;
//    }   
//    //code for seniding out the proportional gain
//    else if (incomingByte != '\n')
//    {
//    inString += (char)incomingByte;
//    }
//    else if(inString != "") {
//    //Kp =inString.toFloat();
//    targetAngle =inString.toFloat();
//    inString = "";
//     }
     
//  Serial.print("Target Angle: ");Serial.print(targetAngle); Serial.print(" ");
//  Serial.print("Angle Error: ");Serial.print(errorAngle); Serial.print(" ");
//  Serial.print("Kp: ");Serial.print(Kp); Serial.print(" ");
//  Serial.print("Kd: ");Serial.print(Kd); Serial.print(" ");
//  Serial.print("Ki: ");Serial.print(Ki); Serial.print(" ");
//  Serial.println("");
//  }
  delay(100);      
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





