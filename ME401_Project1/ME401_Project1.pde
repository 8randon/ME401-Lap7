// ALL VARIABLE ARE UNDEFINED AT THE MOMENT, NEED TO BECOME GLOBAL
// ALL VARIABLE ARE UNDEFINED AT THE MOMENT, NEED TO BECOME GLOBAL
// ALL VARIABLE ARE UNDEFINED AT THE MOMENT, NEED TO BECOME GLOBAL

#include <Servo.h> 

// you can add as many states here as you want
enum StateMachineState {
  STARTUP = 0,
  MAZE = 1,
  DETECT = 2,
  FINISH = 3,
};

Servo servoLeft;  // create servo object to control a servo 
Servo servoRight;  // create servo object to control a servo 

int redCount = 0;    
int blueCount = 0;
int servoPinLeft = 9;
int servoPinRight = 8;  // FIX
int pulseWidthLeft = 1500;  // between 1300 and 1700 as min and max values
int pulseWidthRight = 1500;  // between 1300 and 1700 as min and max values


// IR DISTANCE SENSOR -----------------------------------------------------------------------------------------------------
int irSensor1Pin = A1;
int irSensor2Pin = A2;
int irSensor1Value;
int irSensor2Value;
double distance1;
double distance2;


// Color Detect -----------------------------------------------------------------------------------------------------
int sensorPin = 0;    // select the analog input pin for the photoresistor
int bPin = 9;  // select the digital output pin for blue LED
//int gPin = 10; // select the digital output pin for green LED 
int rPin = 11; // select the digital output pin for red LED
int sensorValue = 0;  // variable to store the value coming from the photoresistor
int maxblack[]={482, 527, 529}; // the max reading that will be obtained from a black surface {RED,GREEN,BLUE}
int minwhite[]={446,435,490}; // the min reading that will be obtained from a white surface {RED,GREEN,BLUE}
int color[]={0,0,0}; // array for the readings
int currentColor = 0;  //(0 - white, 1 - red, 2 - blue, 3 - black)
int prevColor = 0;     //(0 - white, 1 - red, 2 - blue, 3 - black)
int padColor = 0;

// Make the current state a global variable
StateMachineState state;


// PID CONTROL-----------------------------------------------------------------------------------------------------
#include <SoftPWMServo.h>

uint32_t MyCallback(uint32_t currentTime);

// Global variables for keeping track of position
static volatile char lastLeftA;
static volatile char lastLeftB;
static volatile bool errorLeft;
volatile long position = 0;

String motorType="Forward"; //TODO: Change according to the type of the motor.
double Kp=5,Ki=0,Kd=0; // Controller parameters. For the project you can change all three. 
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

// -----------------------------------------------------------------------------------------------------




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("STARTING SETUP");
  // MAKE SURE YOU INITIALIZE ALL THE I/O PINS YOU NEED TO USE
  state = MAZE;
  
  servoLeft.attach(servoPinLeft);  // attaches the servo to the servo object 
  servoRight.attach(servoPinRight);  // attaches the servo to the servo object 
  
  // Color Sensor
  // declare the LED pins as an OUTPUT:
  pinMode(rPin, OUTPUT);
  //pinMode(gPin, OUTPUT);
  pinMode(bPin, OUTPUT);
  Serial.begin(9600) ; //initialize the serial communication
  
  // PID CONTROL --------------------------------------------------------------------------------------------------
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
  // --------------------------------------------------------------------------------------------------
  
}

void loop() {
  // put your main code here, to run repeatedly: 

  
  switch (state)
  {
    case(STARTUP):  // Waits to be turned on
      //runStartupState()
          // interrupt running until button is pressed
      break;
    case(MAZE):
      
      runColorDetect();  // runs color detect program
      getColorChange();    // counts color changes
      get_distance();    // gets distance
      runMazeState();
      break; 
      
    case(DETECT):
      beaconScan(180.0);    // scan for max IR beacon
      moveForward();
      break;

    default:
      Serial.println("ERROR - UNKNOWN STATE");
      break;
  }
  delay(100); // delay between state changes to allow time for the switch to be released 
}

void beaconScan(double angle)
{
  // rotate until it finds the max
  
}



// Begin the individal state functions
void runStartupState()
{
  // intterupt
}


void runMazeState()
{
  if (distance1 > 10) // if distance sensor > 20 cm
  {
      pulseWidthLeft = 1600;
      pulseWidthRight = 1600;
      servoLeft.writeMicroseconds(pulseWidthLeft);                  // sets the servo position according to the scaled value 
      servoRight.writeMicroseconds(pulseWidthRight);                  // sets the servo position according to the scaled value 
      delay(500);                           // waits for the servo to get there 
  }else
  {
    // turn left 90 degrees
     servoLeft.writeMicroseconds(pulseWidthLeft);                  // sets the servo position according to the scaled value 
     servoRight.writeMicroseconds(-pulseWidthRight);                  // sets the servo position according to the scaled value 
     delay(500);
     get_distance();
    
    if (distance1 < 10) //   if distance sensor < 20 cm
    {
       // turn 180 degrees arond
       servoLeft.writeMicroseconds(pulseWidthLeft);                  // sets the servo position according to the scaled value 
       servoRight.writeMicroseconds(-pulseWidthRight);                  // sets the servo position according to the scaled value 
       delay(1000);
    }
  }
}

/*
void runLeftState()
{
    // turns for a set amount of time
 
     if (distance1 > 20)  // if distance sensor > 20 cm
     {
       state = TURNAROUND;
     }
     
}
*/

void runRightState()
//REPEAT THE runMainState() CODE HERE, MAKING THE APPROPRIATE MODIFICATIONS
{
  

}
void moveForward()
{
      pulseWidthLeft = 1600;
      pulseWidthRight = 1600;
      servoLeft.writeMicroseconds(pulseWidthLeft);                  // sets the servo position according to the scaled value 
      servoRight.writeMicroseconds(pulseWidthRight);                  // sets the servo position according to the scaled value 
  
}


void getColorChange()  // ALL VARIABLE ARE UNDEFINED AT THE MOMENT, NEED TO BECOME GLOBAL
{
   if (currentColor != prevColor)
   {
     if (prevColor = 0) // if previous color is white
     {
        if (currentColor = 1) // if new color is red
       {
         redCount+=1;
       } 
       else if (currentColor = 2)  // if new color is blue
       {
         blueCount+=1;
       }
       else  // black is detected, exits maze and begins looking for IR beacon
       {
         state = DETECT;
         if (redCount > blueCount)
         {
           padColor = 1;  // end pad color is red
         }
         else
         {
           padColor = 2;  // end pad color is blue
         }
         servoLeft.writeMicroseconds(pulseWidthLeft);                  // sets the servo position according to the scaled value 
         servoRight.writeMicroseconds(pulseWidthRight);                  // sets the servo position according to the scaled value 
         delay(500);                           // waits for the servo to get there 
       }
     }
   }
}


void runColorDetect()      // ALL VARIABLE ARE UNDEFINED AT THE MOMENT, NEED TO BECOME GLOBAL
{
    // turn on the red led only
  //WRITE A SET OF COMMANDS TO LIGHT UP THE RED LED AND KEEP THE OTHERS OFF.
  digitalWrite(rPin, LOW);
  digitalWrite(bPin, HIGH);
  //digitalWrite(gPin, HIGH);
  delay(100); //wait for the photresistor value to settle
  sensorValue = analogRead(sensorPin); // read the photoresistor value
  color[0]=sensorValue; // record the red reading
  color[0]=constrain(sensorValue,minwhite[0],maxblack[0]); //constrain the reading such that it is between the white and black values
  color[0]=map(color[0],maxblack[0],minwhite[0],0,100); // map the reading between 0 and 100 such that black is 0, 100 is white
  Serial.print("Red: ");Serial.print(color[0]);Serial.print(" "); // output the reading
 /*
  // repeat the above procedure for green 
  // WRITE A SET OF COMMANDS TO LIGHT UP THE GREEN LED AND KEEP THE OTHERS OFF.
   digitalWrite(rPin, HIGH);
  digitalWrite(bPin, HIGH);
  digitalWrite(gPin, LOW);
  delay(100);
  sensorValue = analogRead(sensorPin); 
  color[1]=sensorValue;
  color[1]=constrain(sensorValue,minwhite[1],maxblack[1]);
  color[1]=map(color[1],maxblack[1],minwhite[1],0,100);
  Serial.print("Green: ");Serial.print(color[1]);Serial.print(" ");
 */
  // repeat the above procedure for blue
  //WRITE A SET OF COMMANDS TO LIGHT UP THE BLUE LED AND KEEP THE OTHERS OFF.
   digitalWrite(rPin, HIGH);
  digitalWrite(bPin, LOW);
  //digitalWrite(gPin, HIGH);
  delay(100);
  sensorValue = analogRead(sensorPin); 
  color[2]=sensorValue;
  color[2]=constrain(sensorValue,minwhite[2],maxblack[2]);
  color[2]=map(color[2],maxblack[2],minwhite[2],0,100);
  Serial.print("Blue: ");Serial.print(color[2]);Serial.print(" ");  
  Serial.println("");
  
  prevColor = currentColor;  // sets current color to previous color 
  
  if ((color[0] < 10) && (color[2] < 10))
  {
    currentColor = 3;  // current color is black
  }
  else if ((color[0] > 10) && (color[2] < 10))
  {
    currentColor = 1;  // current color is red 
  }
  else if ((color[0] < 10) && (color[2] > 10))
  {
    currentColor = 2;  // current color is blue
  }
  else
  {
    currentColor = 0;  // current color is white
  }
}


// DISTANCE FUNCTION ------------------------------------------------------------------------------------

void get_distance() {
 //READ FROM ONE OF THE SENSORS
irSensor1Value = analogRead(irSensor1Pin);
delay(100); //wait for the measurement to settle
//IMPLEMENT READING TO DISTANCE CONVERSION FOR THE FIRST SENSOR
distance1 = 2*11.428/(irSensor1Value*.0048875855+.42);
//READ FROM THE OTHER SENSOR
irSensor2Value = analogRead(irSensor2Pin);
delay(100);//wait for the measurement to settle
//IMPLEMENT READING TO DISTANCE CONVERSION FOR THE SECOND SENSOR
distance2 = 2*56.6/(irSensor2Value*.0048875855);
}


/*
//------------------------------------------------------ PID FUNCTIONS ------------------------------------------------------
void PID(double angle_adjustment) {
  
  long newPosition = position;
  targetAngle += angle_adjustment;

  //Serial.print("Target Angle: ");Serial.print(targetAngle); Serial.print(" ");
  //Serial.print("Angle Error: ");Serial.print(errorAngle); Serial.print(" ");
  //Serial.print("Kp: ");Serial.print(Kp); Serial.print(" ");
  //Serial.println("");
        
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
   setpoint=map(targetAngle,-360,360,-2400,2400);
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
   error=setpoint-input;
   errorAngle=map(error,0,2400,0,360);
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
*/

