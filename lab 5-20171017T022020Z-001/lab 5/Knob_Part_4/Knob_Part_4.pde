// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

// NOTE: UART will be disabled when servo is attached to pin 0 or 1.


#include <Servo.h> 

 
Servo myservo;  // create servo object to control a servo 
 
//int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
int pulseWidth=1500;


void setup() 
{ 
  myservo.attach(30);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(9600); //refers to the correct pin given
} 
 
 
 
void loop() 
{ 
  //val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  //val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  //myservo.write(val);                  // sets the servo position according to the scaled value 
  
  
  if (Serial.available())
  {
   val = Serial.parseInt();
  }
  pulseWidth = map(val, -100, 100, 1300, 1700);
  myservo.writeMicroseconds(pulseWidth);
  
  delay(15);                            // waits for the servo to get there 
  Serial.println(pulseWidth);
  
} 
