
// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

// NOTE: UART will be disabled when servo is attached to pin 0 or 1.


#include <Servo.h> 

 
Servo myservo;  // create servo object to control a servo 
int pin = 30;
int potpin = 20;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
int pos=1500;
 
void setup() 
{ 
  Serial.begin(9600);

  myservo.attach(pin);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  pos =map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservo.write(pos);                  // sets the servo position according to the scaled value 
  Serial.println(pos);
  delay(15);                           // waits for the servo to get there 
} 
