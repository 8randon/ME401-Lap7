/*
 ArdBot interrupt bumper demo 
 Requires Arduino IDE version 0017 
  or later (0019 or later preferred)
*/

#include <Servo.h> 

const int ledPin =  13;
const int bumpLeft = 6;
const int bumpRight = 7;
int pbLeft = 0;
int pbRight = 0;
Servo servoLeft;
Servo servoRight;
                 
void setup() { 
  servoLeft.attach(30);
  servoRight.attach(31);
  // Set pin modes
  pinMode(bumpLeft, INPUT);  
  pinMode(bumpRight, INPUT);  
  pinMode(ledPin, OUTPUT);

  // Set up interrupts
  attachInterrupt(0, hitLeft, RISING);
  attachInterrupt(1, hitRight, RISING);
} 
 
void loop() { 
  forward();        // Start forward
  showLED();        // Show LED indicator
  
  // If left bumper hit
  if (pbLeft == HIGH) {
   reverse();
   delay(500); 
   turnRight();
   delay(1500);
   pbLeft = LOW;
  }
  
  // If right bumper hit
  if (pbRight == HIGH) {
   reverse();
   delay(500); 
   turnLeft();
   delay(1500);
   pbRight = LOW;
  }
}

// Motion routines
void forward() {
  servoLeft.write(180);
  servoRight.write(0);
}

void reverse() {
  servoLeft.write(0);
  servoRight.write(180);
}

void turnRight() {
  servoLeft.write(180);
  servoRight.write(180);
}

void turnLeft() {
  servoLeft.write(0);
  servoRight.write(0);
}

void stopRobot() {
  servoLeft.write(90);
  servoRight.write(90);
}

void showLED() {
  if (pbRight == HIGH || pbLeft == HIGH) {     
    digitalWrite(ledPin, HIGH);
  } 
  else {
    digitalWrite(ledPin, LOW); 
  }   
}

// Interrupt handlers
void hitLeft() {
  pbLeft = HIGH;
}

void hitRight() {
  pbRight = HIGH;
}

