/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */
#if !defined(PIN_LED1)
  #define PIN_LED1 13
#endif

int led = 2; //variable for pin
int d_time = 1500; //blink delay time

void setup() {
  // initialize the digital pin as an output.
  // Pin PIN_LED1 has an LED connected on most Arduino and compatible boards:
  pinMode(led, OUTPUT);
}

void loop() {
  digitalWrite(led, HIGH);   // set the LED on
  delay(d_time);              // wait for a second
  digitalWrite(led, LOW);    // set the LED off
  delay(d_time);              // wait for a second
}
