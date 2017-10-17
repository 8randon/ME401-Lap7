int ledPin = 9;
int intNumber = 2;  // Interrupt number to be used. CHANGE ACCORDINGLY
volatile int state = LOW; //state variable to be changed during an interrupt

void setup() {
	pinMode(ledPin, OUTPUT); //configure the LED Pin
        digitalWrite(7, HIGH); //configure the interrupt pin for the switch operation. CHANGE THE PIN NUMBER ACCORDINGLY.
        //TODO: configure the external interrupt for Pin 38 and initialize the serial port
        attachInterrupt(intNumber,blink, FALLING);
        Serial.begin(9600);
}

void loop() {
    digitalWrite(ledPin, state);// light up the LED to display the current value of the state variable
    // TODO: Output the text "LOOP" through the serial port
    Serial.println("LOOP");
    delay(1000);
}

void blink() {
    state = !state; // negate the state variable when the interrupt happens
    // TODO: Output the text "INTERRUPT" through the serial port
    Serial.println("INTERRUPT");
}

