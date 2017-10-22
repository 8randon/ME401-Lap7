int sensorPin = 0;    // select the analog input pin for the photoresistor
int bPin = 12;  // select the digital output pin for blue LED

int rPin = 13; // select the digital output pin for red LED
int sensorValue = 0;  // variable to store the value coming from the photoresistor
int maxblack[]={282,263}; // the max reading that will be obtained from a black surface {RED,GREEN,BLUE}
int minwhite[]={113,105}; // the min reading that will be obtained from a white surface {RED,GREEN,BLUE}
int color[]={0,0}; // array for the readings
void setup() {
  // declare the LED pins as an OUTPUT:
  pinMode(rPin, OUTPUT);

  pinMode(bPin, OUTPUT);
  Serial.begin(9600) ; //initialize the serial communication
}

void loop() {
  // turn on the red led only
  
   digitalWrite(rPin, LOW); //Set rPin to HIGH++++++++++++++++
   digitalWrite(bPin, HIGH); //Set bPin to LOW+++++++++++++++
  //WRITE A SET OF COMMANDS TO LIGHT UP THE RED LED AND KEEP THE OTHERS OFF.
  delay(1000); //wait for the photresistor value to settle
  sensorValue = analogRead(sensorPin); // read the photoresistor value
//  color[0]=sensorValue; // record the red reading
//  color[0]=constrain(sensorValue,minwhite[0],maxblack[0]); //constrain the reading such that it is between the white and black values
//  color[0]=map(color[0],maxblack[0],minwhite[0],0,100); // map the reading between 0 and 100 such that black is 0, 100 is white
//  Serial.print("Red: ");Serial.print(color[0]);Serial.print(" "); // output the reading
Serial.println("RED");
  Serial.println(sensorValue);
  // repeat the above procedure for blue
  //WRITE A SET OF COMMANDS TO LIGHT UP THE BLUE LED AND KEEP THE OTHERS OFF.
  
  digitalWrite(bPin, LOW); //Set bPin to HIGH++++++++++++++++
  digitalWrite(rPin, HIGH); //Set rPin to LOW+++++++++++++++
  
  delay(1000);
  sensorValue = analogRead(sensorPin);
 Serial.println("BLUE"); 
  Serial.println(sensorValue);
//  color[2]=sensorValue;
//  color[2]=constrain(sensorValue,minwhite[2],maxblack[2]);
//  color[2]=map(color[2],maxblack[2],minwhite[2],0,100);
//  Serial.print("Blue: ");Serial.print(color[2]);Serial.print(" ");  
  Serial.println("");
}
