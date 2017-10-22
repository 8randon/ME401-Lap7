// INITIALIZE THE ANALOG INPUT PORTS
//INITIALIZE ANY VARIABLE THAT YOU USE FOR DISTANCE MEASUREMENT AND CALCULATIONS
int irSensor1Pin1 = A7;//long; close limit is 17cm; if close range senses below 17, listen to close sensor
int irSensor1Pin2 = A8;//short
float irSensor1Value1 = 0;
float irSensor1Value2 = 0;

float f1=0.0;
float f2=0.0;

void setup() {
Serial.begin(9600);

}

void loop() {
//READ FROM ONE OF THE SENSORS
irSensor1Value1 = analogRead(irSensor1Pin1);
f1 = -0.0612*irSensor1Value1 + 68.863;
delay(100); //wait for the measurement to settle

//IMPLEMENT READING TO DISTANCE CONVERSION FOR THE FIRST SENSOR
//READ FROM THE OTHER SENSOR
irSensor1Value2 = analogRead(irSensor1Pin2);
f2 = 4536.1*pow(irSensor1Value2, -1.048);
delay(100);//wait for the measurement to settle


Serial.print("Sensor1:");
Serial.print("        ");
Serial.print("Sensor2:");
Serial.println("");
Serial.print(f1);
Serial.print("        ");
Serial.print(f2);
Serial.println("");

//IMPLEMENT READING TO DISTANCE CONVERSION FOR THE FIRST SENSOR

//Serial.print("Sensor1:");
//Serial.print(distance1); //PRINT THE OUTPUT FOR THE FIRST SENSOR IN DISTANCE UNITS, CHANGE THE VARIABLE NAME ACCORDINGLY
//Serial.print(" Sensor2:");
//Serial.print(distance2); //PRINT THE OUTPUT FOR THE SECOND SENSOR, CHANGE THE VARIABLE NAME ACCORDINGLY
//Serial.println("");
}
