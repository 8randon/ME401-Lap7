// INITIALIZE THE ANALOG INPUT PORTS
//INITIALIZE ANY VARIABLE THAT YOU USE FOR DISTANCE MEASUREMENT AND CALCULATIONS
int irSensor1Pin1 = A1;
int irSensor1Pin2 = A2;
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
f1 = -0.0771*irSensor1Value1 + 77.16;

delay(1000); //wait for the measurement to settle
//IMPLEMENT READING TO DISTANCE CONVERSION FOR THE FIRST SENSOR
//READ FROM THE OTHER SENSOR
irSensor1Value2 = analogRead(irSensor1Pin2);

f2 = 2527.5*pow(irSensor1Value2, -0.958);
Serial.print("Sensor1:");
Serial.print("        ");
Serial.print("Sensor2:");
Serial.println("");
Serial.print(f1);
Serial.print("        ");
Serial.print(f2);
Serial.println("");
delay(1000);//wait for the measurement to settle
//IMPLEMENT READING TO DISTANCE CONVERSION FOR THE FIRST SENSOR

//Serial.print("Sensor1:");
//Serial.print(distance1); //PRINT THE OUTPUT FOR THE FIRST SENSOR IN DISTANCE UNITS, CHANGE THE VARIABLE NAME ACCORDINGLY
//Serial.print(" Sensor2:");
//Serial.print(distance2); //PRINT THE OUTPUT FOR THE SECOND SENSOR, CHANGE THE VARIABLE NAME ACCORDINGLY
//Serial.println("");
}
