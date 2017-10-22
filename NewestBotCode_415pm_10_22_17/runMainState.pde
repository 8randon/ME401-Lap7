void runMainState() //need to modify; implementatio has changed++++++++++++++
{
//  Serial.println("\nMAIN STATE: ");
//  Serial.print(state);

  //code that makes bot go straight
    fwd();
    
  //READ FROM f1 (long distance)
//      start_time = millis();
//      while(millis()<start_time+50){
//      } //wait for the measurement to settle
      
      irSensor1Value1 = analogRead(irSensor1Pin1);
      f1 = -0.0612*irSensor1Value1 + 68.863;
      
      //READ FROM f2 (short distance)
      //delay(50);//wait for the measurement to settle
      irSensor1Value2 = analogRead(irSensor1Pin2);
      f2 = 4536.1*pow(irSensor1Value2, -1.048);
      
//      Serial.print("Sensor1:");
//      Serial.print("        ");
//      Serial.print("Sensor2:");
//      Serial.println("");
//      Serial.print(f1);
//      Serial.print("        ");
//      Serial.print(f2);
//      Serial.println("");
      
    if(f2 < 17){
      botDist = f2;
//      Serial.println("Printing from Short Range");
    }
    else{
      botDist = f1;
//       Serial.println("Printing from Long Range");
    }
    
//     Serial.print("        ");
//      Serial.println("Bot Distance");
//      Serial.println(botDist);
    
    if(botDist<17){
      stopServos();
      distance();
      stopServos();
      sensorZero();
//      start_time = millis();
//      while(millis()<start_time+50){
//      }
    }
      else{
      fwd();
    }
//     Serial.println("Ran Main");
    
}
