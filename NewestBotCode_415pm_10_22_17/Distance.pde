void distance(){
//  Serial.println("\n\ntargetAngle:");
 // Serial.println("\n\nChecking Distance\n\n");
  newPosition = position;
  
  Kp = 17;
  Kd = 3.0;
  Ki = 0.5;
  
  
  for(i = 0; i<5;i++){
    
    // TODO:
    //write lookA[i] to motor
    targetAngle = lookA[i];
    start_time = millis();
    while (millis() < start_time + 500) {
  }//wait 1 second
//    Serial.println("targetAngle:");
//    Serial.println(targetAngle);
    
    //resetting sensor readings
    f1 = 0;
    f2 = 0;
    
    
    //TAKING THE AVERAGE DISTANCE READING AT A POSITION
    
    for(p = 0; p<numReadingsPerLook; p++){ //takes multiple readings at each position
      
      //READ FROM f1 (long distance)
      start_time = millis();
      while (millis() < start_time + 50) {
      }//wait 1 second
      //delay(50); //wait for the measurement to settle
      irSensor1Value1 = analogRead(irSensor1Pin1);
      f1 += -0.0612*irSensor1Value1 + 68.863;
      
      //READ FROM f2 (short distance)
      //delay(50);//wait for the measurement to settle
      irSensor1Value2 = analogRead(irSensor1Pin2);
      f2 += 4536.1*pow(irSensor1Value2, -1.048);
      
    }
    
    //takes average of readings to account for servo swivel
    f1 /= numReadingsPerLook;
    f2 /= numReadingsPerLook;
    
//          //READ FROM f1 (long distance)
//      delay(50); //wait for the measurement to settle
//      irSensor1Value1 = analogRead(irSensor1Pin1);
//      f1 += -0.0612*irSensor1Value1 + 68.863;
//      
//      //READ FROM f2 (short distance)
//      //delay(50);//wait for the measurement to settle
//      irSensor1Value2 = analogRead(irSensor1Pin2);
//      f2 += 4536.1*pow(irSensor1Value2, -1.048);
//    
    
    
    //decide which sensor to read from; (short distance sensor will never read below 17cm when out of accuracy range)
    if(f2 < 17){
      lookD[i] = f2;
//      Serial.println("Printing from Short Range:  ");
//      Serial.println(lookD[i]);
    }
    else{
      lookD[i] = f1;
//      Serial.println("Printing from Long Range:  ");
//      Serial.println(lookD[i]);
    }
    
  }
  
  // INITIALIZES THE LARGEST DISTANCE FOR THE LOOP
  largestDistance = lookD[0];
//  Serial.println("largestDistance:");

  
  if(largestDistance < lookD[1]){
    largestDistance = lookD[1];
//    Serial.println("1largestDistance:");

  }
  
  if(largestDistance < lookD[2]){
    largestDistance = lookD[2];
//    Serial.println("2largestDistance:");

  }
  
    if(largestDistance < lookD[3]){
    largestDistance = lookD[3];
//    Serial.println("3largestDistance:");

  }
  
  if(largestDistance < lookD[4]){
    largestDistance = lookD[4];
//    Serial.println("4largestDistance:");

  }
  //loops through whole distance value array to find the largest value
//  for(i = 0; i<2 ; i++){
//    if(lookD[i] > lookD[i+1]){
//    largestDistance = lookD[i+1];
//      Serial.println("largestDistance:");
//      Serial.println(largestDistance);
//    }
//  }


  
  
  // finds out where that value is in the array
  for(i = 0; lookD[i] != largestDistance; i++){
    //looking for largestDistance value in look array
  }
  
  // uses that index to find the corresponding angle at which that value was read
  botAngle = lookA[i];
  
//  Serial.println("botAngle:");
//  Serial.println(botAngle);
  
  start_time = millis();
    while (millis() < start_time + 100) {
  }//wait 1 second
  
  //determines which way to turn based on the ongle at which the greatest value was found
  
  if(botAngle<0){ // turn right
   myServoL.writeMicroseconds(1700);
   myServoR.writeMicroseconds(1700);
//   Serial.println("TURN RIGHT");
//   Serial.println("Delay: ");
//   Serial.println(int(abs(botAngle)*encoderToDeg*degToMs));
//   Serial.println("bot angle: ");
//   Serial.println(botAngle);
//   Serial.println("encoder to deg: ");
//   Serial.println(encoderToDeg);
//   Serial.println("deg to mss: ");
//   Serial.println(degToMs);
   //delay(int(abs(botAngle)*encoderToDeg*degToMs));
   start_time = millis();
   while(millis()<start_time+int(abs(botAngle)*encoderToDeg*degToMs)){
    }
  }
  else if(botAngle>0){// turn left
   myServoL.writeMicroseconds(1300);
   myServoR.writeMicroseconds(1300);
//   Serial.println("TURN LEFT");
//   Serial.println("Delay: ");
//   Serial.println((botAngle)*encoderToDeg*degToMs);
//   Serial.println("bot angle: ");
//   Serial.println(botAngle);
//   Serial.println("encoder to deg: ");
//   Serial.println(encoderToDeg);
//   Serial.println("deg to mss: ");
//   Serial.println(degToMs);
   //delay(int(botAngle*encoderToDeg*degToMs+200));
   start_time = millis();
   while(millis()<start_time+int(abs(botAngle)*encoderToDeg*degToMs)){
    }
  } 
}

