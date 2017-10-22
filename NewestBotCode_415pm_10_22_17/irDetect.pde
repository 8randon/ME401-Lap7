void irDetect(){
//  Serial.println("\n\ntargetAngle:");
 // Serial.println("\n\nChecking Distance\n\n");
  newPosition = position;
  
  Kp = 17;
  Kd = 3.0;
  Ki = 0.5;

    //TAKING THE AVERAGE DISTANCE READING AT A POSITION
    
    for(p = -200; p<200 ; p+=5 ){ //takes multiple readings at each position
    
      targetAngle = p;
      
      start_time = millis();
      while (millis() < start_time + 100) {
      }//wait 0.1 seconds
      
      //READ FROM IR SENSOR
      
      //if(IRSENSOR == HIGH){
      //break;
      //}
      
    }

  //determines which way to turn based on the ongle at which the greatest value was found
  
  if(p<0){ // turn right
   myServoL.writeMicroseconds(1700);
   myServoR.writeMicroseconds(1700);

   start_time = millis();
   while(millis()<start_time+int(abs(p)*encoderToDeg*degToMs)){
    }
  }
  else if(p>0){// turn left
   myServoL.writeMicroseconds(1300);
   myServoR.writeMicroseconds(1300);

   start_time = millis();
   while(millis()<start_time+int(abs(p)*encoderToDeg*degToMs)){
    }
  } 
}


