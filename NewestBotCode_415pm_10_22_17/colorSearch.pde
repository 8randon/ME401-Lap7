void colorSearch(){
  
  if( redCounter > blueCounter){
    colorState = RED;
  }
  else{
    colorState = BLUE;
  }


  while(!found){
    
    while(state != colorState){
      // turn on the red led only
     digitalWrite(rPin, HIGH); //Set rPin to HIGH++++++++++++++++
     digitalWrite(bPin, LOW); //Set bPin to LOW+++++++++++++++
  
      start_time = millis();
       while(millis()<start_time+50){
      }//wait
      
      sensorValue = analogRead(sensorPin); 
      color[0]=sensorValue;
      
      
      color[0]=constrain(sensorValue,minwhite[0],maxblack[0]);
      

      Serial.print("Red: ");Serial.print(color[0]);Serial.print(" "); 
    
    
      digitalWrite(rPin, LOW); 
      digitalWrite(bPin, HIGH);
    
      
      start_time = millis();
       while(millis()<start_time+50){
      }
      
      sensorValue = analogRead(sensorPin); 
      color[1]=sensorValue;
      color[1]=constrain(sensorValue,minwhite[1],maxblack[1]);
      
      Serial.print("Blue: ");Serial.print(color[1]);Serial.print(" ");  

      if(color[0] < 465 && color[1] < 470)
      {
        state = WHITE;
        fwd();
      }
      else if(color[1] > 710 && color[0] > 700)
      {
        state = BLACK;
        fwd();
      }
      else if(color[0]<color[1])
      {
        state = RED;
      }
      else if(color[1]<color[0])
      {
        state = BLUE;
      }
      
      //if the sensor picked up that the bot is on the right color, then break out of the current while loop
      if(state == colorState){
        break;
      }
      
      
      switch (state)
      {
        case (RED):
          myServoL.writeMicroseconds(1700);
          myServoR.writeMicroseconds(1700);
          
          while(millis()<start_time+750){
          }//wait

          fwd();
          
          start_time = millis();
          while(millis()<start_time+500){
          }//wait
          break;
          
        case (BLUE):
          myServoL.writeMicroseconds(1300);
          myServoR.writeMicroseconds(1300);
          
          while(millis()<start_time+750){
          }//wait
          
          fwd();
          
          start_time = millis();
          while(millis()<start_time+500){
          }//wait
          break;
          
        case (BLACK):
          break;
        case (WHITE):
          state = MAIN;
          break;
        default:
          Serial.println("ERROR - UNKNOWN STATE");
          break;
      }
    }
    
    //______________________________ Moves forward to move more onto correct color pad
    fwd();
    start_time = millis();
    while(millis()<start_time+500){
    }//wait
    
    //_______________________________________________________________Checks sensors again to see if it is still on the right color
    
     // turn on the red led only
     digitalWrite(rPin, HIGH); //Set rPin to HIGH++++++++++++++++
     digitalWrite(bPin, LOW); //Set bPin to LOW+++++++++++++++
  
      start_time = millis();
       while(millis()<start_time+50){
      }//wait
      
      sensorValue = analogRead(sensorPin); 
      color[0]=sensorValue;
      
      
      color[0]=constrain(sensorValue,minwhite[0],maxblack[0]);
      

      Serial.print("Red: ");Serial.print(color[0]);Serial.print(" "); 
    
    
      digitalWrite(rPin, LOW); 
      digitalWrite(bPin, HIGH);
    
      
      start_time = millis();
       while(millis()<start_time+50){
      }
      
      sensorValue = analogRead(sensorPin); 
      color[1]=sensorValue;
      color[1]=constrain(sensorValue,minwhite[1],maxblack[1]);
      
      Serial.print("Blue: ");Serial.print(color[1]);Serial.print(" ");  

      if(color[0] < 465 && color[1] < 470)
      {
        state = WHITE;
        fwd();
      }
      else if(color[1] > 710 && color[0] > 700)
      {
        state = BLACK;
        fwd();
      }
      else if(color[0]<color[1])
      {
        state = RED;
      }
      else if(color[1]<color[0])
      {
        state = BLUE;
      }
      
      // If the sensor picked up that the bot is on the right color, then set fount to true 
      if(state == colorState){
        found == true;
      }
      
      // If not, then it repeats this process
      // Maybe run through irDetect again ro re-orient bot
  }
  
  Serial.print("WE DONE");
}

