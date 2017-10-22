void colorLoop()
{
  // turn on the red led only
   digitalWrite(rPin, HIGH); //Set rPin to HIGH++++++++++++++++
   digitalWrite(bPin, LOW); //Set bPin to LOW+++++++++++++++
  
  start_time = millis();
   while(millis()<start_time+50){
  }//wait
  
  sensorValue = analogRead(sensorPin); // read the photoresistor value
  // record the red reading
  color[0]=sensorValue;
  
  //constrain the reading such that it is between the white and black values
  color[0]=constrain(sensorValue,minwhite[0],maxblack[0]);
  
  // map the reading between 0 and 100 such that black is 0, 100 is white
  //color[0]=map(color[0],maxblack[0],minwhite[0],0,100);
  
  // output the reading
  Serial.print("Red: ");Serial.print(color[0]);Serial.print(" "); 


  digitalWrite(rPin, LOW); //Set rPin to LOW+++++++++++++++ 
  digitalWrite(bPin, HIGH); //Set bPin to HIGH++++++++++++++++

  
  start_time = millis();
   while(millis()<start_time+50){
  }//wait
  
  sensorValue = analogRead(sensorPin); 
  color[1]=sensorValue;
  color[1]=constrain(sensorValue,minwhite[1],maxblack[1]);
  //color[1]=map(color[1],maxblack[1],minwhite[1],0,100);
  Serial.print("Blue: ");Serial.print(color[1]);Serial.print(" ");  
  //Serial.println("");
  
  
  
  //white color detected
  if(color[0] < 465 && color[1] < 470)
  {
    //Serial.println("WHITE!");
    state = WHITE;
    colorState = WHITE;
  }
  //black color detected
  else if(color[1] > 710 && color[0] > 700)
  {
    //Serial.println("BLACK!");
    state = BLACK;
    colorState = BLACK;
  }
   //Red color detected
  else if(color[0]<color[1])
  {
    //Serial.println("RED!");
    state = RED;
    colorState = RED;
  }
  //Blue color detected
  else if(color[1]<color[0])
  {
    //Serial.println("BLUE!");
    state = BLUE;
    colorState = BLUE;
  }

}
