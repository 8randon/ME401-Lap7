
int counter = 0;
int time = 0;
void setup() {
  
  attachCoreTimerService(MyCallback);
  Serial.begin(9600);
}

void loop() {
   Serial.print("\nCounter: ");
 Serial.print(counter);
 delay(1);
 //Serial.print(time++);
}

uint32_t MyCallback(uint32_t currentTime) {
 // TODO: Add code here to increment your countercouner
 counter++;

 // Tell the timer interrupt how far in the future this should
 //be called again.
 return (currentTime + CORE_TICK_RATE/1000);
 
} 
