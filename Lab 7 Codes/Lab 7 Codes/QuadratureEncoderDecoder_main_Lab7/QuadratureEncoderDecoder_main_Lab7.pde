#include <SoftPWMServo.h>
//#include <PID_v1.h>

// Global variables for keeping track of position
static volatile char lastLeftA;
static volatile char lastLeftB;
static volatile bool errorLeft;
volatile long position = 0;


void setup() {
    
  // TODO: Set up the serial port
  
  // Set up the quadrature inputs
  pinMode(2, INPUT);
  pinMode(20, INPUT);
  
  errorLeft = false;
  lastLeftA = digitalRead(2);
  lastLeftB = digitalRead(20);
  
  attachCoreTimerService(MyCallback);
    
}



void loop() {
  
  // TODO: Convert the position into an angle        

  // TODO: Print the angular position over the serial port
  
  delay(200);
        
}

/* For the core timer callback, just toggle the output high and low
   and schedule us for another 100uS in the future. CORE_TICK_RATE
   is the number of core timer counts in 1 millisecond. So if we 
   want this callback to be called every 10uS, we just divide 
   the CORE_TICK_RATE by 100, and add it to the current time.
   currentTime is the core timer clock time at the moment we get
   called
*/
uint32_t MyCallback(uint32_t currentTime) {
  char newLeftA = digitalRead(2);
  char newLeftB = digitalRead(20);
  
  position += (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB);
  
  if((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB))
    {
        errorLeft = true;
    }
  
  lastLeftA = newLeftA;
  lastLeftB = newLeftB;
      
  return (currentTime + CORE_TICK_RATE/100);
}






