

// 38 kHz is the carrier frequency
// pin 1 out, pin 2 gnd, pin 3 Vs

#define DETECT 1
#define NOSIGNAL -1

float frequency = 0;
const int IRSensorInputPin = 7;


 
void setup() 
{
  // Initialize the sensor on pin 8 as an input
  pinMode(IRSensorInputPin,INPUT);
  
  // Initialize the timer interrupt that decodes the IR beacon signal
  attachCoreTimerService(Decoder);

  Serial.begin(9600);
} 
 
void loop() 
{ 
  Serial.print("Val: ");
  int state = readIRFrequency();
  if (state== NOSIGNAL){
   Serial.println("NO SIGNAL DETECTED");
  }
  else if (state == DETECT){
   Serial.println("BEACON DETECTED!");
  }
  // Wait just a tad so we don't print too fast.  
  delay(100);
}



// The readFrequency function returns a value indicating whether it is detecting beacon signal or not

int readIRFrequency ()
{
  if (frequency >= 75 && frequency < 135)
    return DETECT;
  else
    return NOSIGNAL;
  
}



// This timer interrupt will determine if you are pointing at the beacon.
// The beacon is sending a 100Hz  signal riding on a 38KHz carrier frequency
// The sensor itself demodulates the 38KHz carrier frequency, so the digital pin should be
// receiving just the  100 Hz signal.
// This timer interrupt counts the number of pulses over an amount of time specified by windowTime
// at and interval time specified by sampleTime, and computes the current frequency.
int windowTime = 100;   // ms
int sampleTime= 1;     // ms
int windowIters = windowTime/sampleTime;

uint32_t Decoder(uint32_t currentTime) {

  static int lastVal = digitalRead(8);
  static int iters = 0;
  static int counter = 0;

  if (iters < windowIters)
  {
    int newVal = digitalRead(8);
    if (newVal==HIGH && lastVal == LOW)
    {
      counter++;
    }
    lastVal = newVal;
  }
  else
  {
    frequency = 1000.0*(float)counter/(float)windowTime;

    counter = 0;
    int newVal = digitalRead(8);    
    lastVal = newVal;
    iters = 0;
  }

  iters++;
  return (currentTime + CORE_TICK_RATE*sampleTime);
}

