const int SensorPin = 2;

volatile unsigned long LastPulseTime = 0; // Time when last pulse is detected
volatile bool objectPresent = false; // Track if the object is currently detected
volatile bool readyForNextPulse = true; // The new pulse is only registered when the object leaves
volatile bool newValidPulse = false; // A new pulse has been recorded
unsigned long TimeBetweenPulses = 0;
float RPM = 0;

void setup() {
  Serial.begin(9600);
  pinMode(SensorPin, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(SensorPin), PulseDetected, CHANGE);  // Trigger interrupt on both rising and falling edges of the sensor signal
}

void loop() {
  unsigned long currentTime = micros();

  // Timeout: reset RPM if no object has passed in the last 5 seconds
  if (currentTime - LastPulseTime > 5000000) {
    RPM = 0;
  }

  // If a new valid pulse has been registered, compute RPM
  if (newValidPulse) {
    static unsigned long PreviousPulseTime = 0;

    if (PreviousPulseTime > 0) {
      TimeBetweenPulses = LastPulseTime - PreviousPulseTime;  // Time between current and last pulse
      RPM = 60000000.0 / (float)(TimeBetweenPulses);  // Convert microseconds to RPM
    }

    PreviousPulseTime = LastPulseTime; // Update last pulse time
    newValidPulse = false;
  }

  Serial.print("RPM: ");
  Serial.println(RPM);
  delay(1000);  // Display update every second
}

void PulseDetected() {
  unsigned long CurrentTime = micros();
  bool sensorState = digitalRead(SensorPin);

  // Debounce: ignore pulses faster than 1ms apart
  if (CurrentTime - LastPulseTime < 1000) return;

  if (sensorState == HIGH && readyForNextPulse) {
    // Object just arrived
    objectPresent = true; // Mark that object is detected
    readyForNextPulse = false; // Prevent duplicate detection
    LastPulseTime = CurrentTime;  // Save pulse timestamp
    newValidPulse = true;
  }

  if (sensorState == LOW && objectPresent) {
    // Object has left, ready for next pass
    objectPresent = false; // Object is no longer detected
    readyForNextPulse = true; // Ready to detect the next incoming pulse
  }
}