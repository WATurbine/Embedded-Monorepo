#include <Stepper.h>
#include <Servo.h>
#include <math.h>

const float clockPeriod = 500; // clock period in ms, used to make serial monitor more readable

const float r = 0.05; // radius in meters, dont know this yet
const float alpha = 0.12; // blade angle of attack in radians, 7 degrees not radians
const float radPerStep = 1/200; 

const int stepper1A = 9;
const int stepper2A = 10;
const int stepper3A = 11;
const int stepper4A = 12;
const int ledPin = 13;
const int motorControllerThermistor = A0;
#define stepsPerRevolution 200
Stepper myStepper(stepsPerRevolution, stepper1A, stepper2A, stepper3A, stepper4A);

// Thermistor parameters
const float R0 = 10000.0;      // resistance at 25°C
const float T0 = 25.0 + 273.15; // nominal temperature in Kelvin
const float BETA = 3375.0;     // beta coefficient
const float Rref = 10000.0;    // series resistor value

Servo myServo;

struct ThermistorReadings {
  float t1;
  float t2;
  float t3;
};

bool isLit = false;

void setup() {
  // initialize the serial port:
  Serial.begin(9600);

  myServo.attach(7);

  // set the servo
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  pinMode(ledPin, OUTPUT);
}

void loop() {
  // poll all the sensors
  ThermistorReadings temps = readThermistors();
  printTime(millis());
  Serial.print(" - 1: ");
  Serial.print(temps.t1);
  Serial.print(", 2: ");
  Serial.print(temps.t2);
  Serial.print(", 3: ");
  Serial.println(temps.t3);


  // set servo
  if (Serial.available()) {
    int percent = Serial.parseInt();   // read full incoming string
    if (percent >= 0 && percent <= 100) {
      int angle = map(percent, 0, 100, 0, 180);  // convert %
      myServo.write(angle);

      printTime(millis());
      Serial.print(" - Percent: ");
      Serial.print(percent);
      Serial.print("  -> Angle: ");
      Serial.println(angle);
    } else {
      Serial.println("Invalid. Enter 0–100.");
    }
  }

  isLit = !isLit;
  if (isLit){
    digitalWrite(13, HIGH);
  }else{
    digitalWrite(13, LOW);
  }
  delay(clockPeriod);
}

ThermistorReadings readThermistors(){  
  ThermistorReadings r;
  r.t1 = readThermistor(14);
  r.t2 = readThermistor(15);
  r.t3 = readThermistor(16);
  return r;  // safe
}

void printTime(unsigned long ms){
  unsigned long minutes = ms / 60000;
  unsigned long seconds = (ms % 60000) / 1000;
  unsigned long millisPart = ms % 1000;
  // Print with zero-padding
  Serial.print(minutes);
  Serial.print(":");
  if (seconds < 10) Serial.print("0");
  Serial.print(seconds);
  Serial.print(".");
  if (millisPart < 100) Serial.print("0");
  if (millisPart < 10)  Serial.print("0");
  Serial.print(millisPart);
}

float readThermistor(int pinToRead){
  int adc = analogRead(pinToRead);

  float Vratio = adc / 1023.0;   // or 4095.0 for 12-bit ADC chips

  // correct for your wiring
  float Rtherm = Rref * (1.0 / Vratio - 1.0);
  float steinhart;
    steinhart = Rtherm / 10000;
    steinhart = log(steinhart);
    steinhart /= BETA;
    steinhart += 1.0 / (25 + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

  return steinhart;
}
