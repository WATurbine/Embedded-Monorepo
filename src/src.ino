#include <Stepper.h>

const int stepper1A = 9;
const int stepper2A = 10;
const int stepper3A = 11;
const int stepper4A = 12;
const int ledPin = 13;
const int motorControllerThermistor = A0;
#define stepsPerRevolution 200
Stepper myStepper(stepsPerRevolution, stepper1A, stepper2A, stepper3A, stepper4A);

//These  values are in the datasheet
#define RT0 10000   // Ω
#define B 3977      //  K
#define VCC 3.3    //Supply  voltage
#define R 10000  //R=10KΩ
float RT, VR, ln, TX,  T0, VRT;

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
  Serial2.begin(9600);

  pinMode(ledPin, OUTPUT);

  // TODO:: buy a relay/digital cmos for the solenoid
  // drive logic high to the cmos/relay
  // add an interrupt for this

  // move the stepper to the limitswitch, reset step count 
  unsigned int currentSteps = calibrateStepper();
}

void loop() {
  // poll all the sensors
  int arrayTherm[5];
  arrayTherm = readThermistors();

  unsigned int V_w = analogRead(anemometerPin);
  unsigned int omega = RPMcalculator();

  // trigger an interrupt if any sensors give average bad data


  // pitching blades -> TODO figure what radius to use

  // beta = arctan(V_w/(omega*r)) - alpha
  // Get the desired angle
  // turn into number of steps
  int desiredSteps = pitchMath(V_w,omega);
  int amountToStep = desiredSteps - currentSteps;
  // compensate for intertia of blades
  // .step(1) or .step(-1) or dont
  // iterate desired steps/current

  //applying brakes if necessary 
  // make a function that corresponds to over-rpm


}

int* readThermistors(){
  VRT = analogRead(A0);              //Acquisition analog value of VRT
  29  VRT  = (5.00 / 1023.00) * VRT;      //Conversion to voltage
  30  VR = VCC - VRT;
  31  RT = VRT / (VR / R);               //Resistance of RT
  32
  33  ln = log(RT / RT0);
  34  TX = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistor
  35
  36  TX =  TX - 273.15;                 //Conversion to Celsius
  37
  38  Serial.print("Temperature:");
  39  Serial.print("\	");
  40  Serial.print(TX);
  41  Serial.print("C\	\	");
}

void emergencyHandler(){
  // pitch the blades out of the wind -> max in one of the directions
  // apply servo max
  // release solenoid

}

void rpmHandler(){

}
