// Anemometer calibration values
const float anemometer_min_volts = 0.4;
const float anemometer_max_volts = 2.0;
const float min_wind_speed = 0.0;
const float max_wind_speed = 50;

// ADC input pin
const int adcPin = A0; //use A0 or whatever analog pin you are using

// Teensy 4.0 default analog reference is 3.3V (don't need to set analogReference)

void setup() {
  Serial.begin(9600); //standard baud rate
}

void loop() {
  int adcVal = analogRead(adcPin); // 10-bit ADC, returns 0-1023
  float voltage = adcVal / 1023.0 * 3.3;
  float wind_speed = mapFloat(voltage, anemometer_min_volts, anemometer_max_volts, min_wind_speed, max_wind_speed);

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Wind Speed: ");
  Serial.print(wind_speed);
  Serial.println(" m/s");

  delay(50);
}

// Custom map function for floats
float mapFloat(float x, float in_min ,float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

