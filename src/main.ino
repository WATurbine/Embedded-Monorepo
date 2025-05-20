

// #defines go up here
// ex: #define LED_BUILTIN 13
// #includes go up here
// ex: #include <Arduino.h>
void setup() {
    // put your setup code here, to run once

    Serial.begin(9600);
}

void loop() {
    // duty loop. we will repeat this code
    
    blink_led(1, 2);
}

// function definition with docstring 

void blink_led(int led1, int led2){
    /* 
        this function will blink led1 and led2, where led1 is the first led and led2 is the second led
    */
    return; 
}
