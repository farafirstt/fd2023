/*
Note:
the drive motor direction will be inverse, because we connect anode and cathod inversly so.
*/

#include "CytronMotorDriver.h"
#include <Arduino.h>
#include <string.h>
// using std::string;

String inputString = "";
bool stringComplete = false;

// Drive motor
CytronMD motor1(PWM_DIR, 11, 12);  // PWM 1 = Pin 11, DIR 1 = Pin 12.
CytronMD motor2(PWM_DIR, 5, 4);    // PWM 2 = Pin 5, DIR 2 = Pin 4.
//Leave collectpor motor
CytronMD motor3(PWM_DIR, 9, 8);  // PWM 1 = Pin 9, DIR 1 = Pin 8.

//variable for timer
const unsigned long interval = 2000;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  // #include <string>
  inputString.reserve(20);
}

void loop() {
   while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      stringComplete = true;
    } else {
      inputString += c;
    }
  }

  // If a complete string has been received
  if (stringComplete) {
    // Split the string into two parts using the underscore
    int value1, value2;
    // serial.print("Received: %d_%d", &value1, &value2);
    sscanf(inputString.c_str(), "%d_%d", &value1, &value2);

    motor1.setSpeed(value1);
    motor2.setSpeed(value2);
    motor3.setSpeed(220);

    // Clear the inputString and reset the flag
    inputString = "";
    stringComplete = false; 
  } 
  
}