#include <Arduino.h>
#include <string>
using std::string;

string inputString = "";
bool stringComplete = false;

// Pin for first Cytron
const int PWM_A = 3;
const int PWM_B = 5;
const int DIR_A = 6;
const int DIR_B = 9;

// Pin for second Cytron
const int PWM_C = 10;
const int PWM_D = 11;
const int DIR_C = 12;
const int DIR_D = 13;

// Set the PWM frequency and duty cycle
const int PWM_FREQUENCY = 1000;  // Adjust as needed
const int PWM_DUTY_CYCLE = 128;  // Adjust as needed (0-255)

void setup() {
  Serial.begin(9600);
  inputString.reserve(20);
  // Initialize the PWM pins for the motors
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(PWM_C, OUTPUT);
  pinMode(PWM_D, OUTPUT);
  
  // Initialize the direction pins for the motors
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(DIR_C, OUTPUT);
  pinMode(DIR_D, OUTPUT);

  // Start the PWM for the motors
  analogWrite(PWM_A, PWM_DUTY_CYCLE);
  analogWrite(PWM_B, PWM_DUTY_CYCLE);
  analogWrite(PWM_C, PWM_DUTY_CYCLE);
  analogWrite(PWM_D, PWM_DUTY_CYCLE);
}

void loop() {
  // Read the serial input
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

    digitalWrite(DIR_A, value1 > 0 ? HIGH : LOW);
    digitalWrite(DIR_B, value2 > 0 ? HIGH : LOW);
    
    // Clear the inputString and reset the flag
    inputString = "";
    stringComplete = false;
  }
}
