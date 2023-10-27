#include <Arduino.h>

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
  // (Forward)
  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, HIGH);
  digitalWrite(DIR_C, HIGH);  // Leaf motor
  // Motors run forward
  delay(2000);

  // (Backward)
  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, LOW);
  digitalWrite(DIR_C, HIGH);  // Leaf motor
  // Motors run backward
  delay(2000);
}

