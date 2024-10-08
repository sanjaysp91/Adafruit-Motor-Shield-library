// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

// Initialize motor at M3 terminal
AF_DCMotor motor(3);

// Define the Hall sensor pins
const int hallSensorUnlocked = A10;  // Sensor for "unlocked" state
const int hallSensorLocked = A11;    // Sensor for "locked" state

// Declare unlocked and locked states as global variables
int unlockedState = 0;  // State for "unlocked"
int lockedState = 0;    // State for "locked"

// Variables for filtering analog values
float filteredUnlocked = 0;
float filteredLocked = 0;
const float alpha = 0.1;  // Low-pass filter coefficient

// Define a threshold to decide between HIGH and LOW
const int threshold = 375;  // Midpoint of 1023 (adjust as necessary)
const int debug = 0; 

// Define time keepers
unsigned long startTime = 0;
unsigned long endTime = 0;

// Function prototypes
void handleUnlocking();
void handleLocking();

int tic();
int toc();
void tic_toc_print(); 

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor with Hall Sensor control!");

  // Initialize Hall sensor pins as input
  pinMode(hallSensorUnlocked, INPUT);
  pinMode(hallSensorLocked, INPUT);

  // Turn on motor
  motor.setSpeed(200);  // Set initial speed to 200 (adjustable as needed)
  motor.run(RELEASE);   // Ensure motor is initially stopped
}

void loop() {
  startTime = tic();  // Start execution timer

  // Read the raw analog values from the Hall sensors
  int rawUnlocked = analogRead(hallSensorUnlocked);
  int rawLocked = analogRead(hallSensorLocked);

  // Apply a low-pass filter (moving average)
  filteredUnlocked = alpha * rawUnlocked + (1 - alpha) * filteredUnlocked;
  filteredLocked = alpha * rawLocked + (1 - alpha) * filteredLocked;

  if (debug) {
      // Debug output to monitor raw and filtered values
      Serial.print("Raw Unlocked: ");   Serial.print(rawUnlocked);
      Serial.print("\tFiltered Unlocked: ");  Serial.println(filteredUnlocked);

      Serial.print("Raw Locked: ");   Serial.print(rawLocked);
      Serial.print("\tFiltered Locked: ");  Serial.println(filteredLocked);
  }

  // Determine the states based on the filtered values
  bool unlockedState = filteredUnlocked > threshold;
  bool lockedState = filteredLocked > threshold;

  // print the states: 
  Serial.print("unlockedState = ");   Serial.print(unlockedState);
  Serial.print("\t lockedState = ");  Serial.println(lockedState);

  if (unlockedState == HIGH && lockedState == LOW) {
    handleLocking();

  } else if (lockedState == HIGH && unlockedState == LOW) {
    handleUnlocking();

  } else {
    // If no clear lock/unlock signal, stop the motor
    Serial.println("No valid sensor input - Motor stopped.");
    motor.run(RELEASE);  // Stop the motor
  }

  // Small delay to avoid too frequent polling
  delay(1000);
  endTime = toc();  // Stop execution timer
  tic_toc_print(startTime, endTime);  // Print execution time
}

// Function to handle unlocking (motor runs forward)
void handleLocking() {
  // If the lock is unlocked, run the motor forward
  if (debug) Serial.println("Lock is unlocked - Motor running forward...");
  motor.run(FORWARD);
  motor.setSpeed(100);  // Full speed forward

}

// Function to handle locking (motor runs backward)
void handleUnlocking() {
  // If the lock is locked, run the motor backward
  if (debug) Serial.println("Lock is locked - Motor running backward...");
  motor.run(BACKWARD);
  motor.setSpeed(255);  // Full speed backward
}

// Function to start execution timer 
int tic() {
  // Start time measurement
  unsigned long startTime = millis();
  if (debug) Serial.print("Start time (ms): ");
  if (debug) Serial.println(startTime);
  return startTime;
}

// Function to stop execution timer 
int toc() {
    // End time measurement
  unsigned long endTime = millis();
  if (debug) {Serial.print("End time (ms): ");  Serial.println(endTime);}
  return endTime;
}

// Function to print execution time 
void tic_toc_print(unsigned long startTime, unsigned long endTime) {
  // Calculate and print execution time
  unsigned long executionTime = endTime - startTime;
  Serial.print("Execution time (ms): ");  Serial.println(executionTime); 
}