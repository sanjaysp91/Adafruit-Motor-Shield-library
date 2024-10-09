// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

// Initialize motor at M3 terminal
AF_DCMotor motor(3);

// Define the Hall sensor pins
const int hallSensorUnlocked = A10;  // Sensor for "unlocked" state
const int hallSensorLocked = A11;    // Sensor for "locked" state

// Define lock states 
enum LockState {
  INIT,
  LOCKED,
  UNLOCKED,
  INVALID
}; 

// Define motor states
enum MotorState {
  MOTOR_IDLE, 
  MOTOR_STOPPED, 
  MOTOR_RAMPING_UP, 
  MOTOR_RUNNING, 
  MOTOR_RAMPING_DOWN
}; 

// previous states 
LockState previousLockState = INIT; 
MotorState previousMotorState = MOTOR_IDLE; 

// present states 
LockState presentLockState = INIT; 
MotorState presentMotorState = MOTOR_IDLE; 

// Variables for ramping 
int targetSpeed = 255;              // Target speed
int presentSpeed = 0;               // Current speed
const int rampStep = 25;            // Speed increment/decrement step
unsigned long lastRampTime = 0;     // Last time speed was changed
const unsigned long rampInterval = 50;  // Interval between speed changes (ms)

// Function prototypes 
LockState updateLockState();
void rampUp();
void rampDown();

// Optional: Function to print speed and execution time
void tic_toc_print(unsigned long startTime, unsigned long endTime);

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
const int PARAM_START_DUTY = 100;
const int PARAM_MAX_DUTY = 100;
int speed = PARAM_START_DUTY; 
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
  motor.setSpeed(0);  // start with motor stopped 
  motor.run(RELEASE);   // Ensure motor is initially stopped
}

void loop() {
  startTime = tic();  // Start execution timer

  // Update the present lock state based on sensor input 
  presentLockState = updateLockState();

  // State machine to handle motor behavior based on lock state 
  switch (presentMotorState) {
    case MOTOR_IDLE:
      if (presentLockState == INIT || INVALID) {
        // do nothing
        motor.run(RELEASE);  // Stop the motor
      }
    case MOTOR_STOPPED:
      if (presentLockState == UNLOCKED) {
        Serial.println("State: Unlocked - Initiating Ramp Up Forward"); 
        presentMotorState = MOTOR_RAMPING_UP; 
        motor.run(FORWARD);
        targetSpeed = 255;
        presentSpeed = 0; 
        lastRampTime = millis(); 
      }
      else if (presentLockState == LOCKED) {
        Serial.println("State: Locked - Initiating Ramp Up Backward");
        presentMotorState = MOTOR_RAMPING_UP;
        motor.run(BACKWARD);
        targetSpeed = 255;
        presentSpeed = 0;
        lastRampTime = millis(); 
      }
      break;

    case MOTOR_RAMPING_UP:
      rampUp();
      presentMotorState = MOTOR_RUNNING;
      break;

    case MOTOR_RUNNING:
      // Maintain full speed or implement any running behavior here
      // For simplicity, we'll just wait for state changes
      break; 

    case MOTOR_RAMPING_DOWN:
      rampDown();
      break;   
  }

  // Small delay e.g. 10ms to prevent overwhelming the serial monitor
  delay(1000);

}  

//   // Small delay to avoid too frequent polling
//   delay(1000);
//   endTime = toc();  // Stop execution timer
//   tic_toc_print(startTime, endTime);  // Print execution time
// }

// Function to update the current lock state based on sensor inputs
LockState updateLockState() {
  // Read the raw analog values from the Hall sensors
  int rawUnlocked = analogRead(hallSensorUnlocked);
  int rawLocked = analogRead(hallSensorLocked);

  // Apply a low-pass filter (moving average)
  filteredUnlocked = alpha * rawUnlocked + (1 - alpha) * filteredUnlocked;
  filteredLocked = alpha * rawLocked + (1 - alpha) * filteredLocked;

  if (debug) {
      // Debug output to monitor raw and filtered values
      Serial.print("Raw Unlocked: ");           Serial.print(rawUnlocked);
      Serial.print("\tFiltered Unlocked: ");    Serial.println(filteredUnlocked);

      Serial.print("Raw Locked: ");             Serial.print(rawLocked);
      Serial.print("\tFiltered Locked: ");      Serial.println(filteredLocked);
  }

  // Determine the states based on the filtered values
  bool unlockedState = filteredUnlocked > threshold;
  bool lockedState = filteredLocked > threshold;

  // print the states: 
  if (debug) {
    Serial.print("unlockedState = ");             Serial.print(unlockedState);
    Serial.print("\t lockedState = ");            Serial.println(lockedState);
  }

  // if (unlockedState == HIGH && lockedState == LOW) {
  //   handleLocking();

  // } else if (lockedState == HIGH && unlockedState == LOW) {
  //   handleUnlocking();

  // } else {
  //   // If no clear lock/unlock signal, stop the motor
  //   Serial.println("No valid sensor input");
  // }

  bool isUnlocked = unlockedState;
  bool isLocked = lockedState; 

  previousLockState = presentLockState; 
  // Determine the lock state 
  if (isUnlocked && !isLocked) {
    return UNLOCKED;
  }
  else if (isLocked && !isUnlocked) {
    return LOCKED;
  }
  else {
    // Handle conflict 
    Serial.println("No valid sensor input");
    return presentLockState;
  }
  if (presentLockState != previousLockState) {
    Serial.print("Lock State changed");
  }
}

// Function to ramp up the motor speed
void rampUp() {
  // Write a ramp up function 
  // For now keep it simple
  motor.setSpeed(150);
  presentSpeed = 149;   // Update from virtual sensor 
  Serial.print("Ramping Up - Present Speed: ");   Serial.println(presentSpeed); 
}

// Function to ramp down the motor speed
void rampDown() {
  // Write a ramp down function 
  // For now keep it simple
  motor.setSpeed(0);
  presentSpeed = 0;   // Update from virtual sensor 
  Serial.print("Ramping Up - Present Speed: ");   Serial.println(presentSpeed); 
}

// Function to handle unlocking (motor runs forward)
void handleLocking() {
  // If the lock is unlocked, run the motor forward
  if (debug) Serial.println("Lock is unlocked - Motor running forward...");
  motor.run(FORWARD);
  speed = speed + 10; 
  motor.setSpeed(speed);  // Full speed forward
  Serial.print("Speed: ");
  Serial.println(speed);
}

// Function to handle locking (motor runs backward)
void handleUnlocking() {
  // If the lock is locked, run the motor backward
  if (debug) Serial.println("Lock is locked - Motor running backward...");
  motor.run(BACKWARD);
  motor.setSpeed(PARAM_MAX_DUTY);  // Full speed backward
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
  Serial.print("Execution time (ms): ");        Serial.println(executionTime); 
}