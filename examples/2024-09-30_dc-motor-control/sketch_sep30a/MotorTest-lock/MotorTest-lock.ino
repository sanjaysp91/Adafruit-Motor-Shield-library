// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

// Initialize motor at M3 terminal
AF_DCMotor motor(3);

// Define the Hall sensor pins
const int hallSensorUnlocked = A10;  // Sensor for "unlocked" state
const int hallSensorLocked = A11;    // Sensor for "locked" state

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  Serial.println("Motor with Hall Sensor control!");

  // Initialize Hall sensor pins as input
  pinMode(hallSensorUnlocked, INPUT);
  pinMode(hallSensorLocked, INPUT);

  int unlockedState = 0;  // init for "unlocked"
  int lockedState = 0;      // init for "locked"

  // turn on motor
  motor.setSpeed(200);  // Set initial speed to 200 (adjustable as needed)
  motor.run(RELEASE);   // Ensure motor is initially stopped
}

void loop() {
  // uint8_t i;
  // Serial.print("tick");

  // motor.run(FORWARD);
  // motor.setSpeed(255);
  // delay(10*1000);
//   for (i=0; i<255; i++) {
//     motor.setSpeed(i);  
//     delay(100);
//  }
 
//   for (i=255; i!=0; i--) {
//     motor.setSpeed(i);  
//     delay(100);
//  }
  
  // Serial.print("tock");

  // motor.run(BACKWARD);
  // motor.setSpeed(255);
  // delay(10*1000);
//   for (i=0; i<255; i++) {
//     motor.setSpeed(i);  
//     delay(100);
//  }
 
//   for (i=255; i!=0; i--) {
//     motor.setSpeed(i);  
//     delay(100);
//  }
  

  // Serial.print("tech");
  // motor.run(RELEASE);
  // delay(1000);  // 1 second delay

  // Read the Hall sensor states
  if (digitalRead(hallSensorUnlocked) > 200) {
  unlockedState = 1;  // Read sensor for "unlocked"
  }
  if (digitalRead(hallSensorLocked) > 200) {
  lockedState = 1;  // Read sensor for "unlocked"
  }

  if (unlockedState == HIGH && lockedState == LOW) {
    // If the lock is unlocked, run the motor forward
    Serial.println("Lock is unlocked - Motor running forward...");
    motor.run(FORWARD);
    motor.setSpeed(255);  // Full speed forward

  } else if (lockedState == HIGH && unlockedState == LOW) {
    // If the lock is locked, run the motor backward
    Serial.println("Lock is locked - Motor running backward...");
    motor.run(BACKWARD);
    motor.setSpeed(255);  // Full speed backward

  } else {
    // If no clear lock/unlock signal, stop the motor
    Serial.println("No valid sensor input - Motor stopped.");
    motor.run(RELEASE);  // Stop the motor
  }

  // Small delay to avoid too frequent polling
  delay(100);
}

}
