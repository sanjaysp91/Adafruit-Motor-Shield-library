#include <AFMotor.h>

// Initialize motor at M3 terminal (index 3)
AF_DCMotor motor(3, 0);

void setup() 
{
  // Set initial motor speed
  motor.setSpeed(100);   
  // Ensure motor is initially stopped
  motor.run(RELEASE);    
}

void loop() 
{
  // Test motor in forward direction
  motorTest(FORWARD);

  // Delay between direction changes
  delay(1000);

  // Test motor in backward direction
  motorTest(BACKWARD);

  // Delay between cycles
  delay(1000);
}

// Function to ramp the motor speed up and down
void motorTest(uint8_t direction) 
{
  // Set motor direction
  motor.run(direction);
  
  // Speed up the motor
  for (uint8_t speed = 0; speed <= 255; speed++) 
  {
    motor.setSpeed(speed);
    delay(10);
  }

  for (uint8_t count = 0; count <= 255; count++) 
  {
    // motor.setSpeed(speed);
    delay(1000);
  }
  // Speed down the motor
  for (uint8_t speed = 255; speed > 0; speed--) 
  {
    motor.setSpeed(speed);
    delay(10);
  }
  
  // Stop the motor
  motor.run(RELEASE);
}
