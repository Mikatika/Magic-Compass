#include <Stepper.h>

// change this to the number of steps on your motor
#define STEPS 400

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 4, 5, 6, 7);


void setup()
{
  Serial.begin(9600);
  Serial.println("Stepper test!");
  // set the speed of the motor to 30 RPMs
  stepper.setSpeed(20);
}

void loop()
{
  Serial.println("Forward");
  stepper.step(10);
  Serial.println("Backward");
  stepper.step(-10);
}