// definition
#define dirPin 45
#define stepPin 42
#define stepsPerRevolution 200
#include "juice_detection.h"

int step_count = 0;

// function to extend arm
void ArmExtend(double target) {
  delay(100);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

 //Counterclockwise
  digitalWrite(dirPin, HIGH);

  for (int i = 0; i < target; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    step_count++;
  }

  delay(100);
}


//function to retract arm
void ArmRetract() {

  //delay(1000);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  //Clockwise
  digitalWrite(dirPin, LOW);
  int i = 0;
  while(!juice_detection() && i < 6*stepsPerRevolution)
  //while(i < 6*stepsPerRevolution)
  {
    i++;
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    step_count--;
  }
  // was 5.9
  //double remaining_rev = 4.9-(i/200);
  //return remaining_rev;


}

void fullRetract(double remaining_rev)
{
  delay(100);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

 //Clockwise
  digitalWrite(dirPin, LOW);

  for (int i = 0; i < remaining_rev; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    step_count--;
  }
  delay(100);
}