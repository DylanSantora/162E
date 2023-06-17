#ifndef CLAW_INCLUDED
#define CLAW_INCLUDED

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

Servo myservo;

int pos = 0;          // variable to store servo position
int posRetrieved = 0; // variable to store servo position once retrieved

//** TESTING PURPOSES **
//
//Serial.begin(115200);
//while (!Serial) {
//  // will pause Zero, Leonardo, etc until serial console opens
//  delay(1);
//}
//
//Serial.println("Hello!");
//
//// Initialize the INA219.
//// By default the initialization will use the largest range (32V, 2A).  However
//// you can call a setCalibration function to change this range (see comments).
//  

//
//// To use a slightly lower 32V, 1A range (higher precision on amps):
////ina219.setCalibration_32V_1A();
//// Or to use a lower 16V, 400mA range (higher precision on volts and amps):
////ina219.setCalibration_16V_400mA();
//
//Serial.println("Measuring voltage and current with INA219 ...");

void openClaw(Servo myservo)
{
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  Serial.println("open claw");
  // open claw for retrieval (in increments of 1 degree)
  // note: claw is closed at 180, fully open at 0
  
  for (pos = 180; pos > 0; pos -= 1) {
    //Serial.print("in loop");
    myservo.write(pos); // tell servo to go to position in variable 'pos'
    delay(1);          // waits 25ms for the servo to reach the position
  }
}

void simple_openClaw(Servo myservo) {
  Serial.println("simple_openClaw()");
  for (pos = 180; pos > 0; pos -= 1) {
    //Serial.print("in loop");
    myservo.write(pos); // tell servo to go to position in variable 'pos'
    delay(1);          // waits 25ms for the servo to reach the position
  }
}

bool retrieve(Servo myservo)
{
  float current_mA = 0;     // variable to store amount of current drawn
  bool  retrieved  = false;

  // move claw inwards (in increments of 5 degrees) until juice box retrieved
  // note: claw is fully open at 0, closed at 180
  
  while (pos <= 150 && !retrieved) {
    myservo.write(pos);

    current_mA = ina219.getCurrent_mA();
    
    //    ** TESTING PURPOSES **
    Serial.print("Position:      "); Serial.print(pos); Serial.println(" deg");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.println();

    if (pos >= 115 && current_mA >= 300) {
      posRetrieved = pos;
      retrieved = true;
    }
    else if (pos == 150 && !retrieved) {
      openClaw(myservo);
    }
    else {
      pos += 5;
      delay(100);
      //delay(750);
    }
  }
  
  return retrieved;
}

void simple_retrieve(Servo myservo) {
  Serial.println("simple_retrieve()");
  while (pos <= 110) {
    Serial.print("pos: "); Serial.print(pos); Serial.println();
    myservo.write(pos);
    pos += 5;
    delay(100);
  }
}

void deliver(Servo myservo)
{
  // open claw SLOWLY (in increments of 10 degrees) until juice box released
  // note: claw is closed at 180, fully open at 0
  
  for (pos = posRetrieved; pos >= 0; pos -= 10) {
    myservo.write(pos); // tell servo to go to position in variable 'pos'
    delay(500);          // waits 25ms for the servo to reach the position
  }
}

#endif // CLAW_INCLUDED