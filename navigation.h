#include "DC_motors.h"
#include "claw.h"
#include "teleArm.h"

//int black = 940;
//Map these pins when the electronics are assembled
// the outer 2 sensors are reserved for turning
int ir1 = A15;
int ir2 = A14;
int ir3 = A13;
int ir4 = A12;
int ir5 = A11;
int ir6 = A10;
int ir7 = A9;
int ir8 = A8;
int ir9 = A7;
int ir10 = A6;
int ir11 = A5;
int ir12 = A4;
int LEDL = 53; // LED SIGNAL FOR LEFT
int LEDR = 52; // LED SIGNAL FOR RIGHT
int irL = A3; // far left
int irR = A2; // far right

// 0.22, 0.001
double Kp = 0.20;
double Kd = 0.001; // was 0.001
int IFSensor[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
int leftSensor;
int rightSensor;
double default_speed = 30;

int u_arm_ePin = 51; //arm Echo
int u_arm_tPin = 50; //arm Trigger

int u_chas_ePin = 49; //chassis echo
int u_chas_tPin = 48; // chassis trigger

long IR_prevT = 0;
float IR_lastError = 0;
float IR_eIntegral = 0;

int dropoff_target;

int lineCount = 0;
bool onLine = false;

int IR_sensor_count = 0;
int u_sensor_count = 0;

char turn_direction;

double error;

long u_duration;
int cm, inches;

double PID;

int u_run_count = 0;

int turn_time = 2100; //ms

void setup_IR_sensors() {
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
  pinMode(ir6, INPUT);
  pinMode(ir7, INPUT);
  pinMode(ir8, INPUT);
  pinMode(ir9, INPUT);
  pinMode(ir10, INPUT);
  pinMode(ir11, INPUT);
  pinMode(ir12, INPUT);
  pinMode(LEDL, OUTPUT);
  pinMode(LEDR, OUTPUT);

  pinMode(u_chas_ePin, INPUT);
  pinMode(u_chas_tPin, OUTPUT);

  pinMode(irL, INPUT);
  pinMode(irR, INPUT);

  digitalWrite(LEDL, HIGH);
  digitalWrite(LEDR, HIGH);

}

void IR_debug() {
  //digitalWrite(LEDR, HIGH);
  //digitalWrite(LEDL, HIGH);
  while (1) {
    IFSensor[0] = analogRead(ir1);
    IFSensor[1] = analogRead(ir2);
    IFSensor[2] = analogRead(ir3);
    IFSensor[3] = analogRead(ir4);
    IFSensor[4] = analogRead(ir5);
    IFSensor[5] = analogRead(ir6);
    IFSensor[6] = analogRead(ir7);
    IFSensor[7] = analogRead(ir8);
    IFSensor[8] = analogRead(ir9);
    IFSensor[9] = analogRead(ir10);
    IFSensor[10] = analogRead(ir11);
    IFSensor[11] = analogRead(ir12);
    // LEFT SIDE
    Serial.print("leftSensor: "); Serial.print(analogRead(irL)); Serial.print(" || ");
    Serial.print("ir1: "); Serial.print(IFSensor[0]); Serial.print(", ");
    Serial.print("ir2: "); Serial.print(IFSensor[1]); Serial.print(", ");
    Serial.print("ir3: "); Serial.print(IFSensor[2]); Serial.print(", ");
    Serial.print("ir4: "); Serial.print(IFSensor[3]); Serial.print(", ");
    Serial.print("ir5: "); Serial.print(IFSensor[4]); Serial.print(", ");
    Serial.print("ir6: "); Serial.print(IFSensor[5]); Serial.print(" ");

    Serial.print("|| ");

    // RIGHT SIDE
    Serial.print("ir7: "); Serial.print(IFSensor[6]); Serial.print(", ");
    Serial.print("ir8: "); Serial.print(IFSensor[7]); Serial.print(", ");
    Serial.print("ir9: "); Serial.print(IFSensor[8]); Serial.print(", ");
    Serial.print("ir10: "); Serial.print(IFSensor[9]); Serial.print(", ");
    Serial.print("ir11: "); Serial.print(IFSensor[10]); Serial.print(", ");
    Serial.print("ir12: "); Serial.print(IFSensor[11]); Serial.print(" || ");
    Serial.print("rightSensor: "); Serial.print(analogRead(irR));

    Serial.println();
  }
}

bool wall_detection()
{
  float duration, cm, inches;
  digitalWrite(u_chas_tPin, LOW);
  delayMicroseconds(5);
  digitalWrite(u_chas_tPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(u_chas_tPin, LOW);

  duration = pulseIn(u_chas_ePin, HIGH);

  cm = (duration/2.0) / 29.1;
  inches = (duration/2.0) / 74.0;

  Serial.println(inches);

  if(inches < 1.5)
  {
    if (u_sensor_count == 2) {
      u_sensor_count = 0;
      Serial.println("CONDITION");
      return true;
    }
    else {
      u_sensor_count++;
    }
  }
  return false;
}

bool obstacle_detection()
{
  float duration, cm, inches;
  digitalWrite(u_chas_tPin, LOW);
  delayMicroseconds(5);
  digitalWrite(u_chas_tPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(u_chas_tPin, LOW);

  duration = pulseIn(u_chas_ePin, HIGH);

  cm = (duration/2.0) / 29.1;
  inches = (duration/2.0) / 74.0;

  Serial.println(inches);

  if(inches < 4.0)
  {
    if (u_sensor_count == 2) {
      u_sensor_count = 0;
      Serial.println("CONDITION");
      return true;
    }
    else {
      u_sensor_count++;
    }
  }
  else if (u_sensor_count > 0) {
    u_sensor_count--;
  }
  return false;
  //delay(10);
}

void get_IR_readings() {
  //digitalWrite(LEDL, HIGH);
  //digitalWrite(LEDR, HIGH);
  IFSensor[0] = analogRead(ir1);
  IFSensor[1] = analogRead(ir2);
  IFSensor[2] = analogRead(ir3);
  IFSensor[3] = analogRead(ir4);
  IFSensor[4] = analogRead(ir5);
  IFSensor[5] = analogRead(ir6);
  IFSensor[6] = analogRead(ir7);
  IFSensor[7] = analogRead(ir8);
  IFSensor[8] = analogRead(ir9);
  IFSensor[9] = analogRead(ir10);
  IFSensor[10] = analogRead(ir11);
  IFSensor[11] = analogRead(ir12);
  leftSensor = analogRead(irL)-20;
  rightSensor = analogRead(irR)-20;
}

double calc_Error()
{
  error = 0;
  get_IR_readings();
  for (int i = 0; i < 12; i++)
  {
    if(i < 6)
    {
    error-=IFSensor[i];       
    } 
    else
    {
      error+=IFSensor[i];
    }
  }
  //Serial.println("Error: "); Serial.print(error);
  return error;

}

double calc_PID()
{
  error = calc_Error();
  long IR_currT = micros();
  float IR_deltaT = ((float) (IR_currT - IR_prevT))/1.0e6;
  float error_derivative = (error - IR_lastError)/IR_deltaT;

  IR_lastError = error;
  IR_prevT = IR_currT;

  double PID = (Kp*error + Kd*error_derivative);

  return PID;
}

void run_PID(double default_speed)
{
    double PID = calc_PID();
    //Serial.print("PID: "); Serial.print(PID); Serial.println();

    if (abs(PID) > 50) {
      PID = 2 * PID;
    }

    if(PID < 0)
    {
      m1_spd_pid(default_speed+abs(PID));
      m3_spd_pid(default_speed-abs(PID));
    }
    else if(PID > 0)
    {
      m1_spd_pid(default_speed-abs(PID));
      m3_spd_pid(default_speed+abs(PID));
    }
    else
    {
      m1_spd_pid(default_speed);
      m3_spd_pid(default_speed);
    }
    // JUST CHANGED 
    delay(1);
}

void turn_90(char direction)
{
  m_brake(100);
  int tSpeed = 80;
  long current_time = millis();

  // JUST CHANGED
  m1_lastError = 0;
  m3_lastError = 0;

  if(direction == 'l')
  {
    Serial.println("left_turn");
    while (millis() < current_time + turn_time) {
      m1_spd_pid(tSpeed);
      m3_spd_pid(-tSpeed);
      //Serial.println("IN LEFT TURN LOOP");
    }
  }
  else if (direction == 'r')
  {
    Serial.println("right_turn");
    while (millis() < current_time + turn_time) {
      m1_spd_pid(-tSpeed);
      m3_spd_pid(tSpeed);
      //Serial.println("IN RIGHT TURN LOOP");
    }
  }
  m_brake(100);
 }

bool intersectionDetection_right(int target)
 {
  get_IR_readings();

  //Serial.print("leftSensor: "); Serial.print(leftSensor); Serial.print(",");
  Serial.print("IR10:"); Serial.print(analogRead(A10)); Serial.print(",");
  Serial.print("rightSensor"); Serial.print(rightSensor); Serial.print(","); Serial.println();
  //delay(50);

  if ((rightSensor > analogRead(A10) - 20) && (rightSensor < analogRead(A10) + 20) && !onLine) // check if leftSensor and IFSensor[10] are on same colors (intersection)
  {
    if (IR_sensor_count == 2) {
      Serial.println("Intersection Detected");
      onLine = true;
      IR_sensor_count = 0;
      long PID_time = millis() + 100;
      while (millis() < PID_time) {
        run_PID(60);
      }
    }
    else {
      Serial.println("INCREMENT");
      IR_sensor_count++;
    }
  }
  else if(rightSensor < analogRead(A10) - 30 && onLine) // check if leftSensor and IFSensor[10] are on different colors (after intersection)
  {
    if (IR_sensor_count == 3) {
      Serial.println("White Detected after Intersection");
      onLine = false;
      IR_sensor_count = 0;
      lineCount++;
    }
    else {
      IR_sensor_count++;
    }
  }

  if(lineCount == target)
  {
    return true;    
  }
  return false;
 }

 bool intersectionDetection(int target)
 {
  calc_Error();

  Serial.print("leftSensor: "); Serial.print(leftSensor); Serial.print(",");
  Serial.print("IR10:"); Serial.print(analogRead(A10)); Serial.print(",");
  Serial.print("rightSensor"); Serial.print(rightSensor); Serial.print(","); Serial.println();
  //delay(50);

  if (((leftSensor > analogRead(A10) - 20) && (leftSensor < analogRead(A10) + 20)) && !onLine) // check if leftSensor and IFSensor[10] are on same colors (intersection)
  {
    Serial.println("INTERSECTION DETECTED");
    onLine = true;
    long PID_time = millis() + 200;
    while (PID_time < millis()) {
      run_PID(60);
    }
    /*
    if (IR_sensor_count == 1) {
      Serial.println("Intersection Detected");
      onLine = true;
      IR_sensor_count = 0;
      long PID_time = millis() + 200;
      while (PID_time < millis()) {
        run_PID(60);
      }
    }
    else {
      Serial.println("INCREMENT");
      IR_sensor_count++;
    }*/
  }
  else if(leftSensor < analogRead(A10) - 30 && onLine) // check if leftSensor and IFSensor[10] are on different colors (after intersection)
  {
    Serial.println("WHITE DETECTED AFTER INTERSECTION");
    onLine = false;
    lineCount++;
    /*
    if (IR_sensor_count == 1) {
      Serial.println("WHITE DETECTED AFTER INTERSECTION");
      onLine = false;
      IR_sensor_count = 0;
      lineCount++;
    }
    else {
      IR_sensor_count++;
    }*/
  }

  if(lineCount == target)
  {
    lineCount = 0;  
    return true; 
  }
  return false;
 }

void execute_line_count(int target) {
  // CHANGED 6/10/23 14:27
  if (target % 2 == 1) {
    turn_direction = 'l';
    if (target == 1) {
      dropoff_target = 1;
      Serial.println("TARGET WAS 1");
    }
    else if (target == 3) {
      dropoff_target = 3;
      target = target - 1;
      Serial.println("TARGET WAS 3");
    }
    else if (target == 5) {
      dropoff_target = 5;
      target = target - 2;
      Serial.println("TARGET WAS 5");
    }
  }
  else {
    turn_direction = 'r';
    if (target == 2) {
      dropoff_target = 2;
      target = target - 1;
      Serial.println("TARGET WAS 2");
    }
    else if (target == 4) {
      dropoff_target = 4;
      target = target - 2;
      Serial.println("TARGET WAS 4");
    }
    else if (target == 6) {
      dropoff_target = 6;
      target = target - 3;
      Serial.println("TARGET WAS 6");
    }
  }
  while(!intersectionDetection(target)) 
  {
    run_PID(60);
  }
  Serial.println("TARGET MET");
  m_brake(100);
  turn_90(turn_direction);

  // code to reverse
  m_brake(100);
  //m_reverse(1700);
}

void approach_wall() {
  while(!wall_detection()) {
    Serial.println("RUNNING WALL DETECTION");
    run_PID(60);
  }
  m_brake(1000);
}

void reverse_to_line() {
  //get_IR_readings();
  Serial.println("REVERSE");
  Serial.print("leftSensor: "); Serial.print(leftSensor); Serial.print(",");
  Serial.print("IR10:"); Serial.print(analogRead(A10)); Serial.print(",");
  Serial.print("rightSensor"); Serial.print(rightSensor); Serial.print(","); Serial.println();
  while (IR_sensor_count < 3) {
    if (analogRead(irL) > analogRead(A10) - 20 && analogRead(irR) > analogRead(A10) - 20) {
      IR_sensor_count++;
    }
    Serial.println("REVERSING BACK TO LINE");
    m_reverse(50);
  }
  /*
  while(analogRead(irL) < analogRead(A10) - 20 || analogRead(irR) < analogRead(A10) - 20 || IR_sensor_count < 3) {
    if (analogRead())
    Serial.println("REVERSING BACK TO LINE");
    m_reverse(10);
  }
  */
  IR_sensor_count = 0;
  Serial.println("AT LINE");
  m_brake(100);
}

void reverse_turn() {
  if (turn_direction == 'r') {
    turn_direction = 'l';
    Serial.println("turn_direction was 'r' and is now 'l'.");
  }
  else if (turn_direction == 'l') {
    turn_direction = 'r';
    Serial.println("turn_direction was 'l' and is now 'r'.");
  }
  turn_90(turn_direction);
  m_brake(1000);
}

bool all_black() {
  //digitalWrite(LEDR, HIGH);
  //digitalWrite(LEDL, HIGH);
  get_IR_readings();
  
  Serial.print("A10: "); Serial.print(ir6); Serial.println();
  Serial.print("irL: "); Serial.print(leftSensor); Serial.println();
  Serial.print("irR: "); Serial.print(rightSensor); Serial.println();
  
  if ((analogRead(irL) > analogRead(A10) - 20) && (analogRead(irL) < analogRead(A10) + 20)) { // check if leftSensor and IFSensor[10] are on same colors (intersection))
    
    if (IR_sensor_count == 1) {
      IR_sensor_count = 0;
      Serial.println("ALL BLACK DETECTED");
      return true;
    }
    else {
      IR_sensor_count++;
      Serial.println("INCREMENT");
    }
    
    /*
    if (IR_sensor_count == 1) {
      IR_sensor_count = 0;
      Serial.println("TRUE");
      return true;
    }
    else {
      Serial.println("incrementing");
      IR_sensor_count++;
    }*/
  }
  else {
    if (IR_sensor_count > 0) {
      IR_sensor_count = 0;
    }
    Serial.println("FALSE");
    return false;
  }
}

void dropoff_one() {
  long pid_time = millis() + 1500;
  while (millis() < pid_time) {
    run_PID(60);
  }
  m_brake(10);
  fullRetract(100);
  myservo.write(0);
  m_brake(1000);
  //m_reverse(1000);
  reverse_to_line();
  m_forward(500);
  turn_90('l');
}

void dropoff_B(int target) {
  long PID_time = millis() + 250;
  while (millis() < PID_time) {
    run_PID(60);
  }
  m_brake(50);
  turn_90('l');
  PID_time = millis() + 1000;
  while (millis() < PID_time) {
    run_PID(60);
  }
  Kp = 0.1;
  while(!intersectionDetection_right(target - 1)) {
    run_PID(60);
  }
  turn_90('r');
  Kp = 0.2;
  long pid_time = millis() + 1250;
  while (millis() < pid_time) {
    run_PID(60);
  }
  m_brake(10);
  fullRetract(100);
  myservo.write(0);
  m_brake(1000);
  reverse_to_line();
  m_forward(500);
  turn_time = 2100;
  turn_90('l');
}

void execute_dropoff(int target) {
  //Kp = 0.1;
  if (target == 1) {
    Serial.println("EXECUTING DROPOFF: 1");
    dropoff_one();
  }
  else if (target == 2, 3, 4, 5, 6) {
    Serial.println("EXECUTING DROPOFF: "); Serial.print(target);
    dropoff_B(target);
  }
  else {
    Serial.println("INVALID DROPOFF TARGET");
    delay(600000);
  }
}

void PID_debug() {
  int test_PID = 60;
  while (1) {
    m1_spd_pid(60+test_PID);
    m3_spd_pid(60+test_PID);
    test_PID = test_PID - 10;
    delay(10);
    Serial.println("while loop");
  }
  m_brake(1000);
}