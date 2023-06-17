#include <util/atomic.h> // For the ATOMIC_BLOCK macro

//pins (pwm, IN1, IN2, A, B), (OUTPUT, OUTPUT, OUTPUT, INPUT, INPUT)
// move m3 A 
int m1_pins[] = {4, 22, 23, 19, 31};
int m2_pins[] = {5, 24, 25, 3, 32};
int m3_pins[] = {6, 26, 27, 18, 33};
int m4_pins[] = {7, 28, 29, 2, 34};

int motor_counter = 1;

// parameters
float kp = 6;
float ki = 0; // WAS 1
float kd = 0.01; // was 0.001

// m1_variables
long m1_prevT = 0;
int m1_posPrev = 0;
volatile int m1_pos_i = 0;
volatile float m1_velocity_i = 0;
volatile long m1_prevT_i = 0;
float m1_eintegral = 0;
float m1_lastError = 0;

// m2_variables
long m2_prevT = 0;
int m2_posPrev = 0;
volatile int m2_pos_i = 0;
volatile float m2_velocity_i = 0;
volatile long m2_prevT_i = 0;
float m2_eintegral = 0;
float m2_lastError = 0;

// m3_variables
long m3_prevT = 0;
int m3_posPrev = 0;
volatile int m3_pos_i = 0;
volatile float m3_velocity_i = 0;
volatile long m3_prevT_i = 0;
float m3_eintegral = 0;
float m3_lastError = 0;

// m4_variables
long m4_prevT = 0;
int m4_posPrev = 0;
volatile int m4_pos_i = 0;
volatile float m4_velocity_i = 0;
volatile long m4_prevT_i = 0;
float m4_eintegral = 0;
float m4_lastError = 0;

void setup_motor_pins() {
  //m1 pins pinMode
  for (int i=0; i<=2; i++) {
    pinMode(m1_pins[i], OUTPUT);
  }
  for (int i=3; i<=4; i++) {
    pinMode(m1_pins[i], INPUT);
  }
  // m2 pins pinMode
  for (int i=0; i<=2; i++) {
    pinMode(m2_pins[i], OUTPUT);
  }
  for (int i=3; i<=4; i++) {
    pinMode(m2_pins[i], INPUT);
  }
  // m3 pins pinMode
  for (int i=0; i<=2; i++) {
    pinMode(m3_pins[i], OUTPUT);
  }
  for (int i=3; i<=4; i++) {
    pinMode(m3_pins[i], INPUT);
  }
  // m4 pins pinMode
  for (int i=0; i<=2; i++) {
    pinMode(m4_pins[i], OUTPUT);
  }
  for (int i=3; i<=4; i++) {
    pinMode(m4_pins[i], INPUT);
  }
}

void m1_readEncoder() {
  int b;
  int increment = 0;
  b = digitalRead(m1_pins[4]);
  if (b>0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  m1_pos_i = m1_pos_i + increment;
  long m1_currT = micros();
  float m1_deltaT = ((float) (m1_currT - m1_prevT_i))/1.0e6;
  m1_velocity_i = increment/m1_deltaT;
  m1_pos_i = m1_pos_i + increment;
  //Serial.println("m1_readEncoder(): ");
  //Serial.print(m1_pos_i);
}

void m2_readEncoder() {
  int b;
  int increment = 0;
  b = digitalRead(m2_pins[4]);
  if (b>0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  m2_pos_i = m2_pos_i + increment;
  long m2_currT = micros();
  float m2_deltaT = ((float) (m2_currT - m2_prevT_i))/1.0e6;
  m2_velocity_i = increment/m2_deltaT;
  m2_pos_i = m2_pos_i + increment;
}

void m3_readEncoder() {
  int b;
  int increment = 0;
  b = digitalRead(m3_pins[4]);
  if (b>0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  m3_pos_i = m3_pos_i + increment;
  long m3_currT = micros();
  float m3_deltaT = ((float) (m3_currT - m3_prevT_i))/1.0e6;
  m3_velocity_i = increment/m3_deltaT;
  m3_pos_i = m3_pos_i + increment;
  //Serial.println("m3_readEncoder(): ");
}

void m4_readEncoder() {
  int b;
  int increment = 0;
  b = digitalRead(m4_pins[4]);
  if (b>0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  m4_pos_i = m4_pos_i + increment;
  long m4_currT = micros();
  float m4_deltaT = ((float) (m4_currT - m4_prevT_i))/1.0e6;
  m4_velocity_i = increment/m4_deltaT;
  m4_pos_i = m4_pos_i + increment;
  //Serial.println("m4_readEncoder(): ");
  //Serial.print(m4_pos_i);
}

void setup_motor_interrupts() {
  attachInterrupt(digitalPinToInterrupt(m1_pins[3]),m1_readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(m3_pins[3]),m3_readEncoder,RISING);
}

void setup_DC_motors() {
  setup_motor_pins();
  setup_motor_interrupts();
}

void setMotor(int dir, int pwmVal, int pins[]){
  int pwm = pins[0]; int in1 = pins[1]; int in2 = pins[2]; 
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void m1_spd_pid(int velocity_target) {
  int vt = 1*velocity_target;
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = m1_pos_i;
    velocity2 = m1_velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-m1_prevT))/1.0e6;
  float velocity1 = (pos - m1_posPrev)/deltaT;
  m1_posPrev = pos;
  m1_prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/1176.0*60.0;
  float v2 = velocity2/1176.0*60.0;

  float e = vt-v1;
  m1_eintegral = m1_eintegral + e*deltaT;

  float rateError = (e-m1_lastError)/deltaT;
  
  // JUST CHANGED
  float u = kp*e + ki*m1_eintegral + kd*rateError;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }

  /*
  if (motor_counter = 0) {
    pwr = 0;
    motor_counter++;
  }
  */

  setMotor(dir,pwr,m1_pins);
  setMotor(dir,pwr,m2_pins);

  /*
  Serial.print("m1:   ");
  Serial.print("vt:");  Serial.print(vt);  Serial.print(",");
  Serial.print("v1:"); Serial.print(v1); Serial.print(",");
  Serial.println();*/
  
  //delay(1);
  delay(1);

  m1_lastError = e;
}

void m3_spd_pid(int velocity_target) {
  int vt = -1*velocity_target;
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = m3_pos_i;
    velocity2 = m3_velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-m3_prevT))/1.0e6;
  float velocity1 = (pos - m3_posPrev)/deltaT;
  m3_posPrev = pos;
  m3_prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/1176.0*60.0;
  float v2 = velocity2/1176.0*60.0;

  float e = vt-v1;
  m3_eintegral = m3_eintegral + e*deltaT;

  float rateError = (e-m3_lastError)/deltaT;
  
  // JUST CHANGED
  float u = kp*e + ki*m3_eintegral + kd*rateError;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }

  setMotor(dir,pwr,m3_pins);
  setMotor(dir,pwr,m4_pins);

  /*
  Serial.print("m3:   ");
  Serial.print("vt:");  Serial.print(vt);  Serial.print(",");
  Serial.print("v1:"); Serial.print(v1); Serial.print(",");
  Serial.println();*/

  //delay(1);
  delay(1);

  m3_lastError = e;
}

void m_brake(int brake_time) {
  long current_time = millis();
  while (millis() < current_time + brake_time) {
    setMotor(1, 0, m1_pins);
    setMotor(1, 0, m2_pins);
    setMotor(1, 0, m3_pins);
    setMotor(1, 0, m4_pins);
  }
}

void m_debug() {
  
  int i = 180;
  while (i > -180) {
    m1_spd_pid(i);
    m3_spd_pid(i);
    i--;
    delay(10);
  }
  m_brake(1000);
  delay(10000);
}

void m_reverse(int reverse_time) {
  long current_time = millis();
  while (millis() < current_time + reverse_time) {
    m1_spd_pid(-80);
    m3_spd_pid(-80);
  }
  m_brake(50);
}

void m_forward(int forward_time) {
  long current_time = millis();
  while (millis() < current_time + forward_time) {
    m1_spd_pid(60);
    m3_spd_pid(60);
  }
  m_brake(50);
}

void reset_motors() {
  m1_prevT = 0;
  m1_posPrev = 0;
  m1_pos_i = 0;
  m1_velocity_i = 0;
  m1_prevT_i = 0;
  m1_eintegral = 0;
  m1_lastError = 0;

  m2_prevT = 0;
  m2_posPrev = 0;
  m2_pos_i = 0;
  m2_velocity_i = 0;
  m2_prevT_i = 0;
  m2_eintegral = 0;
  m2_lastError = 0;

  m3_prevT = 0;
  m3_posPrev = 0;
  m3_pos_i = 0;
  m3_velocity_i = 0;
  m3_prevT_i = 0;
  m3_eintegral = 0;
  m3_lastError = 0;

  m4_prevT = 0;
  m4_posPrev = 0;
  m4_pos_i = 0;
  m4_velocity_i = 0;
  m4_prevT_i = 0;
  m4_eintegral = 0;
  m4_lastError = 0;
}