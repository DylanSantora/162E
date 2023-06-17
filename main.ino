#include "navigation.h"
#include "remote.h"

void setup() {
  Serial.begin(9600);
  setup_DC_motors();
  setup_IR_sensors();
  setup_juice_detection();
  setup_remote();
}

void loop() 
{
  // SETUP
  myservo.attach(8);
  Serial.println("myservo.attach(8) complete");
  openClaw(myservo);
  Serial.println("openClaw(myservo)");
  delay(100);

  // PART 0: IR REMOTE INPUT
  bool valid_input_received = false;
  int input;
  while (!valid_input_received) {
    Serial.println("NO INPUT");
    delay(1);
    input = get_IR_input();
    if (input != -1) {
      valid_input_received = true;
    }
  }

  // PART 1: JUICE BOX RETRIEVAL
  execute_line_count(input);
  ArmExtend(900);
  approach_wall();
  retrieve(myservo);
  ArmExtend(100);

  // PART 2: STARTING AREA
  delay(100);
  reverse_to_line();
  fullRetract(800);

  m_forward(500);
  turn_time = 1700;
  reverse_turn();
  m_brake(1000);

  // PART 3: OBSTACLE AREA
  while(!obstacle_detection()) {
    run_PID(60);
    Serial.println("no obstacle detected");
  }
  m_brake(100);
  while(obstacle_detection()) {
    m_brake(10);
    Serial.println("OBSTACLE DETECTED");
  }

  // PART 4: WILSON WAREHOUSE
  while(!all_black()) {
    run_PID(60);
    Serial.println("no black detected");
  }
  m_brake(100);

  // PART 5: JUICE BOX DELIVERY
  execute_dropoff(dropoff_target);
  Kp = 0.1;

  // PART 6: FINISH
  while(!obstacle_detection()) {
    run_PID(80);
  }
  m_brake(50);
}
