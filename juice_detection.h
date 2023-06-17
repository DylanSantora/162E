int echoPin = 51; //Echo
int trigPin = 50; //Trigger

int pulse_counter = 0;

long duration;
int distance;

void setup_juice_detection() {
  Serial.begin(9600);
  Serial.println("setup");
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
}

bool juice_detection()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  if (distance < 10) {
    return true;
  }
  return false;
}