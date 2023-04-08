// PINS NOT CORRECT
#define US_Trig 9
#define US_Echo 10

long duration;
int distance;
float SPEED_OF_SOUND = 0.034; // cm/s
float OBSTACLE_THRESHOLD = 57.0; // cm

void setup() {
  pinMode(US_Trig, OUTPUT);
  pinMode(US_Echo, INPUT);
  Serial.begin(9600);
}

void loop() {

  float obstruction_distance = get_obstruction_distance();
  Serial.print("Distance in cm: ");
  Serial.println(distance);

  
  if(obstruction_distance <= OBSTACLE_THRESHOLD){
    Serial.print("Avoid Obstacle\n\n\n");
  }
  
}

float get_obstruction_distance(){
  // clear output of US sensor briefly
  digitalWrite(US_Trig, LOW);
  delayMicroseconds(2);

  // generate ultrasound wave
  digitalWrite(US_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_Trig, LOW);

  // init wave reading vars
  duration = pulseIn(US_Echo, HIGH);
  distance = duration*SPEED_OF_SOUND/2;

  return distance;  
}
