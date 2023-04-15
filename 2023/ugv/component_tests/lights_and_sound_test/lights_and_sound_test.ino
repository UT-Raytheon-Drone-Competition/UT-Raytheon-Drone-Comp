// PINS NOT CORRECT

#define LED_PIN 12
#define B1 8
#define B2 9
#define B3 10

float F = 698.46;
float G = 783.99;
float A = 880.00;
float C = 1046.50;
float B = 987.77;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  //digitalWrite(LED_PIN, HIGH)
}

void loop() {
  //digitalWrite(LED_PIN, HIGH);
  //tone(BUZZER, 1000, 1000);
  //delay(1000);
  
  //(LED_PIN, LOW);
  //noTone(BUZZER);
  //delay(2000);

  tone(B1, F, 1000);
  delay(1000);
  noTone(B1);

  tone(B2, A, 1000);
  delay(1000);
  noTone(B2);

  tone(B3, C, 1000);
  delay(1000);
  noTone(B3);
}

void CHECK_HIT() {
  bool UGV_WAS_TAGGED;
  if(digitalRead(12) == HIGH){
    UGV_WAS_TAGGED = true;
    allMotorsOff();
    // SEND TAGGED IRC MESSAGE
    playTaggedSequence();
    }
  else{
    UGV_WAS_TAGGED = false;
    goStraight(DISTANCE_TO_GO, totalDistTraveled, 20);
    }
