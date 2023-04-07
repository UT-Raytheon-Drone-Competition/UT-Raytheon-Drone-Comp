#include <TimerOne.h>
#include <Servo.h>

bool wasTagged = false;

// Motor interrupt pins?

///// MOTOR CONSTANTS /////
#define MOTOR_FL 9
#define MOTOR_FR 8
#define MOTOR_BL 11
#define MOTOR_BR 10

Servo esc_FL; // create servo object to control the PWM signal
Servo esc_FR;
Servo esc_BL;
Servo esc_BR;
///////////////////////////


//// BUZZER AND LIGHTS ////
#define LED 12
#define BUZZER1 1
#define BUZZER2 2
float G = 783.99;
///////////////////////////


// distance and time vars?

// disk counters?

//float diskslots = 20.00;


void setup() {
  Serial.begin(9600);

  // Attach Motor Pins to Servo object
  esc_FL.attach(MOTOR_FL);  // make sure to use a PWM capable pin
  esc_FR.attach(MOTOR_FR);
  esc_BL.attach(MOTOR_BL);
  esc_BR.attach(MOTOR_BR);

  // Init. Lights and Sound
  pinMode(LED, OUTPUT);
  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);

}

void loop() {
  
  if(!wasTagged){
    // Move all motors at constant power
    float power = 15;
    set_esc_power(esc_FL, power);
    set_esc_power(esc_FR, -power);
    set_esc_power(esc_BL, power);
    set_esc_power(esc_BR, power);
  }else{
    playTaggedSequence();
  }

}


void playTaggedSequence() {
  // Blink the LED and sound the Buzzers for three
  // seconds. Then turn off the Buzzers for 60 seconds.

  for (int i = 0; i < 3; i++) {
    //light and buzzer on for 1 second
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER1, G, 10000);
    tone(BUZZER2, G, 10000);

    //light and buzzer off for 1 second
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER1);
    noTone(BUZZER2);
    delay(10000);
  }
  // keep lights on but delay buzzers ~60sec
  digitalWrite(LED_PIN, HIGH);
  delay(60 * 10000);
}


void set_esc_power(Servo esc, int power){
  // Set given Servo object to given output power

  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
}

// Interrupt Service Routines?




