#include "TimerOne.h"
#include "Servo.h"

// Tags
bool wasTagged = false;
bool buzzerWentOff = false; // not used

// Constants for Interrupt Pins CHANGE VALUES IF NOT USING ARDUINO UNO
const byte MOTOR_FL = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR_FR = 3;  // Motor 2 Interrupt Pin - INT 1
const byte MOTOR_BL = 4;
const byte MOTOR_BR = 5;

// Motor Controller Constants
Servo esc_FL;  // create servo object to control PWM signal
Servo esc_FR;
Servo esc_BL;
Servo esc_BR;


// Lights and Buzzers
#define LED_PIN 12
#define BUZZER1 8
#define BUZZER2 9
float G = 783.99;

// Distance and Time Vars
float time_elapsed = 0;
float distance_traveled = 0;

// Integers to count disk pulses
unsigned int counter_FL = 0;
unsigned int counter_FR = 0;
unsigned int counter_BL = 0;
unsigned int counter_BR = 0;

// Float for number of slots in encoder disk CHANGE TO MATCH VALUE OF ENCODER DISK
float diskslots = 20.00;

void setup() {
  Serial.begin(9600);

  // attach microcontroller
  esc_1.attach(MICROCONTROLLER);  // make sure to use PWM-capable pin

  // init lights and sound
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);

  // init timer to count 1 second
  Timer1.initialize(1000000);

  // Attach the motors/timer to the ISR functions
  attachInterrupt(digitalPinToInterrupt(MOTOR1), ISR_count1, RISING);  // inc counter1 when speed sensor pin goes HIGH
  attachInterrupt(digitalPinToInterrupt(MOTR2), ISR_count2, RISING);   // "" for counter2
  Timer1.attachInterrupt(ISR_timerone);                                // Enable the timer
}


void loop() {

  if (!wasTagged):
    // check if was tagged and if so break from if
    

    // move forward at X speed
    set_esc_power(esc_1, 0); // set power of motor

  // check speed and adjust power




  // if(time_elapsed == X):
  // turnRight();
  // move X yards;
  // turn left

  else:
    playTaggedSequence();
}

void playTaggedSequence() {
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

void set_esc_power(Servo esc, int power) {
  power = constrain(power, -100, 100);                                // 100 is full speed. Negative is backwards.
  int signal_min = 1050;                                              // DEPENDS ON MOTOR CONTROLLER. CHECK.
  int signal_max = 1950;                                              // DEPENDS ON MOTOR CONTROLLER. CHECK.
  int signal_output = map(power, -100, 100, signal_min, signal_max);  //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
}

//// Interrupt Service Routines ////
// MOTOR1 PULSE COUNT ISR
void ISR_count() {
  counter1++;  // "manually" increment Motor1 counter
}

// MOTOR2 PULSE COUNT ISR
void ISR_count2() {
  counter2++;  // "manually" increment Motor2 counter
}

//TimerOne ISR
void ISR_timerone() {
  Timer1.detachInterrupt();  // stop the timer
  Serial.print("Motor Speed 1: ");
  float rotation1 = (counter1 / diskslots) * 60.00;  // calc. RPM for motor1
  Serial.print(rotation1);
  Serial.print("RPM - ");
  counter1 = 0;  //reset counter1 to zero

  Serial.print("Motor Speed 2: ");
  float rotation2 = (counter2 / diskslots) * 60.00;  //calc RPM for Motor2
  Serial.print(rotation2);
  Serial.print(" RPM");
  counter2 = 0;                          // reset counter2 to zero
  Timer1.attachInterrupt(ISR_timerone);  //Re-enable the timer
}
