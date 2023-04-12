#include <Servo.h>

// Motor Vars
#define MOTOR_FL 9
#define MOTOR_FR 8
#define MOTOR_BL 11
#define MOTOR_BR 10

Servo esc_FL;  // create servo object to control the PWM signal
Servo esc_FR;
Servo esc_BL;
Servo esc_BR;

// Encoder Vars

struct speedCheckReturn{
  double currentSpeed;
  double distanceTraveled;
  unsigned long totalSteps;       // total steps triggered - important for persistent odometry
  unsigned long stepTimes[3];     // array for tracking step times - must be declared externally so it is persistent trigger to trigger
  unsigned int stepTimesIndex;        // tracks the index of the time tracking array for speed calculations
  unsigned int lastEncoderState;  // records previous encoder state
  unsigned int encoderPin;        // pin number for encoder
};

speedCheckReturn MOTOR_FL_SPEED;  // speedCheckReturn declare
speedCheckReturn MOTOR_FR_SPEED;
speedCheckReturn MOTOR_BL_SPEED;
speedCheckReturn MOTOR_BR_SPEED;

double wheelDiameter = 0.13;    // wheel diameter in meters
double stepsPerRotation = 20.0; // number of slots in encoder wheel
String units = "imperial";        // select "imperial", "metric", or "rotational"
unsigned int stepTimesLength = sizeof(MOTOR_FL_SPEED.stepTimes)/sizeof(MOTOR_FL_SPEED.stepTimes[0]); // length of stepTimes array
// IMPORTANT: stepTimesLength determines number of rotations speed will be averaged over as (stepTimesLength/stepsPerRotation)

void setup() {

  // Motor
  esc_FL.attach(MOTOR_FL);  // make sure to use a PWM capable pin
  esc_FR.attach(MOTOR_FR);
  esc_BL.attach(MOTOR_BL);
  esc_BR.attach(MOTOR_BR);

  // encoder
  MOTOR_FL_SPEED.encoderPin = 18;  // Rotary Encoder Inputs
  MOTOR_FR_SPEED.encoderPin = 19;
  MOTOR_BL_SPEED.encoderPin = 20;
  MOTOR_BR_SPEED.encoderPin = 21;
  pinMode(MOTOR_FL_SPEED.encoderPin,INPUT);  // Set encoder pins as inputs
  pinMode(MOTOR_FR_SPEED.encoderPin,INPUT);  
  pinMode(MOTOR_BL_SPEED.encoderPin,INPUT);  
  pinMode(MOTOR_BR_SPEED.encoderPin,INPUT);  
  MOTOR_FL_SPEED.lastEncoderState = digitalRead(MOTOR_FL_SPEED.encoderPin);  // Read the initial state of encoder1
  MOTOR_FR_SPEED.lastEncoderState = digitalRead(MOTOR_FR_SPEED.encoderPin);
  MOTOR_BL_SPEED.lastEncoderState = digitalRead(MOTOR_BL_SPEED.encoderPin);
  MOTOR_BR_SPEED.lastEncoderState = digitalRead(MOTOR_BR_SPEED.encoderPin);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_SPEED.encoderPin), updateEncoderFL, CHANGE);  // Call updateEncoder when any high/low changed seen
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_SPEED.encoderPin), updateEncoderFR, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(MOTOR_BL_SPEED.encoderPin), updateEncoderBL, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(MOTOR_BR_SPEED.encoderPin), updateEncoderBR, CHANGE); 

  Serial.begin(9600);
}

void loop() {
  //float power = 10.0;
  //all_constant_test(power);

  //set_esc_power(esc_BR, -power);

  MOTOR_FL_SPEED = speedCheck(MOTOR_FL_SPEED);
  MOTOR_FR_SPEED = speedCheck(MOTOR_FR_SPEED);
  MOTOR_BL_SPEED = speedCheck(MOTOR_BL_SPEED);
  MOTOR_BR_SPEED = speedCheck(MOTOR_BR_SPEED);
  Serial.print(MOTOR_FL_SPEED.currentSpeed); Serial.print("|||");Serial.print(MOTOR_FL_SPEED.stepTimesIndex); Serial.print("|||");Serial.println(MOTOR_FL_SPEED.stepTimes[MOTOR_FL_SPEED.stepTimesIndex]);
  
  for(int i = 1; i <= 10; i++){
    set_esc_power(esc_BR, 10*i);
    //speedCheck1();
    //Serial.print(10*i);
    //Serial.println();
    //Serial.print(MOTOR_BR_SPEED.currentSpeed);
    //Serial.println();
    //Serial.println();
    delay(5*1000);
  }

}

// Motor Functions 
void set_esc_power(Servo esc, int power){
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
}

void all_constant_test(float power){
  // power the motors at a constant value
  // for N seconds
  //int power = 12; // NEED MIN OF 12 FOR ALL WHEELS TO MOVE
  int numSeconds = 10;
  set_esc_power(esc_FL, power);
  set_esc_power(esc_FR, -power);
  set_esc_power(esc_BL, power);
  set_esc_power(esc_BR, -power);
  delay(numSeconds*1000);
}

// Encoder functions
void updateEncoderFL(){
  MOTOR_FL_SPEED = updateEncoderHelper(MOTOR_FL_SPEED);   // call helper to update global vars outside ISR
}
void updateEncoderFR(){
  MOTOR_FR_SPEED = updateEncoderHelper(MOTOR_FR_SPEED);   // call helper to update global vars outside ISR
}
void updateEncoderBL(){
  MOTOR_BL_SPEED = updateEncoderHelper(MOTOR_BL_SPEED);   // call helper to update global vars outside ISR
}
void updateEncoderBR(){
  MOTOR_BR_SPEED = updateEncoderHelper(MOTOR_BR_SPEED);   // call helper to update global vars outside ISR
}

speedCheckReturn updateEncoderHelper(speedCheckReturn UE){  // update global vars
  int currentEncoderState = digitalRead(UE.encoderPin);   // Read the current state
  ////
  if (currentEncoderState != UE.lastEncoderState  && currentEncoderState == 1){    // If last and current state are different, then pulse occurred
      UE.totalSteps++;
      if (UE.stepTimesIndex == (stepTimesLength-1)){     // if at end of array, cycles back to pos 0 - FIFO array
        UE.stepTimesIndex = 0;}
      else{
        UE.stepTimesIndex++;}
      UE.stepTimes[UE.stepTimesIndex] = millis();
      }
  UE.lastEncoderState = currentEncoderState;  // Remember last encoder state
  return UE;
}

speedCheckReturn speedCheck(speedCheckReturn SCR){
  unsigned long deltaT;   
  double currentRPS;      // current rotations per second
  double totalRotations;  // finds the number of rotations completed based on the number of encoder state changes that should occur per rotation
  double travelPerRotation = wheelDiameter*PI;  // distance traveled per wheel rotation
  ////
  totalRotations = SCR.totalSteps/stepsPerRotation;
  ////
  if (SCR.stepTimesIndex == (stepTimesLength-1)){         // finds time difference over one "revolution" of stepTimes
    deltaT = SCR.stepTimes[SCR.stepTimesIndex] - SCR.stepTimes[0];    
  } else {
    deltaT = SCR.stepTimes[SCR.stepTimesIndex] - SCR.stepTimes[SCR.stepTimesIndex+1];
  }
  currentRPS = (stepTimesLength/stepsPerRotation)/(deltaT/1000.0);
  ////
  if (units == "metric"){
    SCR.currentSpeed = currentRPS*travelPerRotation;      // speed in meters/second
    SCR.distanceTraveled = totalRotations*travelPerRotation;  // distance traveled in meters
  }
  if (units == "imperial"){
    SCR.currentSpeed = currentRPS*travelPerRotation*2.23694;      // speed in miles per hour
    SCR.distanceTraveled = totalRotations*travelPerRotation*3.28084;  // distance traveled in feet
  }
  if (units == "rotational"){
    SCR.currentSpeed = currentRPS/60.0;       // speed in rotations per minute
    SCR.distanceTraveled = totalRotations;    // rotations completed
  }
  ////

  return SCR;
}
