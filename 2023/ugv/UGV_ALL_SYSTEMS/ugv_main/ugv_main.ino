#include <Servo.h>
//#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile
#include <stdio.h>
//#include <XInput.h>

#define MOTOR_FL 9
#define MOTOR_FR 8
#define MOTOR_BL 11
#define MOTOR_BR 10

#define ENCODER_FL 18
#define ENCODER_FR 19
#define ENCODER_BL 20
#define ENCODER_BR 21

#define RECEIVER 4
#define TRANSMITTER 2
#define PTT 5 //idk what this is but DONT TOUCH

#define LED 3
#define READ_HITS 12

#define US_Trig 6
#define US_Echo 7

#define DPAD_UP 13
#define DPAD_DOWN 14
#define BUTTON_X 15

////////// MOTOR VARS //////////
Servo esc_FL;  // create servo object to control the PWM signal
Servo esc_FR;
Servo esc_BL;
Servo esc_BR;

// modifier values for motor power
float FL_POWER_MOD;
float FR_POWER_MOD;
float BL_POWER_MOD;
float BR_POWER_MOD;

////////// ENCODER VARS //////////
struct speedCheckReturn{
  double currentSpeed;
  double distanceTraveled;
  unsigned long totalSteps;       // total steps triggered - important for persistent odometry
  unsigned long stepTimes[3];     // array for tracking step times - must be declared externally so it is persistent trigger to trigger
  unsigned int stepTimesIndex;        // tracks the index of the time tracking array for speed calculations
  unsigned int lastEncoderState;  // records previous encoder state
  unsigned int encoderPin;        // pin number for encoder
};

speedCheckReturn MOTOR_FL_SPEED;  // speedCheckReturn init
speedCheckReturn MOTOR_FR_SPEED;
speedCheckReturn MOTOR_BL_SPEED;
speedCheckReturn MOTOR_BR_SPEED;

float WHEEL_SPEED_TOL = .1;
double wheelDiameter = 0.13;    // wheel diameter in meters
double stepsPerRotation = 20.0; // number of slots in encoder wheel
String units = "imperial";        // select "imperial", "metric", or "rotational"
unsigned int stepTimesLength = sizeof(MOTOR_FL_SPEED.stepTimes)/sizeof(MOTOR_FL_SPEED.stepTimes[0]); // length of stepTimes array
// IMPORTANT: stepTimesLength determines number of rotations speed will be averaged over as (stepTimesLength/stepsPerRotation)

////////// MISCELL. VARS //////////
//RH_ASK driver(2000, RECEIVER, TRANSMITTER, PTT); // transmitter init

bool UGV_WAS_TAGGED = false; // tag status
bool UGV_WAS_KILLED = false; // kill switch

float totalDistTraveled = 0.0; // distance counter
float DISTANCE_TO_GO = 10.0; // in feet

// time vars
float prevTime; // in milliseconds
float deltaT;   // in milliseconds
float currTime; // IN SECONDS

// ultrasonic vars
bool OBSTACLE_DETECTION = false;
long US_duration;
int US_distance;
float SPEED_OF_SOUND = 0.034; // cm/s
float OBSTACLE_THRESHOLD = 57.0; // cm

// controller vars
const int ADC_Max = 1023; // 10 bit



void setup() {
  Serial.begin(9600);

  // init time
  currTime = millis();

  // set power modifiers to 0
  FL_POWER_MOD = 0.0;
  FR_POWER_MOD = 0.0;
  BL_POWER_MOD = 0.0;
  BR_POWER_MOD = 0.0;

  // make sure to use a PWM capable pin
  esc_FL.attach(MOTOR_FL);  
  esc_FR.attach(MOTOR_FR);
  esc_BL.attach(MOTOR_BL);
  esc_BR.attach(MOTOR_BR);

  initEncoders();

  pinMode(LED, OUTPUT);
  pinMode(US_Trig, OUTPUT);
  pinMode(US_Echo, INPUT);

  /*
  // Transmitter init
  if(!driver.init()){
    Serial.println("Transmitter Init. Failed.");
  }
  */

  //pinMode(DPAD_UP, INPUT_PULLUP);
  //pinMode(DPAD_DOWN, INPUT_PULLUP);
  //pinMode(BUTTON_X, INPUT_PULLUP);
  //XInput.setAutoSend(false); // wait for all controls before sending
  //XInput.begin();

}

void loop() {
  if(UGV_WAS_TAGGED){
    allMotorsOff();
    // SEND TAGGED IRC MESSAGE
    playTaggedSequence();
  }else if(UGV_WAS_KILLED){
    allMotorsOff();
    exit(0);
  }else{
    // CHECK IF TAGGED
    goStraight(DISTANCE_TO_GO, totalDistTraveled, 20);

    // updates
    updateTime(); // update time vars and deltaT
    totalDistTraveled += getAvgSpeed() * deltaT; // // update distance traveled
    if(OBSTACLE_DETECTION){
      avoidObstacles();
    }

    //boolean buttonX = !digitalRead(BUTTON_X);
    //boolean dpadUp = !digitalRead(DPAD_UP);
    //boolean dpadDown = !digitalRead(DPAD_DOWN);

    //XInput.setButton(BUTTON_X, buttonX);
    //XInput.setDpad(dpadUp, dpadDown);    

    //updateDashboard();
  }
}

void avoidObstacles(){
  float obstruction_distance = get_obstruction_distance();
  if(obstruction_distance <= OBSTACLE_THRESHOLD){
    //turn right
    //obstacle2dash();
  }
}

/*
void obstacle2dash(){
  const char * obsTag = "Obstacle Detected!"
  driver.send((uint8_t*)obsTag, strlen(obsTag));
  driver.waitPacketSent();
}
*/

float get_obstruction_distance(){
  // clear output of US sensor briefly
  digitalWrite(US_Trig, LOW);
  delayMicroseconds(2);

  // generate ultrasound wave
  digitalWrite(US_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_Trig, LOW);

  // init wave reading vars
  US_duration = pulseIn(US_Echo, HIGH);
  US_distance = US_duration*SPEED_OF_SOUND/2;

  return US_distance;  
}

void playTaggedSequence() {
  digitalWrite(LED, HIGH);
}

void updateTime(){
  prevTime = currTime; 
  currTime = millis();
  deltaT = (currTime - prevTime)/1000;
}
/*
void updateDashboard(){
  const char * dashTag = "UGV Tag Status: " + char(UGV_WAS_TAGGED);
  /*const chat\
  
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
  dtostrf(resistance, 6, 2, result);
  String(UGV_WAS_TAGGED);
  const char * killTag = "UGV Kill Status: " + String(UGV_WAS_KILLED);
  const char * dashSpeedHeader = "Speed:\n";
  const char * frontSpeeds = "FL: " + String(MOTOR_FL_SPEED.currentSpeed) + "FR: " + String(MOTOR_FR_SPEED.currentSpeed);
  const char * backSpeeds = "BL: " + String(MOTOR_BL_SPEED.currentSpeed) + "BR: " + String(MOTOR_BR_SPEED.currentSpeed);

  driver.send((uint8_t*)dashTag, strlen(dashTag));
  driver.waitPacketSent();

  driver.send((uint8_t*)killTag, strlen(killTag));
  driver.waitPacketSent();

  driver.send((uint8_t*)dashSpeedHeader, strlen(dashSpeedHeader));
  driver.waitPacketSent();

  driver.send((uint8_t*)frontSpeeds, strlen(frontSpeeds));
  driver.waitPacketSent();

  driver.send((uint8_t*)backSpeeds, strlen(backSpeeds));
  driver.waitPacketSent();
}
*/


void goStraight(float dToGo, float dTraveled, float speed){
  if(dTraveled < dToGo){
    //power = speed2power(speed);
    float power = 20.0;
    allMotorsConstant(power);
    checkWheelsMatchSpeed(speed);
  }
}

void allMotorsConstant(float power){
  // sets all motors to a given power value
  set_esc_power(esc_FL, power+FL_POWER_MOD);
  set_esc_power(esc_FR, -(power+FR_POWER_MOD));
  set_esc_power(esc_BL, power+BL_POWER_MOD);
  set_esc_power(esc_BR, -(power+BR_POWER_MOD));
}

void allMotorsOff(){
  // sets all motors to a given power value
  set_esc_power(esc_FL, 0);
  set_esc_power(esc_FR, 0);
  set_esc_power(esc_BL, 0);
  set_esc_power(esc_BR, 0);
}

void set_esc_power(Servo esc, int power){
  // set one motor to a given power value
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
}

////////// SPEED FUNCTIONS //////////

/*
float speed2power(float speed){
  // convert speed to power
  return power
}
*/

void checkWheelsMatchSpeed(float speedWanted){
  // Sends command to correct wheels
  correctSpeedFL(speedWanted);
  correctSpeedFR(speedWanted);
  correctSpeedBL(speedWanted);
  correctSpeedBR(speedWanted);
}

void correctSpeedFL(float speedWanted){
  if((MOTOR_FL_SPEED.currentSpeed - speedWanted) > WHEEL_SPEED_TOL){
    FL_POWER_MOD -= 1;
  }else if((MOTOR_FL_SPEED.currentSpeed - speedWanted) < WHEEL_SPEED_TOL){
    FL_POWER_MOD += 1;
  }
  // update to dash that wheel was corrected?
}

void correctSpeedFR(float speedWanted){
  if((MOTOR_FR_SPEED.currentSpeed + speedWanted) < WHEEL_SPEED_TOL){
    FR_POWER_MOD -= 1;
  }else if((MOTOR_FR_SPEED.currentSpeed + speedWanted) > WHEEL_SPEED_TOL){
    FR_POWER_MOD += 1;
  }
  // update to dash that wheel was corrected?
}

void correctSpeedBL(float speedWanted){
  if((MOTOR_BL_SPEED.currentSpeed - speedWanted) > WHEEL_SPEED_TOL){
    BL_POWER_MOD -= 1;
  }else if((MOTOR_BL_SPEED.currentSpeed - speedWanted) < WHEEL_SPEED_TOL){
    BL_POWER_MOD += 1;
  }
  // update to dash that wheel was corrected?
}
void correctSpeedBR(float speedWanted){
  if((MOTOR_BR_SPEED.currentSpeed + speedWanted) < WHEEL_SPEED_TOL){
    BR_POWER_MOD -= 1;
  }else if((MOTOR_BR_SPEED.currentSpeed + speedWanted) > WHEEL_SPEED_TOL){
    BR_POWER_MOD += 1;
  }
  // update to dash that wheel was corrected?
}

float getAvgSpeed(){
  float avg = MOTOR_FL_SPEED.currentSpeed;
  avg += MOTOR_FR_SPEED.currentSpeed;
  avg += MOTOR_BL_SPEED.currentSpeed;
  avg += MOTOR_BR_SPEED.currentSpeed;
  return avg/4;
}

////////// ENCODER FUNCTIONS //////////

void initEncoders(){
  // Initialize the encoders. Set struct pins, set pin modes,
  // set initial speeds, attach interrupts.

  // Rotary Encoder Inputs - must be 18, 19, 20, or 21 on MEGA for ISR enabled pins  
  MOTOR_FL_SPEED.encoderPin = ENCODER_FL;
  MOTOR_FR_SPEED.encoderPin = ENCODER_FR;
  MOTOR_BL_SPEED.encoderPin = ENCODER_BL;
  MOTOR_BR_SPEED.encoderPin = ENCODER_BR;

  // Set encoder pins as inputs
  pinMode(MOTOR_FL_SPEED.encoderPin, INPUT);
  pinMode(MOTOR_FR_SPEED.encoderPin, INPUT);
  pinMode(MOTOR_BL_SPEED.encoderPin, INPUT);
  pinMode(MOTOR_BR_SPEED.encoderPin, INPUT);

  // Read the initial state of encoder1
  MOTOR_FL_SPEED.lastEncoderState = digitalRead(MOTOR_FL_SPEED.encoderPin);
  MOTOR_FR_SPEED.lastEncoderState = digitalRead(MOTOR_FR_SPEED.encoderPin);
  MOTOR_BL_SPEED.lastEncoderState = digitalRead(MOTOR_BL_SPEED.encoderPin);
  MOTOR_BR_SPEED.lastEncoderState = digitalRead(MOTOR_BR_SPEED.encoderPin);

  // Call updateEncoder when any high/low changed seen
  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_SPEED.encoderPin), updateEncoderFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_SPEED.encoderPin), updateEncoderFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_SPEED.encoderPin), updateEncoderBL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_SPEED.encoderPin), updateEncoderBR, CHANGE);
}

void updateEncoderFL(){
  // call helper to update global vars outside ISR
  MOTOR_FL_SPEED = updateEncoderHelper(MOTOR_FL_SPEED);
}

void updateEncoderFR(){
  // call helper to update global vars outside ISR
  MOTOR_FR_SPEED = updateEncoderHelper(MOTOR_FR_SPEED);  
}

void updateEncoderBL(){
  // call helper to update global vars outside ISR
  MOTOR_BL_SPEED = updateEncoderHelper(MOTOR_BL_SPEED);
}

void updateEncoderBR(){
  // call helper to update global vars outside ISR
  MOTOR_BR_SPEED = updateEncoderHelper(MOTOR_BR_SPEED);
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
  return SCR;
}

