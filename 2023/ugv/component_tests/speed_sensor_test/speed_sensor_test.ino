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
#define encoder_BR 18

// String units = "metric";        // select 'imperial' (MPH and feet) or 'metric' (mps and meters)
double WD = 0.13;  // diameter of wheel in m
int SPR = 20;      // number of openings in encoder wheel

struct speedCheckReturn{
  double currentSpeed;
  double distanceTraveled;
  unsigned long totalSteps;   // total steps triggered - important for persistent odometry
  unsigned long stepTimes[2];   // array for tracking step times - must be declared externally so it is persistent trigger to trigger
  unsigned int speedIndex;         // tracks the index of the time tracking array for speed calculations
  String units;
  double wheelDiameter;
  int stepsPerRotation;
};

speedCheckReturn MOTOR_BR_SPEED;
// define other encoders here


void setup() {

  // Motor
  esc_FL.attach(MOTOR_FL);  // make sure to use a PWM capable pin
  esc_FR.attach(MOTOR_FR);
  esc_BL.attach(MOTOR_BL);
  esc_BR.attach(MOTOR_BR);

  // encoder
  pinMode(encoder_BR, INPUT); // set pin as digital in for encoder
  MOTOR_BR_SPEED.totalSteps = 0;
  MOTOR_BR_SPEED.speedIndex = 0.0;
  MOTOR_BR_SPEED.units = "metric"; 
  MOTOR_BR_SPEED.wheelDiameter = WD;
  MOTOR_BR_SPEED.stepsPerRotation = SPR;

  // ISR for encoder
  attachInterrupt(digitalPinToInterrupt(encoder_BR), speedCheck1, CHANGE);
  
  Serial.begin(9600);
}

void loop() {
  //float power = 10.0;
  //all_constant_test(power);

  //set_esc_power(esc_BR, -power);
  
  for(int i = 1; i <= 10; i++){
    set_esc_power(esc_BR, 10*i);
    MOTOR_BR_SPEED = speedCheck(MOTOR_BR_SPEED);
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
void speedCheck1(){
  MOTOR_BR_SPEED = speedCheck(MOTOR_BR_SPEED);
}

speedCheckReturn speedCheck(speedCheckReturn SCR) {
  //
  double deltaT;
  double currentRPS;  
  double totalRotations;  // finds the number of rotations completed based on the number of encoder state changes that should occur per rotation
  double travelPerRotation = SCR.wheelDiameter*PI;
  int stepTimesLength = sizeof(SCR.stepTimes)/sizeof(SCR.stepTimes[0]); // length of stepTimes array
  int changesPerRotation = SCR.stepsPerRotation*2;  // number of rising and falling edges per rotation
  //
  
  //  
  SCR.totalSteps++;

  SCR.stepTimes[SCR.speedIndex] = millis();

  if (SCR.speedIndex == (stepTimesLength-1)){   // if at end of array, cycles back to pos 0
    deltaT = (SCR.stepTimes[SCR.speedIndex] - SCR.stepTimes[0]); // finds time difference (milliseconds) of (stepTimesLength) steps ago
    SCR.speedIndex = 0;
  }
  else{
    deltaT = (SCR.stepTimes[SCR.speedIndex] - SCR.stepTimes[SCR.speedIndex+1]); 
    SCR.speedIndex++;
  }
  
  totalRotations = SCR.totalSteps/changesPerRotation;
  currentRPS = (stepTimesLength/changesPerRotation)/(deltaT/1000.0);   // finds current rotations per second
  //
  
  //
  if (SCR.units == "metric"){
    SCR.currentSpeed = currentRPS*travelPerRotation;      // speed in meters/second
    SCR.distanceTraveled = totalRotations*travelPerRotation;  // distance traveled in meters
    //Serial.print(SCR.currentSpeed);
    //Serial.print();
    //Serial.println();    
  }
  if (SCR.units == "imperial"){
    SCR.currentSpeed = currentRPS*travelPerRotation*2.23694;      // speed in miles per hour
    SCR.distanceTraveled = totalRotations*travelPerRotation*3.28084;  // distance traveled in feet
    //Serial.print("in imperial");    
  }
  //
  return SCR;
}
