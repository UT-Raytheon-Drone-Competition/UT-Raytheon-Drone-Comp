// overall setup constants
const int encoder1 = 18;    // encoder pin - must be interrupt capable (18,19,20,21 on mega)

// variables used in program

String units = "metric";        // select 'imperial' (MPH and feet) or 'metric' (mps and meters)
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

speedCheckReturn MOTOR_FL_SPEED;

void setup() {
  // put your setup code here, to run once:
  pinMode(encoder1, INPUT); // set pin as digital in for encoder
  Serial.begin(9600);
  MOTOR_FL_SPEED.totalSteps = 0;
  MOTOR_FL_SPEED.speedIndex = 0;
  MOTOR_FL_SPEED.units = "imperial"; 
  MOTOR_FL_SPEED.wheelDiameter = WD;
  MOTOR_FL_SPEED.stepsPerRotation = SPR;
}

void loop() {
  // put your main code here, to run repeatedly:
  attachInterrupt(digitalPinToInterrupt(encoder1), speedCheck1, CHANGE);
}

void speedCheck1(){
  MOTOR_FL_SPEED = speedCheck(MOTOR_FL_SPEED);
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
  }
  if (SCR.units == "imperial"){
    SCR.currentSpeed = currentRPS*travelPerRotation*2.23694;      // speed in miles per hour
    SCR.distanceTraveled = totalRotations*travelPerRotation*3.28084;  // distance traveled in feet
  }
  //
  return SCR;
}
