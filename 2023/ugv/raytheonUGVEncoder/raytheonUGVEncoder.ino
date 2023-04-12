struct speedCheckReturn{
  double currentSpeed;
  double distanceTraveled;
  unsigned long totalSteps;       // total steps triggered - important for persistent odometry
  unsigned long stepTimes[3];     // array for tracking step times - must be declared externally so it is persistent trigger to trigger
  unsigned int stepTimesIndex;        // tracks the index of the time tracking array for speed calculations
  unsigned int lastEncoderState;  // records previous encoder state
  unsigned int encoderPin;        // pin number for encoder
};

//// 
double wheelDiameter = 0.13;    // wheel diameter in meters
double stepsPerRotation = 20.0; // number of slots in encoder wheel
String units = "imperial";        // select "imperial", "metric", or "rotational"

speedCheckReturn MOTOR_FL_SPEED;  // speedCheckReturn init
speedCheckReturn MOTOR_FR_SPEED;
speedCheckReturn MOTOR_BL_SPEED;
speedCheckReturn MOTOR_BR_SPEED;

unsigned int stepTimesLength = sizeof(MOTOR_FL_SPEED.stepTimes)/sizeof(MOTOR_FL_SPEED.stepTimes[0]); // length of stepTimes array
// IMPORTANT: stepTimesLength determines number of rotations speed will be averaged over as (stepTimesLength/stepsPerRotation)

void setup() {
  Serial.begin(9600);  // Setup Serial Monitor

  MOTOR_FL_SPEED.encoderPin = 18;  // Rotary Encoder Inputs
  MOTOR_FR_SPEED.encoderPin = 19;
  MOTOR_BL_SPEED.encoderPin = 20;
  MOTOR_BR_SPEED.encoderPin = 21;
  
  pinMode(MOTOR_FL_SPEED.encoderPin,INPUT);  // Set encoder pins as inputs
  
  MOTOR_FL_SPEED.lastEncoderState = digitalRead(MOTOR_FL_SPEED.encoderPin);  // Read the initial state of encoder1

  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_SPEED.encoderPin), updateEncoderFL, CHANGE);  // Call speedCheck1() when any high/low changed seen
}


void loop() {
  //Do some useful stuff here
  MOTOR_FL_SPEED = speedCheck(MOTOR_FL_SPEED);
  //Serial.println(MOTOR_FL_SPEED.stepTimesIndex);
  //Serial.print("Speed: ");Serial.println(MOTOR_FL_SPEED.currentSpeed);Serial.print("Odo: ");Serial.print(MOTOR_FL_SPEED.distanceTraveled);Serial.println("\n\n");
  Serial.print(MOTOR_FL_SPEED.currentSpeed); Serial.print("|||");Serial.print(MOTOR_FL_SPEED.stepTimesIndex); Serial.print("|||");Serial.println(MOTOR_FL_SPEED.stepTimes[MOTOR_FL_SPEED.stepTimesIndex]);  
  //Serial.println();Serial.println(MOTOR_FL_SPEED.stepTimesIndex); Serial.println();
}


void updateEncoderFL(){
  MOTOR_FL_SPEED = updateEncoderHelper(MOTOR_FL_SPEED);
}
speedCheckReturn updateEncoderHelper(speedCheckReturn UE){
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
