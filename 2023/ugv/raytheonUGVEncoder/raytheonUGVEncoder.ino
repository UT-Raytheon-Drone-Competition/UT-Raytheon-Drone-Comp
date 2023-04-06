// overal setup constants
const int digitalEncoderPin = 2;
const int analogEncoderPin = 2;

bool previousEncoderState = 0;
bool currentEncoderState = 0;

// variables used in program
unsigned int stepTimes[2];

unsigned int totalSteps = 0;
int speedIndex = 0; // tracks the index of the time tracking array for speed calculations
int stepTimesLength = sizeof(stepTimes)/sizeof(stepTimes[0]);
double deltaT;

const double wheelDiameter = 0.13;
double travelPerRotation = wheelDiameter*PI;  // circumference of wheel 

double rotations = 0.0;
double currentSpeed = 0.0;
double currentRPS = 0.0;
double metersTraveled = 0.0;
double totalStepsForCalcs = 0.0;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(digitalEncoderPin, INPUT); // set pin as digital in for encoder
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  currentEncoderState = digitalRead(digitalEncoderPin);

  if(currentEncoderState != previousEncoderState){
    stepDetected();
    previousEncoderState = currentEncoderState;
  }
  
}

void stepDetected() {

totalSteps++;
    
totalStepsForCalcs = totalSteps;    // converts steps to double for calculations
rotations = totalStepsForCalcs/40;  // finds the number of rotations completed based on the number of encoder state changes that should occur per rotation
metersTraveled = rotations*travelPerRotation;

stepTimes[speedIndex] = millis();

if (speedIndex == (stepTimesLength-1)){   // if at end of array, cycles back to pos 0
  deltaT = (stepTimes[speedIndex] - stepTimes[0]); // finds time difference (milliseconds) of (stepTimesLength) steps ago
  speedIndex = 0;
}
else{
  deltaT = (stepTimes[speedIndex] - stepTimes[speedIndex+1]); 
  speedIndex++;
}

currentRPS = (stepTimesLength/40.0)/(deltaT/1000.0);   // finds current rotations per second
currentSpeed = currentRPS*travelPerRotation;  // finds speed (m/s) of the vehicle from time difference of 1 rotation ago

Serial.println(metersTraveled);
Serial.println(currentSpeed);   // current speed in m/s
Serial.println();
}
