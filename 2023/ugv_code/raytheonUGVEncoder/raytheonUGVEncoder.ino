const int digitalEncoderPin = 2;
const int analogEncoderPin = 2;

bool previousEncoderState = 0;
bool currentEncoderState = 0;

int totalSteps = 0;
int speedIndex = 0; // tracks the index of the time tracking array for speed calculations
int stepTimes[20];

double travelPerRotation = 0.013*PI;  // circumference of wheel

double rotations = 0.0;
double currentSpeed = 0.0;
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

if (speedIndex = 19){   // if at end of array, cycles back to pos 0
  currentSpeed = (travelPerRotation/2)/((stepTimes[speedIndex] - stepTimes[0])/1000); // finds speed (m/s) of the vehicle from time difference of 1/2 turn ago
  speedIndex = 0;
}
else{
  currentSpeed = (travelPerRotation/2)/((stepTimes[speedIndex] - stepTimes[speedIndex+1])/1000); // finds speed (m/s) of the vehicle from time difference of 1/2 turn ago
  speedIndex++;
}
  
}
