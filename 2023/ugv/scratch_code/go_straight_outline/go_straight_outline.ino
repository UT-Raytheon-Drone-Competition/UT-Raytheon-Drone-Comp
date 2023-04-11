float totalDistTraveled = 0.0;


void setup() {
  // put your setup code here, to run once:
  float power = 15.0;
}

void loop() {

  goStraight(dist1, speed1); // define vars later
  totalDistTraveled += getAvgSpeed() * time_passed; // define time passed var

}

float getAvgSpeed(){
  float avg = 0.0;

  avg += MOTOR_FL_SPEED;
  avg += MOTOR_FR_SPEED;
  avg += MOTOR_BL_SPEED;
  avg += MOTOR_BR_SPEED;

  return avg/4;
}

void allMotorsForward(power){
  // move all motors

}

void goStraight(distanceToTravel, speedWanted){

  if(totalDistTraveled < distanceToTravel){
    // something that converts speed to power
    allMotorsForward(power);
    avg = getAvgSpeed();

    if(avgSpeed != speedWanted){
      // command to check each wheel speed and inc. or dec. if ahead or lagging
    }

  }

}
