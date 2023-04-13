
#include <Servo.h>

void goStraight( float , float );
void allMotorsForward(float);
void set_esc_power(Servo , int);
float getAvgSpeed();

// GLOBAL VARIABLES
float eintegral = 0;
float deltaT = 0;
float totalDistTraveled = 0.0;

#define MOTOR_FL 9
#define MOTOR_FR 8
#define MOTOR_BL 11
#define MOTOR_BR 10

#define MOTOR_FL_SPEED 0;
#define MOTOR_FR_SPEED 0;
#define MOTOR_BL_SPEED 0;
#define MOTOR_BR_SPEED 0;

Servo esc_FL;   // create servo object to control the PWM signal
Servo esc_FR;
Servo esc_BL;
Servo esc_BR;


void setup() {
  float power = 15.0;
Serial.print("power");
}
void loop() {

  goStraight(50, .17); // distance to travel, MPH
  totalDistTraveled += getAvgSpeed() * deltaT; // define time passed var
Serial.print("goStraight");
}

float getAvgSpeed(){
  float avg = 0.0;

  avg += MOTOR_FL_SPEED;
  avg += MOTOR_FR_SPEED;
  avg += MOTOR_BL_SPEED;
  avg += MOTOR_BR_SPEED;
  avg /= 4;

  Serial.print(avg);
  return avg;

  
}

void allMotorsForward(float power){
  // move all motors
  int numSeconds = 10;
  set_esc_power(esc_FL, power);
  set_esc_power(esc_FR, -(power+2));
  set_esc_power(esc_BL, power);
  set_esc_power(esc_BR, -(power+2));
  delay(numSeconds*1000);
Serial.print("allMotorsForward");
}

void set_esc_power(Servo esc, int power){
  // Set given Servo object to given output power

  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
Serial.print("inside_esc_power");
}


void goStraight( float distanceToTravel, float speedWanted){
Serial.print("goStraight");  

  if(totalDistTraveled < distanceToTravel){
    // something that converts speed to power

    void allMotorsForward(float power);
    float avgSpeed = getAvgSpeed();
    Serial.print("allMotorsForward");
   


    if(avgSpeed != speedWanted){ // command to check each wheel speed and inc. or dec. if ahead or lagging

      float kp = 1;   //compute control signal u
      float ki = 1;  
      float e = speedWanted-avgSpeed; 
      eintegral += e*deltaT;

      float u = kp*e + ki*eintegral;
      Serial.print("avgSpeed");

      int dir = 1;
      if (u<0){
        dir = -1;
      Serial.print("dir");
       } 
      int power = (int) fabs(u);
      if (power > 100){
        power = 100;
      Serial.print("PID");
      }
      }


  }

}