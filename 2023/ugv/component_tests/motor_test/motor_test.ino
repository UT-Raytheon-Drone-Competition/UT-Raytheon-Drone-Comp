#include <Servo.h>

#define MOTOR_FL 9
#define MOTOR_FR 8
#define MOTOR_BL 11
#define MOTOR_BR 10

Servo esc_FL;  // create servo object to control the PWM signal
Servo esc_FR;
Servo esc_BL;
Servo esc_BR;

void setup() {
  esc_FL.attach(MOTOR_FL);  // make sure to use a PWM capable pin
  esc_FR.attach(MOTOR_FR);
  esc_BL.attach(MOTOR_BL);
  esc_BR.attach(MOTOR_BR);
}

void loop(){  
  //TEST ONE AT A TIME INC AND DEC
  //single_motors_test();
  
  // TEST ALL WHEELS TOGETHER
  //all_increasing_test();

  // TEST ALL AT CONSTANT SPEED
  float power = 17.0;
  all_constant_test(power);
}

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
  set_esc_power(esc_FR, -(power+2));
  set_esc_power(esc_BL, power);
  set_esc_power(esc_BR, -(power+2));
  delay(numSeconds*1000);
}

void all_increasing_test(){
  for(int pow = 0; pow <= 10; pow++){
    set_esc_power(esc_FL, 10*pow);
    set_esc_power(esc_FR, -10*pow);
    set_esc_power(esc_BL, 10*pow);
    set_esc_power(esc_BR, -10*pow);
    delay(500);
  }  
}

void single_motors_test(){
  // test each motor going up and down in power.
  // does one motor at a time.
  motor_test(esc_FL);
  motor_test(esc_FR);
  motor_test(esc_BL);
  motor_test(esc_BR);
}

void motor_test(Servo esc){
  // power a motor up and down in power.
  set_esc_power(esc, 0);  
  delay(500) ;
  set_esc_power(esc, 20);
  delay(500) ;
  set_esc_power(esc, 40);
  delay(500) ;
  set_esc_power(esc, 60);
  delay(500) ;
  set_esc_power(esc, 80);
  delay(500) ;
  set_esc_power(esc, 100);
  delay(500) ;
  set_esc_power(esc, 80);
  delay(500) ;
  set_esc_power(esc, 60);
  delay(500) ;
  set_esc_power(esc, 40);
  delay(500) ;
  set_esc_power(esc, 20);
  delay(500) ;
  set_esc_power(esc, 0);
  delay(500) ;
  set_esc_power(esc, -20);
  delay(500) ;
  set_esc_power(esc, -40);
  delay(500) ;
  set_esc_power(esc, -60);
  delay(500) ;
  set_esc_power(esc, -80);
  delay(500) ;
  set_esc_power(esc, -100);
  delay(500) ;
  set_esc_power(esc, -80);
  delay(500) ;
  set_esc_power(esc, -60);
  delay(500) ;
  set_esc_power(esc, -40);
  delay(500) ;
  set_esc_power(esc, -20);
}
