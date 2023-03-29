#include <ezButton.h>
#include <AccelStepper.h>

// EE Rotation - blue single channel (and single direction) 
#define EE_LPWM 44 // yellow PWM
#define EE_RPWM 46    // orange PWM
#define EE_LEN 48     // brown 
#define EE_REN 50     // white

// Drive Motors DO NOT CONNECT V+ 12V back voltage
#define DMR_ENA 6     // orange PWM
#define DML_ENB 7     // yellow PWM
#define DMR_IN1 14    // blue
#define DMR_IN2 15    // green
#define DML_IN3 12   // purple
#define DML_IN4 11   // white

// Aux Motors (LAV+ Punch) DO NOT CONNECT V+ 12V back voltage
#define LAV_ENA 4    // orange PWM
#define PUNCH_ENB 5  // yellow PWM
#define LAV_IN1 24  // blue
#define LAV_IN2 25  // green
#define PUNCH_IN3 23 // purple
#define PUNCH_IN4 22 // grey

int motorArray[4][3] ={{DMR_ENA, DMR_IN1, DMR_IN2},
                      {DML_ENB, DML_IN3, DML_IN4},
                      {LAV_ENA, LAV_IN1, LAV_IN2},
                      {PUNCH_ENB, PUNCH_IN3, PUNCH_IN4},};
enum motor {DMR, DML, LAV, PUNCH};
enum motorDriverPin {EN, IN1, IN2};
                     
// setup limit switches
#define extendLimitSwitch 13
#define retractLimitSwitch 12

#define rightLimitSwitch 19
#define leftLimitSwitch 18

ezButton lsExtend(extendLimitSwitch);
ezButton lsRetract(retractLimitSwitch);

ezButton lsRight(rightLimitSwitch);
ezButton lsLeft(leftLimitSwitch);

// setup stepper motor
#define dirPin 9
#define stepPin 8
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  // initialize arm limit switches
  pinMode(extendLimitSwitch, INPUT_PULLUP); // for debounce
  digitalWrite(extendLimitSwitch, HIGH);
  pinMode(retractLimitSwitch, INPUT_PULLUP); // for debounce
  digitalWrite(retractLimitSwitch, HIGH);
  lsExtend.setDebounceTime(50);
  lsRetract.setDebounceTime(50);

  /*
  // initialize stepper limit switches
  pinMode(rightLimitSwitch, INPUT_PULLUP); // for debounce
  digitalWrite(rightLimitSwitch, HIGH);
  pinMode(leftLimitSwitch, INPUT_PULLUP); // for debounce
  digitalWrite(leftLimitSwitch, HIGH);
  lsLeft.setDebounceTime(50);
  lsRight.setDebounceTime(50);

  // initialize stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1600);
  stepper.setSpeed(-1000);

  //home before setting up interrupts
  homeStepper();

  //setup stepper limit switch interrupts
  Serial.println("Setting up interrupts.");
  pinMode(rightLimitSwitch, INPUT_PULLUP);//for debounce
  digitalWrite(rightLimitSwitch, HIGH);
  attachInterrupt(digitalPinToInterrupt(rightLimitSwitch), rightLimitReached, CHANGE); 

  pinMode(leftLimitSwitch, INPUT_PULLUP);//for debounce
  digitalWrite(leftLimitSwitch, HIGH);
  attachInterrupt(digitalPinToInterrupt(leftLimitSwitch), leftLimitReached, CHANGE); 

  Serial.println("Interrupts setup.");
  Serial.println("Entering loop.");
*/
  motorsInit();
}

void loop() {
  //motorPositive(DMR, 90);
  digitalWrite(DMR)
}

// stepper homing routine, move arm right until right limit switch
void homeStepper(){
  Serial.println("Homing...");
  //stepper.setMaxSpeed(800);
  //stepper.setAcceleration(400);
  lsLeft.loop();
  lsRight.loop();
  while(lsRight.isPressed()==false){
    stepper.runSpeed();
    lsRight.loop();
  }

  stepper.stop();
  delay(1000);
  stepper.move(100);
  stepper.runToPosition();
  stepper.setCurrentPosition(0);

  Serial.println("Homing Complete");
}

// ISR for striking the right limit switch - safety feature for ball screw
void rightLimitReached(){
  Serial.println("Right Limit Hit");
  stepper.stop();
  delay(1000);
}

// ISR for striking the left limit - safety feature for ball screw
void leftLimitReached(){
  Serial.println("Left Limit Hit");
  stepper.stop();
  delay(1000);
}

// extend arm until limit reached - maybe add current feedback for when the ee enters the ground
void extendArm(){
  Serial.println("extending...");
  lsExtend.loop();
  while(lsExtend.isPressed() == false){
    lsExtend.loop();
    motorPositive(LAV, 90);
  }
  Serial.println("stopping...");
  motorStop(LAV);
  delay(1000);

}

// retract arm until arm is in the home/upright position (limit)
void retractArm(){
  Serial.println("retracting...");
  lsRetract.loop();
  while(lsRetract.isPressed() == false){
    lsRetract.loop();
    motorNegative(LAV, -90);
  }
  Serial.println("stopping...");
  motorStop(LAV);
  delay(1000);

}

// initialize motor pins
void motorsInit(){
  pinMode(DMR_ENA, OUTPUT);
  pinMode(DMR_IN1, OUTPUT);
  pinMode(DMR_IN2, OUTPUT);
  pinMode(DML_ENB, OUTPUT);
  pinMode(DML_IN3, OUTPUT);
  pinMode(DML_IN4, OUTPUT);

  pinMode(LAV_ENA, OUTPUT);
  pinMode(LAV_IN1, OUTPUT);
  pinMode(LAV_IN2, OUTPUT);
  pinMode(PUNCH_ENB, OUTPUT);
  pinMode(PUNCH_IN3, OUTPUT);
  pinMode(PUNCH_IN4, OUTPUT);

  pinMode(EE_LEN, OUTPUT);
  pinMode(EE_LPWM, OUTPUT);
  pinMode(EE_REN, OUTPUT);
  pinMode(EE_RPWM, OUTPUT);
}

// move motor forward or extend motor
void motorPositive(int motorNumb, int speed){
  int mappedSpeed = map(speed, 0, 100, 0, 250);

  digitalWrite(motorArray[motorNumb][IN2], LOW);
  digitalWrite(motorArray[motorNumb][IN1], HIGH);
  analogWrite(motorArray[motorNumb][EN], mappedSpeed);
}

// move motor backwards or retract motor
void motorNegative(int motorNumb, int speed){
  int mappedSpeed = map(speed, -100, 0, 0, 250);

  digitalWrite(motorArray[motorNumb][IN1], LOW);
  digitalWrite(motorArray[motorNumb][IN2], HIGH);
  analogWrite(motorArray[motorNumb][EN], mappedSpeed);
}

// break motor before changing directions
void motorStop(int motorNumb){
  digitalWrite(motorArray[motorNumb][IN1], LOW);
  digitalWrite(motorArray[motorNumb][IN2], LOW);
}

// move ee motor (unidirectional)
void blue_single_channel(){
  Serial.println("forward");
  // put your main code here, to run repeatedly:
  digitalWrite(EE_LEN,LOW);
  digitalWrite(EE_REN,HIGH);
  analogWrite(EE_LPWM, 200); //retract
  analogWrite(EE_RPWM, 200);
  delay(4000);

  Serial.println("reverse");
 digitalWrite(EE_LEN,HIGH);
  digitalWrite(EE_REN, LOW);
  analogWrite(EE_LPWM, 200); //retract
  analogWrite(EE_RPWM, 200);//send 0 before turing other pwm on
  delay(4000);
}

void pickWeed (int xLoc){
  stepper.moveTo(xLoc);
  while(stepper.distanceToGo()>0){
    stepper.run();
  }
  //turn EE on
  extendArm();
  //maybe add force/current feedback 
  retractArm();
  //turn off ee

  //if arm is closer to the left side then go to left to dispose
  //punch out weed

  // 

}

void adjustWeedLoc(int positionDifference){
  //positive position diference move forward, negative, move backwards
}



