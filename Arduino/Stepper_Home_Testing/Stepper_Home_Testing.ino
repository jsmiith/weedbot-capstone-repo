#include <AccelStepper.h>  //Steppper Library
#include <SerialCommand.h> //Serial Parsing library
#include <ezButton.h>

#define dirPin 9
#define stepPin 8
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

#define rightLimitSwitch 19
#define leftLimitSwitch 18
#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type)

ezButton lsRight(rightLimitSwitch);
ezButton lsLeft(leftLimitSwitch);

boolean homingInProgress = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1600);
  stepper.setSpeed(-1000);
  lsLeft.setDebounceTime(50);
  lsRight.setDebounceTime(50);

  homeStepper();//home before setting up interrupts

  Serial.println("Setting up interrupts.");
  pinMode(rightLimitSwitch, INPUT_PULLUP);//for debounce
  digitalWrite(rightLimitSwitch, HIGH);
  attachInterrupt(digitalPinToInterrupt(rightLimitSwitch), rightLimitReached, CHANGE); 

  pinMode(leftLimitSwitch, INPUT_PULLUP);//for debounce
  digitalWrite(leftLimitSwitch, HIGH);
  attachInterrupt(digitalPinToInterrupt(leftLimitSwitch), leftLimitReached, CHANGE); 

  //homeStepper();
  Serial.println("Interrupts setup.");
  Serial.println("Entering loop.");
}

void loop() {
//homeStepper();
//Serial.println("in loop");
//elay(10000);
  /*
  stepper.setMaxSpeed(2800);
  stepper.setAcceleration(1600);

  stepper.moveTo(-15000);
  stepper.runToPosition();
  Serial.println("Moving left");
  delay(1000);

  
  stepper.moveTo(15000);
  stepper.runToPosition();
  Serial.println("Moving right");  
  delay(1000);
  stepper.moveTo(0);
  stepper.runToPosition();
  Serial.println("Moving right");  
  delay(1000);
  */

}

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
  stepper.move(100);
  stepper.runToPosition();
  stepper.setCurrentPosition(0);

  
/*
  
  long startTime = millis();
  while ((millis()-startTime)<8000);{

  }*/
  Serial.println("Homing Complete");
}

void rightLimitReached(){
  Serial.println("Right Limit Hit");
  //stepper.stop();
  //delay(500);
  //stepper.move(1000);
  //stepper.run()
  //stepper.runToPosition();
  delay(1000);
  stepper.setCurrentPosition(0);
  homingInProgress=false;
}

void leftLimitReached(){
  Serial.println("Left Limit Hit");
  stepper.stop();
  stepper.move(-40);
  delay(1000);
}
