#include <AccelStepper.h>  //Steppper Library

#include <ezButton.h>


#define extendLimitSwitch 13
#define retractLimitSwitch 12

ezButton lsExtend(extendLimitSwitch);
ezButton lsRetract(retractLimitSwitch);

#define ENA 7 //yellow
#define EN1 10 //purple
#define EN2 11 //white


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(ENA, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);


  pinMode(extendLimitSwitch, INPUT_PULLUP);//for debounce
  digitalWrite(extendLimitSwitch, HIGH);
  pinMode(retractLimitSwitch, INPUT_PULLUP);//for debounce
  digitalWrite(retractLimitSwitch, HIGH);

  lsExtend.setDebounceTime(50);
  lsRetract.setDebounceTime(50);

  
}

void loop() {
  extend();
  delay(2000);
  retract();
  delay(2000);
/*
  lsExtend.loop();
  lsRetract.loop();
  Serial.println("Extend: " + (String)lsExtend.isPressed());
  Serial.println("Retract: " + (String)lsRetract.isPressed());
  */
}

//push ee into the ground, might want to add current feedback to slow down once in the ground
void extend(){
  Serial.println("extending...");
  lsExtend.loop();
  while(lsExtend.isPressed() == false){
    lsExtend.loop();
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, LOW);
    analogWrite(ENA, 200);
  }
  Serial.println("stopping...");
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  analogWrite(ENA, 0);
  delay(1000);

}
void retract(){
  Serial.println("retracting...");
  lsRetract.loop();
  while(lsRetract.isPressed() == false){
    lsRetract.loop();
    digitalWrite(EN1, LOW);
    digitalWrite(EN2, HIGH);
    analogWrite(ENA, 200);
  }
  Serial.println("stopping...");
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  analogWrite(ENA, 0);
  delay(1000);

}
