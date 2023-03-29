#include <ezButton.h>
#include <AccelStepper.h>


// EE Rotation - blue single channel (and single direction) 
#define LPWM 44 // yellow PWM
#define RPWM 46 // orange PWM
#define LEN 48  // brown 
#define REN 50  // white

// Drive Motors DO NOT CONNECT V+ 12V back voltage
#define ENA 6   // orange PWM
#define ENB 7   // yellow PWM
#define IN1 14  // blue
#define IN2 15  // green
#define IN3 12  // purple
#define IN4 11  // white

// Aux Motors (LAV+ Punch) DO NOT CONNECT V+ 12V back voltage
#define ENA 4   // orange PWM
#define ENB 5   // yellow PWM
#define IN1 24  // blue
#define IN2 25  // green
#define IN3 23  // purple
#define IN4 22  // grey



//yellow, red, grey, orange, green, purple, brown, blue 
int motorArray[2][3] ={{ENA, IN1, IN2},
                      {ENB, IN3, IN4},};

int motorNum = 0;
int speed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(REN, OUTPUT);

}

void loop() {
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 240);
  delay(2000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 240);
  delay(2000);
/*

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
  delay(2000);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
  delay(2000);

  
  if(speed == 0){
      motorStop(motorArray[motorNum][1], motorArray[motorNum][2]);
      speed = 80;
    }else if(speed == 1){
      motorStop(motorArray[motorNum][1], motorArray[motorNum][2]);
      speed = -80;
    }else if(speed < 0){
      motorNegative(motorArray[motorNum][0], motorArray[motorNum][1], motorArray[motorNum][2], speed);
      speed = 0;
    }else if(speed > 0){
      motorPositive(motorArray[motorNum][0], motorArray[motorNum][1], motorArray[motorNum][2], speed);
      speed = 1;
    } 
    delay(5000);
*/

  //blue_single_channel();
}






//move motor forward or extend motor
void motorPositive(int LENA, int EN1, int EN2, int speed){
  int mappedSpeed = map(speed, 0, 100, 0, 255);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, LOW);
  analogWrite(LENA, mappedSpeed);
}

//move motor forward or extend motor
void motorNegative(int LENA, int EN1, int EN2, int speed){
  int mappedSpeed = map(speed, -100, 0, 0, 255);

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, HIGH);
  analogWrite(LENA, mappedSpeed);
}

// break motor before changing directions
void motorStop(int EN1, int EN2){

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}



void blue_single_channel(){
  Serial.println("forward");
  // put your main code here, to run repeatedly:
  digitalWrite(LEN,LOW);
  digitalWrite(REN,HIGH);
  analogWrite(RPWM, 200); //retract
  analogWrite(LPWM, 200);
  delay(4000);

  Serial.println("reverse");
  digitalWrite(REN,LOW);
  digitalWrite(LEN,HIGH);
  analogWrite(RPWM, 200);
  analogWrite(LPWM, 200);//send 0 before turing other pwm on
  delay(4000);
}
