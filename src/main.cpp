#include <Arduino.h>

const int irPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};  
// A-> left   and  B-> right
const int ENA = 3;  
const int ENB = 6;  
const int IN1 = 2;   
const int IN2 = 4;   
const int IN3 = 5;   
const int IN4 = 7;   

float Kp = 0.07;    // 0.072, 0.08
float Kd = 0.0135;    //0.01, 0.012
float Ki = 0;   

float P, I, D;
float error = 0;
float previousError = 0;
float PIDvalue = 0;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int baseSpeed = 200;  //90
int threshold[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int minValues[8];
int maxValues[8];
char turn = 's';
int all_white = 0;
int turn_speed = 90;


void calibrate() {
  for (int i = 0; i < 8; i++) {
    minValues[i] = analogRead(irPins[i]);
    maxValues[i] = analogRead(irPins[i]);
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) { // Run for 10 seconds
//  LEFT
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 90);

// RIGHT
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 90);

    for (int i = 0; i < 8; i++) {
      int sensorValue = analogRead(irPins[i]);
      if (sensorValue < minValues[i]) minValues[i] = sensorValue;
      if (sensorValue > maxValues[i]) maxValues[i] = sensorValue;
    }
  }

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues [i]) / 2;
    threshold[i] += 35;
  }

}

int readSensorPosition() {
  int sensorValues[8];
  int sum = 0;
  int weightedSum = 0;
  
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]) > threshold[i] ? 1 : 0; 
    if (sensorValues[i] == 1) { 
      sum += 1;
      weightedSum += i * 1000; 
    }
  }
  
  if (sum == 0) {
    // line gayab 
    all_white = 1;
    return error > 0 ? 7000 : 0; 
  }
  
  int position = weightedSum / sum;
  all_white = 0;
  if (position < 2000) {
    turn = 'l';  // left corner
  } else if (position > 5000) {
    turn = 'r';  // right corner
  } else {
    turn = 's';  // near the center (likely a dead end / U-turn situation)
  }
  return position;
}

void calculatePID(int position) {
  error = position - 3500;
  if (abs(error) > 2500) {
    error = error > 0 ? 2500 : -2500; 
  }
  
  P = error;
  I = I + error;
  D = error - previousError;

  I = constrain(I, -1*baseSpeed, baseSpeed);
  
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  
  previousError = error;
}

void motor_control() {
  leftMotorSpeed = baseSpeed - PIDvalue;
  rightMotorSpeed = baseSpeed + PIDvalue;
  
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
  
  digitalWrite(IN2, leftMotorSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN1, leftMotorSpeed > 0 ? LOW : HIGH);
  analogWrite(ENA, abs(leftMotorSpeed));
  
  digitalWrite(IN3, rightMotorSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightMotorSpeed > 0 ? LOW : HIGH);
  analogWrite(ENB, abs(rightMotorSpeed));
}


void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, turn_speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, turn_speed);
  
  delay(180);

}

void right() {
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, turn_speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, turn_speed);
  
  
  delay(180); // 210

}

// void straight() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   analogWrite(ENA, 100);
//   digitalWrite(IN3, HIGH);
//   digitalWrite(IN4, LOW);
//   analogWrite(ENB, 100);
//   delay(100);
// }

void back() {
 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, turn_speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, turn_speed);
  
  delay(400);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  Serial.begin(9600);
 
}

void loop() {

  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  delay(1000);
  
  while(digitalRead(12)){}
  while(1){
    int position = readSensorPosition();
    if(all_white == 1){
      // stop();
      // delay(100);
      if(turn == 'l'){
        delay(250);
        left();
        while(all_white == 0);
        turn = 's';
      } else if(turn == 'r'){
        delay(250);
        right();
        while(all_white == 0); 
        turn = 's';
      } else {
        delay(250);
        back();
        while(all_white == 0); 
        turn = 's';
      }
      // delay(50);
      // turn = 's';
    } else {
      // pid_motor_control();
      calculatePID(position);
      motor_control();
    }
    // int position = readSensorPosition();
    // if(all_white == 1){
    //   stop();
    //   delay(100);
    //   if(turn == 'l'){
    //     left();
    //   } else if(turn == 'r'){
    //     right(); 
    //   } else {
    //     back();
    //   }
    //   delay(50);
    //   // turn = 's';
    // } else {
    //   // pid_motor_control();
    //   calculatePID(position);
    //   motor_control();
    // }
    
  }


}
