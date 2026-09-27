// Team 5 HTHS Micromouse Code

// include libraries
#include "Arduino.h"
#include <Wire.h>
#include <VL53L4CD.h>
#include <Adafruit_BNO055.h>
#include <NewPing.h>
#include <EnableInterrupt.h>

// pin constants
// --motor driver--
const uint8_t PWMA = 3, AIN1 = 5, AIN2 = 4; // right motor
const uint8_t PWMB = 9, BIN1 = 7, BIN2 = 8; // left motor
const uint8_t STBY = 6;

// encoders
const uint8_t L_C1 = 11, L_C2 = 12; // left motor
const uint8_t R_C1 = 2, R_C2 = 10; // right motor

// ultrasonics
const uint8_t L_TRIG = A2, L_ECHO = A3; // left ultrasonic
const uint8_t R_TRIG = A0, R_ECHO = A1; // right ultrasonic

volatile long leftCount = 0, rightCount = 0;
// count motor encoders as they spin


// ultrasonic objs
NewPing sonarLeft (L_TRIG, L_ECHO, 100);
NewPing sonarRight (R_TRIG, R_ECHO, 100);

// 12C sensors
VL53L4CD tof;
Adafruit_BNO055 bno(55, 0x28);


// encoder ISRs
// see if encoders equal or not
// if equal, subtract 1 (going backwords)
// if unequal, add 1 (going forwards)
void leftISR() {
  bool a = digitalRead(L_C1);
  bool b = digitalRead(L_C2);

  if (a != b)
    leftCount += 1;
  else
    leftCount += -1;

}

// flipped, so opposite for direction
void rightISR() {
  bool a = digitalRead(R_C1);
  bool b = digitalRead(R_C2);

  if (a != b)
    leftCount += -1;
  else
    leftCount += 1;
  
}


// motor functions
void setLeft(int speed) {
  bool fwd = speed >= 0; // true if speed above 0
  int mag = constrain(abs(speed), 0, 255);

  if (fwd) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, mag);
}

void setRight(int speed) {
  bool fwd = speed >= 0; // true if speed above 0
  int mag = constrain(abs(speed), 0, 255);

  if (fwd) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMB, mag);
}

void stopMotors() {
  setLeft(0);
  setRight(0);
}

void spinMotors(int speed) {
  setLeft(speed);
  setRight(speed);
}

// sensors
float readFrontCM() {
  float cm = tof.read() / 10.0; // distance
  if (tof.timeoutOccurred()) // if tof times out
    return -1;
  return cm;
}

float readLeftCM() {
  unsigned int cm = sonarLeft.ping_cm();
  // outputs zero if doesnt get correct reading back
  // might think touching a wall so must output -1 if 0
  if (cm == 0)
    return -1;
  return cm;
}

float readRightCM() {
  unsigned int cm = sonarRight.ping_cm();
  if (cm == 0)
    return -1;
  return cm;
}


void setup() {
  Serial.begin(115200);
  delay(2000);

  // motor drivers
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();

  // encoders
  pinMode(L_C1, INPUT_PULLUP);
  pinMode(L_C2, INPUT_PULLUP);
  pinMode(R_C1, INPUT_PULLUP);
  pinMode(R_C2, INPUT_PULLUP);
  enableInterrupt(L_C1, leftISR, CHANGE);
  enableInterrupt(R_C1, rightISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000);
  delay(500);

  tof.setTimeout(500);
  if (tof.init()){
    tof.setRangeTiming(50,0);
    tof.startContinuous();
  }
  else{
    Serial.println("TOF Sensor not found");
  }

  if (bno.begin()) {
    delay(1000);
    bno.setExtCrystalUse(true);
  }
  else {
    Serial.println("IMU Sensor not found");
  }

  Serial.println("Starting in 3 seconds");
  Serial.println("ms,front_cm,left_cm,right_cm,heading,gyroz,encL,encR");
  delay(3000);
}


void loop() {
  spinMotors(120);
  float front = readFrontCM();
  float left = readLeftCM();
  delay(10);
  float right = readRightCM();

  float heading = 0;
  float gyroz = 0;
  sensors_event_t e;
  bno.getEvent(&e);
  heading = e.orientation.x;
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyroz = g.z();

  noInterrupts();
  long lc = leftCount, rc = rightCount;
  interrupts();

  Serial.print(millis());   Serial.print(',');
  Serial.print(front, 1);   Serial.print(',');
  Serial.print(left, 1);   Serial.print(',');
  Serial.print(right, 1);   Serial.print(',');
  Serial.print(heading, 1);   Serial.print(',');
  Serial.print(gyroz, 2);   Serial.print(',');
  Serial.print(lc, 1);   Serial.print(',');
  Serial.println(rc);

  delay(20);
}