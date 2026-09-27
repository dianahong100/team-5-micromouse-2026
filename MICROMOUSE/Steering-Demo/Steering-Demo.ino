#include <Wire.h>
#include <VL53L4CD.h>
#include <Adafruit_BNO055.h>
#include <NewPing.h>
#include <EnableInterrupt.h>


// ---------- Motor driver ----------
const uint8_t PWMA = 3, AIN1 = 5, AIN2 = 4; // right motor
const uint8_t PWMB = 9, BIN1 = 7, BIN2 = 8; // left motor
const uint8_t STBY = 6;


// ---------- Encoders ----------
const uint8_t R_C1 = 2, R_C2 = 10;   // right motor
const uint8_t L_C1 = 11, L_C2 = 12;  // left motor


volatile long leftCount = 0, rightCount = 0;


// ---------- Ultrasonics ----------
const uint8_t R_TRIG = A0, R_ECHO = A1; // right ultrasonic
const uint8_t L_TRIG = A2, L_ECHO = A3; // left ultrasonic


NewPing sonarLeft (L_TRIG, L_ECHO, 100);
NewPing sonarRight (R_TRIG, R_ECHO, 100);


// ---------- I2C sensors ----------
VL53L4CD tof;
Adafruit_BNO055 bno(55, 0x28);


// ---------- Bang-bang settings ----------
const int CRUISE_SPEED = 110;
const int TURN_SPEED = 130;
const int BANG_AMOUNT = 50;


const float FRONT_STOP_CM = 8.0;
const float SIDE_CLOSE_CM = 8.0;


const int TURN_MS = 450;


// ---------- Encoder ISRs ----------
void leftISR()
{
  bool a = digitalRead(L_C1);
  bool b = digitalRead(L_C2);


  if (a != b)
    leftCount += 1;
  else
    leftCount += -1;
}
void rightISR()
{
  bool a = digitalRead(R_C1);
  bool b = digitalRead(R_C2);


  if (a != b)
    rightCount += -1;
  else
    rightCount += 1;
}


// ---------- Motor Functions ----------


void setRight(int speed)
{
  speed = -speed;
  bool fwd = speed >=0;
  int mag = constrain(abs(speed), 0, 255);
  if (fwd)
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
  else
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
    analogWrite(PWMA, mag);
}


void setLeft(int speed)
{
  bool fwd = speed >=0;
  int mag = constrain(abs(speed), 0, 255);
  if (fwd)
    {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
  else
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMB, mag);
}


void stopMotors()
{
  setLeft(0);
  setRight(0);
}


void forward(int speed)
{
  setLeft(speed);
  setRight(speed);
}


void backward(int speed)
{
  setLeft(-speed);
  setRight(-speed);
}


void pivotLeft(int speed)
{
  setLeft(-speed);
  setRight(speed);
}


void pivotRight(int speed)
{
  setLeft(speed);
  setRight(-speed);
}


void steerRight(int speed)
{
  setLeft(speed + BANG_AMOUNT);
  setRight(speed - BANG_AMOUNT);
}


void steerLeft(int speed)
{
  setLeft(speed - BANG_AMOUNT);
  setRight(speed + BANG_AMOUNT);
}


void turnLeftTimed()
{
  pivotLeft(TURN_SPEED);
  delay(TURN_MS);
  stopMotors();
}


void turnRightTimed()
{
  pivotRight(TURN_SPEED);
  delay(TURN_MS);
  stopMotors();
}


float readFrontCM()
{
  float cm = tof.read() / 10.0;
  if (tof.timeoutOccurred())
    return -1;
  return cm;
}


float readLeftCM()
{
  unsigned int cm = sonarLeft.ping_cm();
  if (cm == 0)
    return -1;
  return cm;
}


float readRightCM()
{
  unsigned int cm = sonarRight.ping_cm();
  if (cm == 0)
    return -1;
  return cm;
}


bool wallAhead()
{
  float d1 = readFrontCM();
  delay(60);
  float d2 = readFrontCM();
  return (d1 > 0 && d2 > 0 && d1 < FRONT_STOP_CM && d2 < FRONT_STOP_CM);
}


bool wallLeft()
{
  float d = readLeftCM();
  return (d > 0 && d < SIDE_CLOSE_CM);
}


bool wallRight()
{
  float d = readRightCM();
  return (d > 0 && d < SIDE_CLOSE_CM);
}


void setup()
{
  Serial.begin(115200);
  delay(2000);


  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();


  pinMode(L_C1, INPUT_PULLUP); pinMode(L_C2, INPUT_PULLUP);
  pinMode(R_C1, INPUT_PULLUP); pinMode(R_C2, INPUT_PULLUP);
  enableInterrupt(L_C1, leftISR,  CHANGE);
  enableInterrupt(R_C1, rightISR, CHANGE);


  Wire.begin();
  Wire.setClock(400000);
  delay(500);


  tof.setTimeout(500);
  if (tof.init())
  {
    tof.setRangeTiming(50,0);
    tof.startContinuous();
  }
  else
  {
    Serial.println("TOF Sensor not found");
  }


  if (bno.begin())
  {
    delay(1000);
    bno.setExtCrystalUse(true);
  }
  else
  {
    Serial.println("IMU Sensor not found");
  }


  Serial.println("Starting algorithm in 3 seconds.");
  delay(3000);
}


void loop()
{
  bool blockedAhead = wallAhead();
  bool closeLeft = wallLeft();
  delay(30);
  bool closeRight = wallRight();


  if (blockedAhead)
  {
    Serial.println("WALL AHEAD");
    stopMotors();
    delay(100);


    if (closeLeft && closeRight)
    {
      Serial.println("BOXED IN, TURNING AROUND");
      turnLeftTimed();
      turnLeftTimed();
    }
    else if (closeLeft)
    {
      Serial.println("TURNING RIGHT");
      turnRightTimed();
    }
    else
    {
      Serial.println("TURNING LEFT");
      turnLeftTimed();
    }


    delay(100);
    readFrontCM();
    delay(100);
    readFrontCM();
  }
  else if (closeLeft)
  {
    Serial.println("WALL CLOSE LEFT --> STEER RIGHT");
    steerRight(CRUISE_SPEED);
  }
  else if (closeRight)
  {
    Serial.println("WALL CLOSE RIGHT --> STEER LEFT");
    steerLeft(CRUISE_SPEED);
  }
  else
  {
    Serial.println("CLEAR PATH --> GOING FORWARD");
    forward(CRUISE_SPEED);
  }
  delay(20);
}
