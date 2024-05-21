#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>
#include <SparkFun_TB6612.h>

#define DEBUG 0

#define AIN1 7
#define AIN2 8
#define PWMA 6
#define BIN1 10
#define BIN2 9
#define PWMB 11
#define SPEED 100

#define MAX_SERVO_COUNT 5

// TODO: 完善註解，上傳github
/*M,N,O,P,Q
  m,n,o,p,q*/
  /*  i
    j k l*/
const unsigned short servoPin[5] = { 0,1,2,3,4 };
const short servoAngleMin[5] = { -90,-90,-90,-90,-90 };
const short servoAngleMax[5] = { 90,90,90,90,90 };
const short servoAngleInit[5] = { 0 };


PCA9685 ServoController;
// PCA9685 輸出 = 12 位 = 4096 步
// 20ms 的 2.5% = 0.5ms ; 20ms 的 12.5% = 2.5ms
// 4096 的 2.5% = 102 步；4096 的 12.5% = 512 步
PCA9685_ServoEval pwmServo[5]; // (0deg, 90deg, 180deg)
// Initializing motors.
Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, -1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, -1);

bool isIncreasing = false;
bool isDecreasing = false;
bool isMotorRunning = false;
char currentCommand = '\0';
short servoPos[5] = { 0 };


void setup() {
  Serial.begin(9600);
  Wire.begin();
  ServoController.resetDevices(); // Software resets all PCA9685 devices on Wire line
  ServoController.init(); // Address pins A5-A0 set to B000000
  ServoController.setPWMFreqServo();  //  Set frequency to 50Hz

  for (int i = 0;i < MAX_SERVO_COUNT;i++) {
    ServoController.setChannelPWM(servoPin[i], pwmServo[i].pwmForAngle(servoAngleInit[i]));
    servoPos[i] = servoAngleInit[i];
    delay(10);
  }
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    // Comman M ~ Q,servo 1 ~ 5 to rise
    if (input >= 'M' && input <= 'Q') {
      if (currentCommand == input) {
        isIncreasing = !isIncreasing; // 增加的flag復位
        currentCommand = '\0';  //  命令暫存初始化
      }
      else if (currentCommand == '\0') {
        isIncreasing = true;
        isDecreasing = false; // 確保不會減少
        isMotorRunning = false; //  確保馬達不會轉動
        currentCommand = input;
      }
    }

    // Comman m ~ q,servo 1 ~ 5 to down
    if (input >= 'm' && input <= 'q') {
      if (currentCommand == input) {
        isDecreasing = !isDecreasing; //  減少的flag復位
        currentCommand = '\0';  //  命令暫存初始化
      }
      else if (currentCommand == '\0') {
        isDecreasing = true;
        isIncreasing = false; // 確保不會增加
        isMotorRunning = false; //  確保馬達不會轉動
        currentCommand = input;
      }
    }

    if (input >= 'I' && input <= 'L') {
      if (currentCommand == input) {
        brake(motor1, motor2);
#if DEBUG
        Serial.println("brake");
#endif
        currentCommand = '\0';  //  命令暫存初始化
      }
      else if (currentCommand == '\0') {
        isMotorRunning = true;
        isIncreasing = false; // 確保不會增加
        isDecreasing = false; // 確保不會減少
        currentCommand = input;
      }
    }
  }

  if (isIncreasing && currentCommand >= 'M' && currentCommand <= 'Q') {
    int index = currentCommand - 'M';
    servoPos[index]++;
    if (servoPos[index] > servoAngleMax[index]) {
      servoPos[index] = servoAngleMax[index];
      isIncreasing = !isIncreasing;
      currentCommand = '\0';
    }

    ServoController.setChannelPWM(index, pwmServo->pwmForAngle(servoPos[index]));
    delay(10);
#if DEBUG
    Serial.println(index);
    Serial.println(servoPos[index]);
#endif
  }

  if (isDecreasing && currentCommand >= 'm' && currentCommand <= 'q') {
    int index = currentCommand - 'm';
    servoPos[index]--;
    if (servoPos[index] < servoAngleMin[index]) {
      servoPos[index] = servoAngleMin[index];
      isDecreasing = !isDecreasing;
      currentCommand = '\0';
    }
    ServoController.setChannelPWM(index, pwmServo->pwmForAngle(servoPos[index]));
    delay(10);
#if DEBUG
    Serial.println(index);
    Serial.println(servoPos[index]);
#endif
  }

  if (isMotorRunning && currentCommand >= 'I' && currentCommand <= 'L') {
    switch (currentCommand)
    {
    case 'I':
      motor1.drive(SPEED);
      motor2.drive(SPEED);

#if DEBUG
      Serial.println("forward");
#endif
      break;

    case 'J':
      motor1.drive(-SPEED);
      motor2.drive(SPEED);

#if DEBUG
      Serial.println("left");
#endif
      break;

    case 'K':
      motor1.drive(-SPEED);
      motor2.drive(-SPEED);
#if DEBUG
      Serial.println("back");
#endif
      break;

    case 'L':
      motor1.drive(SPEED);
      motor2.drive(-SPEED);
#if DEBUG
      Serial.println("right");
#endif
    default:
      break;
    }
    isMotorRunning = !isMotorRunning;
  }
}