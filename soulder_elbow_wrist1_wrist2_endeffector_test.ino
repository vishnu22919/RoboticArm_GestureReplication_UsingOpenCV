#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Two PCA boards
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);  // Shoulder PCA
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);  // Elbow + Wrist + End-Effector PCA

#define SERVOMIN  150
#define SERVOMAX  600

void setup() {
  Serial.begin(9600);
  
  pca1.begin();
  pca1.setPWMFreq(60);
  
  pca2.begin();
  pca2.setPWMFreq(60);
}

// Converts degrees to PWM pulse count
uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Moves shoulder servos on pca1 (channels 0 and 1)
void moveShoulderRange(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      uint16_t pulse = angleToPulse(angle);
      uint16_t reversePulse = SERVOMAX + SERVOMIN - pulse;

      pca1.setPWM(0, 0, pulse);
      pca1.setPWM(1, 0, reversePulse);

      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      uint16_t pulse = angleToPulse(angle);
      uint16_t reversePulse = SERVOMAX + SERVOMIN - pulse;

      pca1.setPWM(0, 0, pulse);
      pca1.setPWM(1, 0, reversePulse);

      delay(stepDelay);
    }
  }
}

// Moves elbow servo on pca2 (channel 0)
void moveElbowRange(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(0, 0, pulse);
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(0, 0, pulse);
      delay(stepDelay);
    }
  }
}

// Moves wrist1 servo on pca2 (channel 1)
void moveWrist1Range(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(1, 0, pulse);
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(1, 0, pulse);
      delay(stepDelay);
    }
  }
}

// Moves wrist2 servo on pca2 (channel 2)
void moveWrist2Range(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(2, 0, pulse);
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(2, 0, pulse);
      delay(stepDelay);
    }
  }
}

// Moves end-effector servo on pca2 (channel 3)
void moveEndEffectorRange(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(3, 0, pulse);
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      uint16_t pulse = angleToPulse(angle);
      pca2.setPWM(3, 0, pulse);
      delay(stepDelay);
    }
  }
}

void loop() {
  moveShoulderRange(35, 80, 25);    
  delay(900);

  moveElbowRange(35, 90, 10);
  delay(900);

  moveWrist1Range(45, 100, 10);
  delay(900);

  moveWrist2Range(30, 90, 10);
  delay(900);

  moveEndEffectorRange(0, 120, 10);  // Open gripper (or rotate tool)
  delay(900);

  moveEndEffectorRange(120, 0, 10);  // Close gripper (or rotate tool back)
  delay(900);

  moveWrist2Range(90, 30, 10);
  delay(900);

  moveWrist1Range(100, 45, 10);
  delay(900);

  moveElbowRange(90, 35, 10);
  delay(900);

  moveShoulderRange(80, 35, 25);    
  delay(900);
}


  

  

