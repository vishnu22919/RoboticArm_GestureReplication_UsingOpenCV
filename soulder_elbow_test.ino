
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Two PCA boards
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);  // Shoulder PCA
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);  // Elbow PCA

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

void loop() {
  moveShoulderRange(35, 90, 9);    
  delay(500);

  moveElbowRange(35, 90, 10);
  delay(500);

  moveElbowRange(90, 35, 10);
  delay(1000);
  
  moveShoulderRange(90, 35, 9);   
  delay(1000);

  moveElbowRange(35, 90, 10);
  delay(500);

  moveElbowRange(90, 35, 10);
  delay(1000);
}




