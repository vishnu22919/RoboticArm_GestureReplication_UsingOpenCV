#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Min pulse length count (0 deg)
#define SERVOMAX  600 // Max pulse length count (180 deg)

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Standard servo frequency
}

// Converts degrees to PWM pulse count
uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Moves shoulder servos from startAngle to endAngle smoothly
void moveShoulderRange(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      uint16_t pulse = angleToPulse(angle);
      uint16_t reversePulse = SERVOMAX + SERVOMIN - pulse;

      pwm.setPWM(0, 0, pulse);
      pwm.setPWM(1, 0, reversePulse);

      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      uint16_t pulse = angleToPulse(angle);
      uint16_t reversePulse = SERVOMAX + SERVOMIN - pulse;

      pwm.setPWM(0, 0, pulse);
      pwm.setPWM(1, 0, reversePulse);

      delay(stepDelay);
    }
  }
}

void loop() {
  moveShoulderRange(35, 110, 10);    
  delay(500);
  
  moveShoulderRange(110, 35, 10);   
  delay(1000);
}



