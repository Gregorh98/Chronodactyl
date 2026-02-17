#ifndef FINGER_H
#define FINGER_H

#define SERVOMIN 100
#define SERVOMAX 650

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);


#include <Arduino.h>

class Finger {
private:
  int pin;
  int servo_min;
  int servo_max;
  int move_min;
  int move_max;
  bool inverted;
  bool extended;

  int startAngle;
  int targetAngle;
  int currentAngle;

  unsigned long moveStartTime;
  unsigned long moveDuration;  // total move time in ms
  bool moving;

public:
  Finger(int p, int cd, int od, bool i = false) {
    pin = p;
    move_min = cd;
    move_max = od;
    servo_min = 0;
    servo_max = 180;
    inverted = i;
    extended = true;

    currentAngle = move_min;
    targetAngle = move_min;
    startAngle = move_min;

    moveDuration = 2500;  // adjust for speed (ms)
    moving = false;
  }

  uint16_t asServoMap(int val) {
    if (inverted)
      val = servo_max - val;

    return map(val, servo_min, servo_max, SERVOMIN, SERVOMAX);
  }

  float easeInOut(float t) {
    if (t < 0.5)
      return 4 * t * t * t;  // slow start
    else
      return 1 - pow(-2 * t + 2, 3) / 2;  // slow finish
  }

  void startMove(int newTarget) {
    startAngle = currentAngle;
    targetAngle = newTarget;
    moveStartTime = millis();
    moving = true;
  }

  void update() {
    if (!moving) return;

    unsigned long now = millis();
    float elapsed = now - moveStartTime;

    float t = elapsed / moveDuration;
    if (t >= 1.0) {
      moving = false;
      pwm.setPWM(pin, 0, 0);
    }

    float eased = easeInOut(t);
    Serial.println(eased);

    currentAngle = startAngle + (targetAngle - startAngle) * eased;

    if (moving) {
      pwm.setPWM(pin, 0, asServoMap(currentAngle));
    }
  }


  void extend() {
    extended = true;
    startMove(move_min);
  }

  void retract() {
    extended = false;
    startMove(move_max);
  }

  bool isMoving() {
    return moving;
  }

  void setDuration(unsigned long durationMs) {
    moveDuration = durationMs;
  }
};

#endif
