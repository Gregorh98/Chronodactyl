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

  const int tolerance = 1;  // angle tolerance to prevent float jitter restarts

public:
  Finger(int p, int cd, int od, bool i = false, int period = 1000) {
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

    moveDuration = period;
    moving = false;
  }

  uint16_t asServoMap(int val) {
    if (inverted)
      val = servo_max - val;

    return map(val, servo_min, servo_max, SERVOMIN, SERVOMAX);
  }

  float easeInOut(float t) {
    if (t < 0.5)
      return 4 * t * t * t;
    else
      return 1 - pow(-2 * t + 2, 3) / 2;
  }

  void startMove(int newTarget) {

    // Already at target and not moving
    if (!moving && abs(currentAngle - newTarget) <= tolerance)
      return;

    // Already moving to same target
    if (moving && targetAngle == newTarget)
      return;

    startAngle = currentAngle;
    targetAngle = newTarget;
    moveStartTime = millis();
    moving = true;
  }

  void update() {
    if (!moving) {
      pwm.setPWM(pin, 0, 0);
      return;
    }

    unsigned long now = millis();
    float elapsed = now - moveStartTime;
    float t = (float)elapsed / moveDuration;

    if (t >= 1.0f) {
      currentAngle = targetAngle;
      pwm.setPWM(pin, 0, asServoMap(currentAngle));
      moving = false;
      return;
    }

    float eased = easeInOut(t);
    currentAngle = startAngle + (targetAngle - startAngle) * eased;
    pwm.setPWM(pin, 0, asServoMap(currentAngle));
  }

  void extend() {
    if (extended) return;
    extended = true;
    startMove(move_min);
  }

  void retract() {
    if (!extended) return;
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
