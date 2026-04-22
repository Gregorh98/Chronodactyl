#include <Adafruit_PWMServoDriver.h>

class Finger
{
  private:
    Adafruit_PWMServoDriver _servo_controller;
    int8_t _pin;
    int16_t _servo_min;
    int16_t _servo_max;
    int16_t _move_min;
    int16_t _move_max;
    bool _inverted;
    bool _extended;

  public:
    Finger(Adafruit_PWMServoDriver controller = Adafruit_PWMServoDriver(), int p = 0, int cd = 0, int od = 0, bool i = false) :
    _servo_controller(controller), _pin(p), _servo_min(0), _servo_max(180), _move_min(cd), _move_max(od), _inverted(i), _extended(true) {}

    uint16_t as_servo_map(int val);
    void extend();
    void retract();
};