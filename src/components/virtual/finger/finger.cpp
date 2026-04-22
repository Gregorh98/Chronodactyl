#include "finger.h"

uint16_t Finger::as_servo_map(int val)
{
    if (_inverted)
    val = _servo_max - val;

    return map(val, _servo_min, _servo_max, _servo_min, _servo_max);
}

void Finger::extend()
{
    if (!_extended)
    {
    _servo_controller.setPWM(_pin, 0, as_servo_map(_move_min));
    _extended = true;
    }
    else
    {
    _servo_controller.setPWM(_pin, 0, 0);
    }
}

void Finger::retract()
{
    if (_extended)
    {
    _servo_controller.setPWM(_pin, 0, as_servo_map(_move_max));
    _extended = false;
    }
    else
    {
    _servo_controller.setPWM(_pin, 0, 0);
    }
}