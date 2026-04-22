
#include "finger.h"
#include <Arduino.h>


void Finger::update()
{
    switch (_state)
    {
    case IDLE:
        _servo_controller.setPWM(_pin, 0, 0);
        break;
    case MOVING:
        if (abs(_current_deg - _target_deg) <= _tolerance) {
            _state = IDLE;
        }
        else {
            // Calculate the next position based on elapsed time and ease towards the target
            _current_deg = _eased_next_position();
            _servo_controller.setPWM(_pin, 0, as_servo_map(_current_deg));
        }
        break;
    default:
        break;
    }
}



uint8_t Finger::_eased_next_position()
{
    unsigned long now = millis();
    unsigned long elapsed = now - _move_start_time;

    if (elapsed >= (unsigned long)_move_period_ms) {
        return _target_deg;
    }
    
    float t = (float)elapsed / (float)_move_period_ms;
    float ease = (1 - cos(t * 3.14159265f)) / 2.0f;
    int delta = (int)_target_deg - (int)_start_deg;
    
    return (uint8_t)((int)_start_deg + (int)(delta * ease));
}

void Finger::start_move(uint8_t target_deg)
{
    // Ignore subsequent calls if already moving to this target
    if (_state == MOVING && _target_deg == target_deg)
        return;

    _start_deg = _current_deg;
    _target_deg = target_deg;
    _move_start_time = millis();
    _state = MOVING;
}

uint16_t Finger::as_servo_map(int val)
{
    return map(_is_inverted ? _servo_val_max - val : val, _servo_val_min, _servo_val_max, _servo_val_min, _servo_val_max);
}

void Finger::extend()
{
    start_move(as_servo_map(_move_min_deg));
}

void Finger::retract()
{
    start_move(as_servo_map(_move_max_deg));
}