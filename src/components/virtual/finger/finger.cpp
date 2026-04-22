#include "finger.h"

void Finger::update()
{
    switch (_state)
    {
    case IDLE:
        _servo_controller.setPWM(_pin, 0, 0);
        break;
    case MOVING:
        // Check to see if we have reached the target position within tolerance
        if (abs(_current_deg - _target_deg) <= _tolerance)
        {
            _state = IDLE;
            break;
        }

        // If not, calculate the next position based on speed and move towards the target
        _current_deg = _eased_next_position();
        _servo_controller.setPWM(_pin, 0, as_servo_map(_current_deg));
        
        break;    
    default:
        break;
    }
}


uint8_t Finger::_eased_next_position()
{
    return 0; // Placeholder for actual position calculation logic based on speed and time. 
}

uint16_t Finger::as_servo_map(int val)
{
    if (_is_inverted)
    val = _servo_val_max - val;

    return map(val, _servo_val_min, _servo_val_max, _servo_val_min, _servo_val_max);
}

void Finger::extend()
{
    if (!_is_extended)
    {
    _servo_controller.setPWM(_pin, 0, as_servo_map(_move_min_deg));
    _is_extended = true;
    }
    else
    {
    _servo_controller.setPWM(_pin, 0, 0);
    }
}

void Finger::retract()
{
    if (_is_extended)
    {
    _servo_controller.setPWM(_pin, 0, as_servo_map(_move_max_deg));
    _is_extended = false;
    }
    else
    {
    _servo_controller.setPWM(_pin, 0, 0);
    }
}