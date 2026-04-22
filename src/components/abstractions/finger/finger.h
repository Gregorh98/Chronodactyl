# ifndef FINGER_H
# define FINGER_H

#include <Adafruit_PWMServoDriver.h>

class Finger
{
  private:
    Adafruit_PWMServoDriver& _servo_controller;
    int8_t _pin;
    int16_t _servo_val_min;
    int16_t _servo_val_max;
    uint8_t _move_min_deg;
    uint8_t _move_max_deg;
    bool _is_inverted;
    bool _is_extended;
    int16_t _move_period_ms;
    unsigned long _move_start_time; // Time when movement started

    uint8_t _start_deg;
    uint8_t _current_deg;
    uint8_t _target_deg;
    uint8_t _speed;
    uint8_t _tolerance = 1;

    enum FingerState {
      IDLE,
      MOVING
    };

    FingerState _state;

    uint8_t _eased_next_position();
    void start_move(uint8_t target_deg);

  public:
    Finger(Adafruit_PWMServoDriver& controller, int pin = 0, int move_min_deg = 0, int move_max_deg = 0, bool inverted = false, int16_t move_period_ms = 2000) :
    _servo_controller(controller),
    _pin(pin),
    _servo_val_min(0),
    _servo_val_max(180),
    _move_min_deg(move_min_deg),
    _move_max_deg(move_max_deg),
    _is_inverted(inverted),
    _is_extended(true),
    _move_period_ms(move_period_ms),
    _move_start_time(0),
    _start_deg(0),
    _current_deg(0),
    _target_deg(0),
    _speed(0),
    _tolerance(1),
    _state(IDLE)
    {}

    void update();

    uint16_t as_servo_map(int val);
    void extend();
    void retract();
    
};

# endif