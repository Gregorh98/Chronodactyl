#ifndef HAND_H
#define HAND_H

#include <Adafruit_PWMServoDriver.h>
#include <components/abstractions/finger/finger.h>
#include <RTClib.h>
#include "config.h"

class Hand
{
  private:
    Adafruit_PWMServoDriver _controller;
    int _move_period;
    
    Finger _thumb;
    Finger _thumb_knuckle;
    Finger _index;
    Finger _middle;
    Finger _ring;
    Finger _pinky;

  public:
    Hand(uint8_t controller_address) : _controller(controller_address), _move_period(FINGER_MOVE_PERIOD_MS), _thumb(_controller, PIN_SERVO_THUMB, 0, 110, false, _move_period), _thumb_knuckle(_controller, PIN_SERVO_THUMB_KNUCKLE, 0, 60, true, _move_period), _index(_controller, PIN_SERVO_INDEX, 0, 180, false, _move_period), _middle(_controller, PIN_SERVO_MIDDLE, 0, 180, false, _move_period), _ring(_controller, PIN_SERVO_RING, 0, 180, true, _move_period), _pinky(_controller, PIN_SERVO_PINKY, 0, 180, true, _move_period) {};

    void init();
    void update();

    void grip();
    void release();
    void show_binary(uint8_t val);
    void show_animation(uint8_t minute_val);

};

#endif