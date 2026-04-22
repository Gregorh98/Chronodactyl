#ifndef HAND_H
#define HAND_H

#include <Adafruit_PWMServoDriver.h>
#include <components/abstractions/finger/finger.h>
#include <RTClib.h>

class Hand
{
  private:
    Adafruit_PWMServoDriver _controller;
    Finger _thumb;
    Finger _thumb_knuckle;
    Finger _index;
    Finger _middle;
    Finger _ring;
    Finger _pinky;
    int _move_period;

    void _grip();
    void _release();

  public:
    Hand(uint8_t controller_address) : _controller(controller_address), _thumb(_controller, 0, 0, 110, false, _move_period), _thumb_knuckle(_controller, 5, 0, 60, true, _move_period), _index(_controller, 3, 0, 180, false, _move_period), _middle(_controller, 1, 0, 180, false, _move_period), _ring(_controller, 4, 0, 180, true, _move_period), _pinky(_controller, 5, 0, 180, true, _move_period) {};

    void init();
    void update();
    
    void show_binary(uint8_t val);

};

#endif