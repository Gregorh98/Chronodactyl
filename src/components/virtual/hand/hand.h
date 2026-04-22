#ifndef HAND_H
#define HAND_H

#include <Adafruit_PWMServoDriver.h>
#include <components/virtual/finger/finger.h>

class Hand
{
  private:
    Adafruit_PWMServoDriver& _controller;
    Finger _thumb;
    Finger _thumb_knuckle;
    Finger _index;
    Finger _middle;
    Finger _ring;
    Finger _pinky;
    int _move_period;

  public:
    Hand(Adafruit_PWMServoDriver& controller) : _controller(controller), _thumb(controller, 0, 0, 110, false, _move_period), _thumb_knuckle(controller, 5, 0, 60, true, _move_period), _index(controller, 3, 0, 180, false, _move_period), _middle(controller, 1, 0, 180, false, _move_period), _ring(controller, 4, 0, 180, true, _move_period), _pinky(controller, 5, 0, 180, true, _move_period) {};

    void update();

    void grip();
    void release();
    void show_binary(uint8_t val);

};

#endif