#include "hand.h"

void Hand::init()
{
    _controller.begin();
    _controller.setPWMFreq(60);
}

void Hand::update()
{
    _thumb.update();
    _thumb_knuckle.update();
    _index.update();
    _middle.update();
    _ring.update();
    _pinky.update();
}

void Hand::release()
{
  _thumb.extend();
  _thumb_knuckle.extend();
  _index.extend();
  _middle.extend();
  _ring.extend();
  _pinky.extend();
}

void Hand::grip()
{
  _thumb.retract();
  _thumb_knuckle.retract();
  _index.retract();
  _middle.retract();
  _ring.retract();
  _pinky.retract();
}

void Hand::show_binary(uint8_t num) {
  if (num > 31) {
    release();
    return;
  }

  // Reverse bit order (pinky becomes LSB)
  if (num & 0b00001) _pinky.extend(); else _pinky.retract();
  if (num & 0b00010) _ring.extend();  else _ring.retract();
  if (num & 0b00100) _middle.extend(); else _middle.retract();
  if (num & 0b01000) _index.extend(); else _index.retract();
  if (num & 0b10000) _thumb.extend(); else _thumb.retract();

  // Keep thumb knuckle linked to thumb
  if (num & 0b10000) _thumb_knuckle.extend(); else _thumb_knuckle.retract();
}

void Hand::show_animation(uint8_t minute_val)
{
  switch (minute_val / 15)
  {
  case 0:
    // On Hour
    release();
    break;
  case 1:
    // Quarter past
    _index.extend();
    _thumb_knuckle.retract();
    _thumb.retract();
    _middle.retract();
    _ring.retract();
    _pinky.retract();
    break;
  case 2:
    // Half past
    _index.extend();
    _middle.extend();
    _thumb_knuckle.retract();
    _thumb.retract();
    _ring.retract();
    _pinky.retract();
    break;
  case 3:
    // Quarter to
    _index.extend();
    _middle.extend();
    _ring.extend();
    _thumb_knuckle.retract();
    _thumb.retract();
    _pinky.retract();
    break;
  default:
    break;
  }
}
