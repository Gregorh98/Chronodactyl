#include "hand.h"

void Hand::update()
{
    _thumb.update();
    _thumb_knuckle.update();
    _index.update();
    _middle.update();
    _ring.update();
    _pinky.update();
}

void Hand::grip()
{
}

void Hand::release()
{
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
