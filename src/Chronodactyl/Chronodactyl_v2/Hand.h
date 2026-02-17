#ifndef HAND_H
#define HAND_H

#include <Arduino.h>
#include "Finger.h"

class Hand {
public:
  Hand(int period)
    : thumb(0, 0, 110, period),
      thumb_knuckle(5, 0, 60, true, period),
      index(3, 0, 180, period),
      middle(1, 0, 180, period),
      ring(4, 0, 180, true, period),
      pinky(2, 0, 180, true, period) {}


  void update() {
    thumb.update();
    thumb_knuckle.update();
    index.update();
    middle.update();
    ring.update();
    pinky.update();
  }

  void grip() {
    index.retract();
    middle.retract();
    ring.retract();
    pinky.retract();
    thumb.retract();
    thumb_knuckle.retract();
  }

  void release() {
    index.extend();
    middle.extend();
    ring.extend();
    pinky.extend();
    thumb.extend();
    thumb_knuckle.extend();
  }

  void asBinary(int num) {
    if (num > 31) {
      release();
      return;
    }

    Serial.println(num & 0b00001);

    if (num & 0b00001) pinky.extend();
    else pinky.retract();
    if (num & 0b00010) ring.extend();
    else ring.retract();
    if (num & 0b00100) middle.extend();
    else middle.retract();
    if (num & 0b01000) index.extend();
    else index.retract();
    if (num & 0b10000) {
      thumb.extend();
      thumb_knuckle.extend();
    } else {
      thumb.retract();
      thumb_knuckle.retract();
    }
  }


private:
  Finger thumb;
  Finger thumb_knuckle;
  Finger index;
  Finger middle;
  Finger ring;
  Finger pinky;

  unsigned long waveLastUpdate = 0;
  int waveStep = 0;
  bool waveForward = true;
  const int waveInterval = 150;  // ms between steps
};

#endif
