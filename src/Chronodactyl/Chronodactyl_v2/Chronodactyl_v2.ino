#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RTClib.h>
#include "Finger.h"
#include "Hand.h"


// PCA9685 setup
RTC_DS3231 rtc;

const int animSwitch = 11;
const int dstSwitch = 12;

bool lastAnimSwitchState = HIGH;
bool lastDstSwitchState = HIGH;

Hand hand;

void setup() {
  Serial.begin(115200);

  pinMode(animSwitch, INPUT_PULLUP);
  pinMode(dstSwitch, INPUT_PULLUP);

  pwm.begin();
  pwm.setPWMFreq(60);
}

unsigned long lastAction = 0;
int action = 0;
const unsigned long interval = 6000;  // at least 2 seconds between actions

void loop() {
  unsigned long now = millis();

  // Update all fingers every loop
  hand.update();

  // Toggle open/close every 2 seconds
  if (now - lastAction > interval) {
    lastAction = now;

    if (action == 0) {
      hand.grip();
      action++;
    } else if (action == 1) {
      hand.asBinary(12);
      action++;
    } else if (action == 2) {
      hand.release();
      action=0;
    }
  }
}
