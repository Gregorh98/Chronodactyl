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

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1)
      ;  // Stop here if RTC not found
  }

  pwm.begin();
  pwm.setPWMFreq(60);
}

unsigned long lastAction = 0;
int action = 0;
const unsigned long interval = 1000;  // at least 2 seconds between actions

void loop() {
  unsigned long now = millis();

  DateTime ts_now = rtc.now();

  int hourVal = ts_now.hour();
  Serial.println(hourVal);

  if (now - lastAction > interval) {
    lastAction = now;
    hand.asBinary(hourVal);
  }
  hand.update();
}
