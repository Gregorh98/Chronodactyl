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

int period = 1000;

Hand hand(period);

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

void loop() {
    unsigned long now = millis();

    DateTime ts_now = rtc.now();

    int hourVal = ts_now.hour();

    if (digitalRead(dstSwitch) == LOW)
    {
      hourVal += 1;
    }

    Serial.println(hourVal);

    if (now - lastAction > period) {
      lastAction = now;
      hand.asBinary(hourVal);
    }
    hand.update();
}
