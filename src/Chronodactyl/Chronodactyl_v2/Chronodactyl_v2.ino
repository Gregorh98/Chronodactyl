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

int period = 2000;

bool bootGripActive = true;
unsigned long bootGripStart = 0;


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

  bootGripStart = millis();
  hand.grip();
}

unsigned long lastAction = 0;
int action = 0;
bool quarterActive = false;
unsigned long quarterStart = 0;
int lastQuarterMinute = -1;
bool quarterPreGrip = false;
unsigned long quarterPreGripStart = 0;


void loop() {
  unsigned long now = millis();
  DateTime ts_now = rtc.now();

  int minuteVal = ts_now.minute();
  int hourVal = ts_now.hour();

  if (bootGripActive) {
    if (now - bootGripStart >= period) {
      bootGripActive = false;
      hand.release();  // open after holding 1 second
    }

    hand.update();
    return;  // Skip normal logic until boot animation is done
  }


  // DST adjust if needed
  if (digitalRead(dstSwitch) == LOW) {
    hourVal += 1;
  }

  // ---- Trigger quarter animation ----
  if (digitalRead(animSwitch) == LOW && !quarterActive && minuteVal % 15 == 0 && minuteVal != lastQuarterMinute) {

    quarterActive = true;
    quarterStart = now;
    lastQuarterMinute = minuteVal;

    // Clear hand first
    hand.release();

    if (minuteVal == 15) {
      hand.show_15();
    } else if (minuteVal == 30) {
      hand.show_30();
    } else if (minuteVal == 45) {
      hand.show_45();
    } else if (minuteVal == 0) {
      hand.show_0();
    }
  }

  // ---- If animation active ----
  if (quarterActive) {
    if (now - quarterStart >= 3000) {
      quarterActive = false;
    }
  } else {
    // Normal time display (using seconds like before)
    if (now - lastAction > period) {
      lastAction = now;
      hand.asBinary(hourVal);
    }
  }

  hand.update();
}
