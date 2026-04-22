#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RTClib.h>
#include "components/virtual/hand/hand.h"

// PCA9685 setup
Adafruit_PWMServoDriver servo_controller = Adafruit_PWMServoDriver(0x40);
RTC_DS3231 rtc;

#define SERVOMIN  100
#define SERVOMAX  650

int16_t period = 2000;

const int animSwitch = 11;
const int dstSwitch = 12;

bool lastAnimSwitchState = HIGH;

Hand hand = Hand(servo_controller);

// ================= Gestures =================

void grip() {
  hand.grip();
}

void release() {
  hand.release();
}

// ================= Setup =================

void setup() {
  Serial.begin(115200);
  Serial.println("Chronodactyl RTC Start");

  pinMode(animSwitch, INPUT_PULLUP);
  pinMode(dstSwitch, INPUT_PULLUP);

  servo_controller.begin();
  servo_controller.setPWMFreq(60);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Re-enable to set RTC time
  // Serial.println("Setting RTC time");
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


  lastAnimSwitchState = digitalRead(animSwitch);
}



// ================= Time Logic =================

void updateTime()
{
  DateTime now = rtc.now();

  int hourVal = now.hour();

  if (digitalRead(dstSwitch) == LOW) {  // switch ON
    hourVal = (hourVal + 1) % 24;
  }

  int minuteVal = now.minute();
  int secondVal = now.second();

  char timeBuffer[20];
  sprintf(timeBuffer, "Local time: %02d:%02d:%02d", hourVal, minuteVal, secondVal);
  Serial.println(timeBuffer);

  int timesToRun = minuteVal / 15;

  hand.show_binary(hourVal);
  delay(1000);
}

// ================= Loop =================

void loop() {
  bool currentAnimSwitchState = digitalRead(animSwitch);

  // Detect switch turned ON
  if (lastAnimSwitchState == HIGH && currentAnimSwitchState == LOW) {
    Serial.println("Animations turned ON, playing startup animation");
    // Play a quick "blink" animation to show it's active
    for (int i = 0; i < 2; i++) {
      release();
      delay(500);
      grip();
      delay(500);
    }
  }

  lastAnimSwitchState = currentAnimSwitchState; // update state

  updateTime();
}
