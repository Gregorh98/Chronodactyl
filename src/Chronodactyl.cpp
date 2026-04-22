#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RTClib.h>
#include "components/abstractions/hand/hand.h"
#include "components/hardware/switch/switch.h"

// Hardware Instances
RTC_DS3231 _rtc;
Switch animation_switch = Switch(11);
Switch dst_switch = Switch(12);

// Constants
#define SERVOMIN  100
#define SERVOMAX  650
#define SERVO_CONTROLLER_ADDRESS 0x40

// Abstractions
Hand hand = Hand(SERVO_CONTROLLER_ADDRESS);

// ================= Methods =================
void updateTime()
{
  DateTime now = _rtc.now();

  int hourVal = now.hour();

  if (dst_switch.current_state == LOW) {  // switch ON
    hourVal = (hourVal + 1) % 24;
  }

  //int minuteVal = now.minute(); // TODO: Use this for animations later on
  int secondVal = now.second();

  if (secondVal == 0) {
    hand.show_binary(hourVal);
  }
}


// ================= Setup =================
void setup() {
  Serial.begin(115200);
  Serial.println("Chronodactyl RTC Start");

  dst_switch.init();
  animation_switch.init();
  hand.init();

  while (!_rtc.begin()) {
    Serial.println("RTC not found, retrying...");
    delay(1000);
  }

  // Re-enable to set RTC time
  // Serial.println("Setting RTC time");
  // _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

// ================= Loop =================
void loop() {
  animation_switch.update();
  dst_switch.update();
  hand.update();
}
