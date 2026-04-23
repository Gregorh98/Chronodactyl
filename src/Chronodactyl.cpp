#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RTClib.h>
#include "components/abstractions/hand/hand.h"
#include "components/hardware/switch/switch.h"
#include "config.h"

// Initialisation Variables
long start_time;

// Hardware Instances
RTC_DS3231 _rtc;
Switch animation_switch = Switch(PIN_SWITCH_ANIMATION);
Switch dst_switch = Switch(PIN_SWITCH_DST);

// Abstractions
Hand hand = Hand(SERVO_CONTROLLER_ADDRESS);

// ================= Functions =================
void update_time()
{
  // Every hour on the hour, update the hand to show the current time in binary (5 bits for hours, 0-23)
  DateTime now = _rtc.now();
  Serial.print("Current time: ");
  Serial.println(now.timestamp());

  int8_t hour_value = now.hour();

  if (dst_switch.current_state) {
    hour_value = (hour_value + 1) % 24;
  }

  int8_t minute_value = now.minute();
  int8_t second_value = now.second();
}


// ================= Setup =================
void setup() {
  Serial.begin(115200);
  Serial.println("Chronodactyl RTC Start");
  start_time = millis();

  dst_switch.init();
  animation_switch.init();
  hand.init();

  while (!_rtc.begin()) {
    Serial.println("RTC not found, retrying...");
    delay(1000);
  }

  if(RESET_RTC) {
    Serial.println("Resetting RTC time to compile time");
    _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

// ================= Loop =================
void loop() {
  // On first run, grip all fingers to set a known starting position
  if (millis() - start_time <= FINGER_MOVE_PERIOD_MS) {
    hand.grip();
    hand.update();
    return;
  }

  // animation_switch.update();
  dst_switch.update();
  update_time();
  hand.update();
}
