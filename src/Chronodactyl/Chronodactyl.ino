#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RTClib.h>

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
RTC_DS3231 rtc;

#define SERVOMIN  100
#define SERVOMAX  650

const int animSwitch = 11;
const int dstSwitch = 12;

bool lastAnimSwitchState = HIGH;

// ================= Finger Class =================

class Finger
{
  private:
    int pin;
    int servo_min;
    int servo_max;
    int move_min;
    int move_max;
    bool inverted;
    bool extended;

  public:
    Finger(int p, int cd, int od, bool i = false)
    {
      pin = p;
      move_min = cd;
      move_max = od;
      servo_min = 0;
      servo_max = 180;
      inverted = i;
      extended = true;
    }

    uint16_t asServoMap(int val)
    {
      if (inverted)
        val = servo_max - val;

      return map(val, servo_min, servo_max, SERVOMIN, SERVOMAX);
    }

    void extend()
    {
      if (!extended)
      {
        pwm.setPWM(pin, 0, asServoMap(move_min));
        extended = true;
      }
      else
      {
        pwm.setPWM(pin, 0, 0);
      }
    }

    void retract()
    {
      if (extended)
      {
        pwm.setPWM(pin, 0, asServoMap(move_max));
        extended = false;
      }
      else
      {
        pwm.setPWM(pin, 0, 0);
      }
    }
};

// ================= Fingers =================

Finger thumb_finger(0, 0, 110);
Finger thumb_knuckle(5, 0, 60, true);

Finger index_finger(3, 0, 180);
Finger middle_finger(1, 0, 180);
Finger ring_finger(4, 0, 180, true);
Finger pinky_finger(2, 0, 180, true);

// ================= Setup =================

void setup() {
  Serial.begin(115200);
  Serial.println("Chronodactyl RTC Start");

  pinMode(animSwitch, INPUT_PULLUP);
  pinMode(dstSwitch, INPUT_PULLUP);

  pwm.begin();
  pwm.setPWMFreq(60);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Re-enable to set RTC time
  // Serial.println("Setting RTC time");
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


  grip();
  delay(1000);
  release();
  delay(1000);
  gripping = false;

  lastAnimSwitchState = digitalRead(animSwitch);
}

// ================= Gestures =================

void grip() {
  index_finger.retract();
  middle_finger.retract();
  ring_finger.retract();
  pinky_finger.retract();
  thumb_finger.retract();
  thumb_knuckle.retract();
}

void release() {
  index_finger.extend();
  middle_finger.extend();
  ring_finger.extend();
  pinky_finger.extend();
  thumb_finger.extend();
  thumb_knuckle.extend();
}

void on_quarter(int q)
{
  grip();
  delay(1000);

  index_finger.extend();
  delay(500);

  if (q >= 2) {
    middle_finger.extend();
    delay(500);
  }

  if (q == 3) {
    ring_finger.extend();
    delay(500);
  }

  delay(1000);
}

void on_hour() {
  release();
  delay(1000);

  Finger fingerPairs[][2] = {
    {thumb_finger, thumb_knuckle},
    {index_finger, index_finger},
    {middle_finger, middle_finger},
    {ring_finger, ring_finger},
    {pinky_finger, pinky_finger}
  };

  for (int i = 0; i < 5; i++) {
    fingerPairs[i][0].retract();
    fingerPairs[i][1].retract();
    delay(200);
    fingerPairs[i][0].extend();
    fingerPairs[i][1].extend();
  }

  for (int i = 4; i >= 0; i--) {
    fingerPairs[i][0].retract();
    fingerPairs[i][1].retract();
    delay(200);
    fingerPairs[i][0].extend();
    fingerPairs[i][1].extend();
  }

  delay(1000);
}

void asBinary(int num) {
  if (num > 31) {
    release();
    return;
  }

  // Reverse bit order (pinky becomes LSB)
  if (num & 0b00001) pinky_finger.extend(); else pinky_finger.retract();
  if (num & 0b00010) ring_finger.extend();  else ring_finger.retract();
  if (num & 0b00100) middle_finger.extend(); else middle_finger.retract();
  if (num & 0b01000) index_finger.extend(); else index_finger.retract();
  if (num & 0b10000) thumb_finger.extend(); else thumb_finger.retract();

  // Keep thumb knuckle linked to thumb
  if (num & 0b10000) thumb_knuckle.extend(); else thumb_knuckle.retract();
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

  if (minuteVal % 15 == 0 && secondVal == 0 && digitalRead(animSwitch) == LOW)
  {
    if (timesToRun == 0)
      on_hour();
    else
      on_quarter(timesToRun);

    grip();
    delay(1000);
  }

  asBinary(hourVal);
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
