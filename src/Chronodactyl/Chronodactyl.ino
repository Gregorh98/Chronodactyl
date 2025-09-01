#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>      
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Timezone.h>
#include "secrets.h"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC time, no offset here

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); 

#define SERVOMIN  100  // Min pulse length for 0° 
#define SERVOMAX  650  // Max pulse length for 180°

const int animSwitch = D7;  // Animation toggle switch

// UK Timezone with DST rules
TimeChangeRule BST = {"BST", Last, Sun, Mar, 1, 60};  // British Summer Time = UTC+1 hour
TimeChangeRule GMT = {"GMT", Last, Sun, Oct, 2, 0};   // Greenwich Mean Time = UTC+0 hour
Timezone UK(BST, GMT);

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
      extended=true;
    }

    uint16_t asServoMap(int val)
    {
      if (inverted) {
        val = servo_max - val;
      }

      uint16_t pulse = map(val, servo_min, servo_max, SERVOMIN, SERVOMAX);
      return pulse;
    }

    void extend()
    {
      if (!extended)
      {
        pwm.setPWM(pin, 0, asServoMap(move_min));
        extended = true;
      }
      else{
        pwm.setPWM(pin, 0, 0);
      }
    }

    void retract()
    {
      if (extended)
      {
        pwm.setPWM(pin, 0, asServoMap(move_max));
        extended=false;
      }
      else{
        pwm.setPWM(pin, 0, 0);
      }
    }
};

Finger thumb_finger(0, 0, 110);
Finger thumb_knuckle(5, 0, 60, true);

Finger index_finger(3, 0, 180);
Finger middle_finger(1, 0, 180);
Finger ring_finger(4, 0, 180, true);
Finger pinky_finger(2, 0, 180, true);

bool gripping;

void setup() {
    Serial.begin(115200);
    Serial.println("Chronodactly Start");

    pinMode(animSwitch, INPUT_PULLUP);

    pwm.begin();
    pwm.setPWMFreq(60);
    grip();
    delay(1000);
    release();
    delay(1000);
    gripping = false;

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");

    timeClient.begin();
    timeClient.update();
}

// Preset gestures
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

  if (q >= 2)
  {
    middle_finger.extend();
    delay(500);
  }
  
  if (q == 3)
  {
    ring_finger.extend();
    delay(500);
  }

  delay(1000);
}

void on_hour() {
    release();
    delay(1000);

    // Array of finger pairs (thumb moves together, others move individually)
    Finger fingerPairs[][2] = {
        {thumb_finger, thumb_knuckle},
        {index_finger, index_finger}, // Single fingers still need a second reference
        {middle_finger, middle_finger},
        {ring_finger, ring_finger},
        {pinky_finger, pinky_finger}
    };

    // Move fingers forward
    for (int i = 0; i < 5; i++) {
        fingerPairs[i][0].retract();
        fingerPairs[i][1].retract();
        delay(200);
        fingerPairs[i][0].extend();
        fingerPairs[i][1].extend();
    }

    // Move fingers backward
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
    
    // Bitmasking fingers
    if (num & 0b00001) thumb_finger.extend(); else thumb_finger.retract();
    if (num & 0b00001) thumb_knuckle.extend(); else thumb_knuckle.retract();
    if (num & 0b00010) index_finger.extend(); else index_finger.retract();
    if (num & 0b00100) middle_finger.extend(); else middle_finger.retract();
    if (num & 0b01000) ring_finger.extend(); else ring_finger.retract();
    if (num & 0b10000) pinky_finger.extend(); else pinky_finger.retract();
}

void time()
{
    timeClient.update();
    time_t utc = timeClient.getEpochTime();
    time_t local = UK.toLocal(utc);

    int localHour = hour(local);
    int localMinute = minute(local);
    int localSecond = second(local);

    Serial.printf("Local time: %02d:%02d:%02d\n", localHour, localMinute, localSecond);

    int timesToRun = localMinute / 15; // Determines how many times to run extend() and grip()

    if (localMinute % 15 == 0 && localSecond == 0 && digitalRead(animSwitch) == LOW)
    {
      if (timesToRun == 0)
      {
        // On the hour
        on_hour();
      }
      else
      {
        // On quarters of the hour
        on_quarter(timesToRun);
      }
      grip();
      delay(1000);
    }

    asBinary(localHour);
    delay(1000);
}

void debug() {
  grip();
  delay(5000);
  release();
  delay(5000);
  Serial.println("ANIMATION SWITCH: " + digitalRead(animSwitch));
}

void loop() {
  time();
  //debug();
}
