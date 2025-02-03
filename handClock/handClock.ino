#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>        // Use <ESP8266WiFi.h> for ESP8266
#include <NTPClient.h>
#include <WiFiUdp.h>

const char* ssid     = "***";
const char* password = "****";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Adjust GMT offset


// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address

#define SERVOMIN  150  // Min pulse length for 0° 
#define SERVOMAX  600  // Max pulse length for 180°

// Servo channels on PCA9685
#define THUMB   0
#define INDEX   1
#define MIDDLE  2
#define RING    3
#define PINKY   4  // Inverted
#define KNUCKLE 5  // Inverted, 90°-180° only

class Finger
{
  private:
    int pin;
    int servo_min;
    int servo_max;
    int move_min;
    int move_max;
    bool inverted;

  public:
    Finger(int p, int cd, int od, bool i = false)
    {
      pin = p;
      move_min = cd;
      move_max = od;
      servo_min = 0;
      servo_max = 180;
      inverted = i;
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
      pwm.setPWM(pin, 0, asServoMap(move_min));
    }

    void retract()
    {
      pwm.setPWM(pin, 0, asServoMap(move_max));
    }
};

Finger thumb_finger(0, 60, 120);
Finger thumb_knuckle(5, 0, 120, true);

Finger index_finger(1, 0, 180);
Finger middle_finger(2, 0, 180);
Finger ring_finger(3, 0, 180);
Finger pinky_finger(4, 0, 180, true);

bool gripping;

void setup() {
    Serial.begin(115200);
    Serial.println("Robot Hand Preset Control");

    pwm.begin();
    pwm.setPWMFreq(50);
    grip();
    delay(1000);
    release();
    delay(1000);
    gripping = false;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");

    timeClient.begin();

}


// Preset gestures
void grip() {
    Serial.println("Grip activated!");
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

void wave() {
    for (int i = 0; i < 2; i++) {
        grip();
        delay(500);
        release();
        delay(500);
    }
    Serial.println("Waving hand!");
}

void point() {
    grip();
    index_finger.extend();
}

void mexicanWave() {
    Finger fingers[] = {thumb_finger, thumb_knuckle, index_finger, middle_finger, ring_finger, pinky_finger};
    for (int i = 0; i < 6; i++) {
        fingers[i].retract();
        delay(350);
        fingers[i].extend();
    }
    Serial.println("Mexican wave!");
}

void rockOn(){
  grip();
  pinky_finger.extend();
  index_finger.extend();
}

void peace(){
  grip();
  index_finger.extend();
  middle_finger.extend();
}

void flipBird(){
  grip();
  middle_finger.extend();
}

void hangLow(){
  grip();
  pinky_finger.extend();
  thumb_knuckle.extend();
  thumb_finger.extend();
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
    Serial.println(timeClient.getFormattedTime());

    int minutes = timeClient.getMinutes();
    int seconds = timeClient.getSeconds();
    int timesToRun = minutes / 15; // Determines how many times to run extend() and grip()

    if (minutes % 15 == 0 && seconds == 0)
    {
      grip();
      for (int i = 0; i < timesToRun; i++)
      {
          release();
          delay(500);
          grip();
          delay(500);
      }
    }

    asBinary(timeClient.getHours());
}


void wiggleFingers(int duration = 10000, int interval = 200) {
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
        int fingerIndex = random(6);
        switch (fingerIndex) {
            case 0: thumb_finger.retract(); delay(interval); thumb_finger.extend(); break;
            case 1: thumb_knuckle.retract(); delay(interval); thumb_knuckle.extend(); break;
            case 2: index_finger.retract(); delay(interval); index_finger.extend(); break;
            case 3: middle_finger.retract(); delay(interval); middle_finger.extend(); break;
            case 4: ring_finger.retract(); delay(interval); ring_finger.extend(); break;
            case 5: pinky_finger.retract(); delay(interval); pinky_finger.extend(); break;
        }
    }
    Serial.println("Wiggling fingers complete!");
}


void loop() {
  time();
  delay(1000);
}
