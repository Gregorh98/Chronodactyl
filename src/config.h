#ifndef CONFIG_H
#define CONFIG_H

// Servo PWM limits
#define SERVO_MIN  100
#define SERVO_MAX  650
#define SERVO_CONTROLLER_ADDRESS 0x40

// Pin assignments
#define PIN_SERVO_THUMB  0
#define PIN_SERVO_INDEX  1
#define PIN_SERVO_MIDDLE 2
#define PIN_SERVO_RING   3
#define PIN_SERVO_PINKY  4
#define PIN_SERVO_THUMB_KNUCKLE 5

#define PIN_SWITCH_ANIMATION 11
#define PIN_SWITCH_DST       12

// Speed
#define FINGER_MOVE_PERIOD_MS 2000

// RTC
#define RESET_RTC false


#endif