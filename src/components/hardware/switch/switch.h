 # ifndef SWITCH_H
# define SWITCH_H

#include <Adafruit_PWMServoDriver.h>

class Switch
{
  private:
    int _pin;

  public:
    Switch(int pin) : _pin(pin), current_state(HIGH), just_changed(false) {
      pinMode(_pin, INPUT_PULLUP);
    }

    void init();
    void update();

    bool current_state;
    bool just_changed;

    
};

# endif