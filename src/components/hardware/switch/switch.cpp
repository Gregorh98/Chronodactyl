#include "switch.h"

void Switch::init()
{
    pinMode(_pin, INPUT_PULLUP);
    current_state = digitalRead(_pin);
    just_changed = false;
}

void Switch::update()
{
    bool new_state = digitalRead(_pin);
    just_changed = (new_state != current_state);
    current_state = new_state;
}
