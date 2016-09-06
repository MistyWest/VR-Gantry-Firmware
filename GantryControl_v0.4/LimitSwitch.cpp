/* LimitSwitch.cpp

Purpose: Class for handling switch debouncing

Last edited: August 29, 2016
Author: Justin Lam
*/

#include "LimitSwitch.h"
#include "Arduino.h"

// Class constructor
LimitSwitch::LimitSwitch(int pin) {
  switchPin = pin;
  last_debounce = 0;
  prev_state = HIGH;
  state = HIGH;
  debounce_delay = 5;   // Can increase this delay (in ms) if debouncing is not effective
}

// Check that the switch is actually pressed for some time to ensure it's not just noise/rattling. 
// Returns true if the switch is pressed, false if not
bool LimitSwitch::CheckSwitchPress() {
  reading = digitalRead(switchPin);

  if (reading != prev_state) {
    last_debounce = millis();
  }

  if ((millis() - last_debounce) > debounce_delay) {
    if (reading != state) {
      state = reading;
    
      if (state == LOW) {
        switchPressed = true;
      }
    }
    else {
      switchPressed = false;
    }
  }
  else {
    switchPressed = false;
  }
  prev_state = reading;
  return switchPressed;
}