/*LimitSwitch.h

Purpose: Class for handling switch debouncing

Last edited: August 29, 2016
Author: Justin Lam
*/

#ifndef HEADER_LIMITSWITCH
  #define HEADER_LIMITSWITCH

class LimitSwitch{
  int switchPin;      // hardware pin on dev board
  int reading;        
  unsigned long last_debounce; 
  int prev_state;
  int state;
  bool switchPressed;
  int debounce_delay;

  public:
  LimitSwitch(int pin);     // constructor to initialize variables
  bool CheckSwitchPress();  // Returns true if switch is pressed, false if not
};

#endif