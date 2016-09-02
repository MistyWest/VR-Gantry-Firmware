/* Gantry Control v0.4

   Purpose: Control stepper motors in a CoreXY configuration through a Visual Studios C++ program using serial communication.

   Comments:
   - Max steps/pulses per second (pps) of a given stepper motor is limited by the phase winding inductance and input voltage.
      - Using the current 57BHGY420-2 NEMA 23 motors with 5.5mH (+/- 20%) inductance per coil, the max is 1300 pps.
   - Max pps that can be reliably sent by the AccelStepper library is 4000 pps on a 16MHz Arduino.

   Last edited: August 29, 2016
   Author: Justin Lam
*/

#include <AccelStepper.h>

#define MAX_SPEED 700  // 1040 original
#define MAX_ACCEL 600  // 1040
#define CALIBRATION_SPEED 9
#define X_STEP_PIN 6
#define X_DIR_PIN 7
#define Y_STEP_PIN 4
#define Y_DIR_PIN 5
#define X_MIN_PIN 8
#define X_MAX_PIN 9
#define Y_MIN_PIN 2
#define Y_MAX_PIN 3
#define STEP_NUM 0.5  // half-stepping
#define GANTRY_SIZE_X 4934  // 4934: 1.55m length, 2cm pulley radius, 1.8deg step size, 1/2 step = 4934
#define GANTRY_SIZE_Y 4920
#define DEBOUNCE_DELAY 5
#define GANTRY_BOUND_OFFSET 15
#define X_BODY_LENGTH 433
#define Y_BODY_LENGTH 823 // 643 
#define CALIBRATION_CODE 0xF1ACA

// Initialize variables
uint8_t rcv_buff[12];
uint8_t i = 0;
bool new_cmd = false;
uint16_t local_check_sum = 0;
uint8_t mode = 0;
bool calibrate_axis = false;

// Initialize arbitrary limit switch boundaries to be changed after calibration
int32_t y_max = 99999999;
int32_t x_max = 99999999;
int32_t y_min = -99999999;
int32_t x_min = -99999999;

// Command to be received through serial port
typedef struct CMD_RCV_S {
  int32_t pos_x;
  int32_t pos_y;
  uint16_t check_sum;
} cmd_rcv_t;

cmd_rcv_t cmd_rcv;

// Define steppers and the pins they will use
AccelStepper stepper1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);  // type, step pin, direction pin
AccelStepper stepper2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Class definition for limit switches mainly for debouncing
class LimitSwitch{
  uint8_t switchPin;
  uint8_t reading;
  unsigned long last_debounce;
  uint8_t prev_state;
  uint8_t state;
  bool switchPressed;

  public:
  LimitSwitch(uint8_t pin) {
    switchPin = pin;
    last_debounce = 0;
    prev_state = HIGH;
    state = HIGH;
    switchPressed = false;
  }
  bool CheckSwitchPress() {
    reading = digitalRead(switchPin);

    if (reading != prev_state) {
      last_debounce = millis();
    }

    if ((millis() - last_debounce) > DEBOUNCE_DELAY) {
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
};

// Initialize limit switches
LimitSwitch YMaxSwitch(Y_MAX_PIN);
LimitSwitch YMinSwitch(Y_MIN_PIN);
LimitSwitch XMaxSwitch(X_MAX_PIN);
LimitSwitch XMinSwitch(X_MIN_PIN); 

// Checksum to verify serial packet
uint16_t Fletcher16(uint8_t *data, int count) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  int index;

  for (index = 0; index < count; ++index) {
    sum1 = (sum1 + data[index]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return (sum2 << 8) | sum1;
}

// UnStuffData decodes "length" bytes of data at the location pointed to by "ptr",
// writing the output to the location pointed to by "dst".
void UnStuffData(const uint8_t *ptr, unsigned long length, uint8_t *dst) {
  const uint8_t *end = ptr + length;
  while (ptr < end)  {
    int i, code = *ptr++;
    for (i = 1; i < code; i++)
      *dst++ = *ptr++;
    if (code < 0xFF)
      *dst++ = 0;
  }
}

// Calibration procedure to properly zero the gantry
void calibrateAxis(char axis) {
  bool switch_pressed = false;

  // Run at a slower speed so it doesn't smash into the limit switches
  stepper1.setSpeed(CALIBRATION_SPEED);
  stepper2.setSpeed(CALIBRATION_SPEED);

  int32_t dist = 0;

  // Keep moving in the specified direction until the switch is hit
  while(!switch_pressed) {
    delay(CALIBRATION_SPEED/3);
    dist += 1;

    switch (axis) {
    case 'x':
      moveSteppers(dist,0);
      switch_pressed = XMaxSwitch.CheckSwitchPress();
      break;

    case 'y':
      moveSteppers(0,dist);
      switch_pressed = YMaxSwitch.CheckSwitchPress();
      break;
    }

    // Run at constant speed w/o accel/decel
    stepper1.runSpeed();
    stepper2.runSpeed();
  }

  // Move the gantry to the center of the axis to set the correct zero position
  switch (axis) {
    case 'x':
      moveSteppersRelative(-(GANTRY_SIZE_X - X_BODY_LENGTH)/2, 0);
      break;

    case 'y':
      moveSteppersRelative(0, -(GANTRY_SIZE_Y - Y_BODY_LENGTH)/2);
      break;
  }

  // Keep running the steppers until it's at the position
  while(stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }

  // Set the current position as the new zero
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  // Set the software axis limits
  switch (axis) {
    case 'x':
      x_max = (GANTRY_SIZE_X - X_BODY_LENGTH) / 2 - GANTRY_BOUND_OFFSET;
      x_min = - x_max;
      break;

    case 'y':
      y_max = (GANTRY_SIZE_Y - Y_BODY_LENGTH) / 2 - GANTRY_BOUND_OFFSET;
      y_min = - y_max;
      break;
  }  
  // return false;
}

// Command the steppers to move to an absolute position if given an XY input
void moveSteppers(int32_t dX, int32_t dY) {
  // Check if the positions are out of bounds
  if (dY > y_max)
    dY = y_max;
  else if (dY < y_min)
    dY = y_min;

  if (dX > x_max)
    dX = x_max;
  else if (dX < x_min)
    dX = x_min;
    
  int32_t dA = dX + dY;
  int32_t dB = dX - dY;
  
  stepper1.moveTo(dA);
  stepper2.moveTo(dB);
}

// Command the steppers to move to a relative position if given an XY input
void moveSteppersRelative(int32_t dX, int32_t dY) {
  int32_t dA = dX + dY;
  int32_t dB = dX - dY;

  stepper1.move(dA);
  stepper2.move(dB);
}

// Verify the incoming serial packet is a legit command
void parseSerialPacket() {
  while (Serial.available() > 0) {
    switch (mode) {
      case 0:
        if (Serial.read() == 0x00) // Read the first byte that signifies a packet
          mode = 1;
        break;

      case 1:
        rcv_buff[i] = Serial.read(); // Read the rest of the bytes into a buffer
        i++;
        if (i == 11) {
          mode = 0;
          i = 0;
          UnStuffData( rcv_buff, 11, (uint8_t *)(&cmd_rcv));
          local_check_sum = Fletcher16( (uint8_t *)(&cmd_rcv), 8);

          if (cmd_rcv.check_sum == local_check_sum)
            new_cmd = true;
        }
        break;
    }
  }
}

void sendCommandToSteppers(int32_t x, int32_t y) {
  
  moveSteppers(x, y);
}

void checkLimitSwitches() {
  bool y_max_pressed = false;
  bool y_min_pressed = false;
  bool x_max_pressed = false;
  bool x_min_pressed = false;

  int32_t x, y;

  x = getCurrentX();
  y = getCurrentY();

  // only check one of the x limit switches
  if (x > 0) {
    x_max_pressed = XMaxSwitch.CheckSwitchPress();
  } 
  else {
    x_min_pressed = XMinSwitch.CheckSwitchPress();
  }

  // only check one of the y limit switches
  if (y > 0) {
    y_max_pressed = YMaxSwitch.CheckSwitchPress();
  }
  else {
    y_min_pressed = YMinSwitch.CheckSwitchPress();
  }

  // reset the min/max boundaries if a switch has been pressed
  if (y_max_pressed) {
    y_max = getCurrentY() - GANTRY_BOUND_OFFSET;
    stopMotors();
    moveSteppersRelative(0, y_max);
  }
  else if (y_min_pressed) {
    y_min = getCurrentY() + GANTRY_BOUND_OFFSET;
    stopMotors();
    moveSteppersRelative(0, y_min);
  } 
  else if (x_max_pressed) {
    x_max = getCurrentX() - GANTRY_BOUND_OFFSET;
    stopMotors();
    moveSteppersRelative(x_max, 0);
  }
  else if (x_min_pressed) {
    x_min = getCurrentX() + GANTRY_BOUND_OFFSET;
    stopMotors();
    moveSteppersRelative(x_min, 0);
  }
}

// Gets the instantaneous X value based on both stepper positions
int32_t getCurrentX() {
  return (stepper1.currentPosition() + stepper2.currentPosition()) / 2;
}

// Gets the instantaneous Y value based on both stepper positions
int32_t getCurrentY() {
  return (stepper1.currentPosition() - stepper2.currentPosition()) / 2;
}

// Stop both motors
void stopMotors() {
  stepper1.stop();
  stepper2.stop();
}

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);  // onboard LED pin

  digitalWrite(13, LOW);  // make sure LED is off 

  // Initialize stepper motors
  stepper1.setMaxSpeed(MAX_SPEED / STEP_NUM);
  stepper1.setAcceleration(MAX_ACCEL / STEP_NUM);

  stepper2.setMaxSpeed(MAX_SPEED / STEP_NUM);
  stepper2.setAcceleration(MAX_ACCEL / STEP_NUM);

  // Set origin to when it first turns on. Probably won't need this later
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {  
  if (Serial.available() > 0) {
    parseSerialPacket();
    
    if (new_cmd) {
      // Run the calibration procedure
      if (cmd_rcv.pos_x == CALIBRATION_CODE && cmd_rcv.pos_y == CALIBRATION_CODE) {
        calibrateAxis('y');
        calibrateAxis('x');
        digitalWrite(13, HIGH); // turn onboard led on to signify calibration complete
      }
      else {
        moveSteppers(cmd_rcv.pos_x, cmd_rcv.pos_y);
      }
      new_cmd = false;
    }
  }

  checkLimitSwitches();

  // Call these functions as often as possible
  stepper1.run();
  stepper2.run();
}