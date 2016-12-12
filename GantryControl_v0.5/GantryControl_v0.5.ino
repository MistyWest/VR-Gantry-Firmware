/* Gantry Control v0.5

   Purpose: Control stepper motors in a CoreXY configuration through a Visual Studios C++ program using serial communication.

   Comments:
   - 2016-12-06: Removed limit switch check in main loop because it wasn't really working
   - 2016-09-06: Changed board from Arduino Redboard to Teensy 3.2
   - 2016-08-29: Max steps/pulses per second (pps) of a given stepper motor is limited by the phase winding inductance and input voltage.
      - Using the current 57BHGY420-2 NEMA 23 motors with 5.5mH (+/- 20%) inductance per coil, the max is 1300 pps.
   - 2016-08-29, 2016: Max pps that can be reliably sent by the AccelStepper library is 4000 pps on a 16MHz Arduino.

   Author: Justin Lam
*/

#include <AccelStepper.h>
#include <LimitSwitch.h>

#define MAX_SPEED 3000            // Max skip-less speed w/ Teensy 3.2 @ 96MHz clock (determined by max processing speed and phase inductance of motors)
#define MAX_ACCEL 3000       
#define DELAY_LIM_SW_CHECK 10     // only check the switches every X milliseconds
#define CALIBRATION_SPEED 10      // nice and slow speed to run the calibration procedure
#define CALIBRATION_CODE 0xF1ACA  // this needs to match the c++ code
#define STOP_CODE 0xEF355
#define BAUD_RATE 115200    
#define SM_X_DIR_INVERT true      // invert stepper direction if going in opposite direction
#define SM_Y_DIR_INVERT true
#define SM_X_STEP_PIN 16          // stepper motor pins
#define SM_X_DIR_PIN  17
#define SM_Y_STEP_PIN 14
#define SM_Y_DIR_PIN  15
#define LS_X_MIN_PIN 3            // limit switch interrupt pins (blue)
#define LS_X_MAX_PIN 2            // (green)
#define LS_Y_MIN_PIN 4            // (red)
#define LS_Y_MAX_PIN 5            // (yellow)
#define STEP_NUM 0.5              // half-stepping
#define GANTRY_SIZE_X 4934        // 4934: 1.55m length, 2cm pulley radius, 1.8deg step size, 1/2 step. See spreadsheet for calculation
#define GANTRY_SIZE_Y 4934
#define GANTRY_BOUND_OFFSET 955   // 25 cm
#define X_BODY_LENGTH 573         // 18 cm
#define Y_BODY_LENGTH 637         // 20 cm
#define CALIBRATION_OFFSET_X 159  // 5 cm
#define CALIBRATION_OFFSET_Y 80   // 2.5 cm

// Initialize variables
uint8_t rcv_buff[12];
uint8_t i = 0;
bool new_cmd = false;
uint16_t local_check_sum = 0;
uint8_t mode = 0;
bool calibrate_axis = false;
unsigned long prev_lim_check = 0;
uint32_t prev_x = 0;
uint32_t prev_y = 0;

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
AccelStepper stepper1(AccelStepper::DRIVER, SM_X_STEP_PIN, SM_X_DIR_PIN);  // type, step pin, direction pin
AccelStepper stepper2(AccelStepper::DRIVER, SM_Y_STEP_PIN, SM_Y_DIR_PIN);

// Initialize limit switches
LimitSwitch YMaxSwitch(LS_Y_MAX_PIN);
LimitSwitch YMinSwitch(LS_Y_MIN_PIN);
LimitSwitch XMaxSwitch(LS_X_MAX_PIN);
LimitSwitch XMinSwitch(LS_X_MIN_PIN); 

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

// UNSTUFFDATA decodes "length" bytes of data at the location pointed to by "ptr",
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

  int32_t dist = 16;

  // Keep moving in the specified direction until the switch is hit
  while(!switch_pressed) {
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
      moveSteppers(-(GANTRY_SIZE_X - X_BODY_LENGTH)/2 - CALIBRATION_OFFSET_X, 0);
      break;

    case 'y':
      moveSteppers(0, -(GANTRY_SIZE_Y - Y_BODY_LENGTH)/2 - CALIBRATION_OFFSET_Y);
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
      x_max = GANTRY_SIZE_X / 2 - GANTRY_BOUND_OFFSET;
      x_min = - x_max;
      break;

    case 'y':
      y_max = GANTRY_SIZE_Y / 2 - GANTRY_BOUND_OFFSET;
      y_min = - y_max;
      break;
  }  
}

// Command the steppers to move to an absolute position if given an XY input
void moveSteppersTo(int32_t x, int32_t y) {
  long newAccel;
  int32_t dX, dY;

  // Check if the y position is out of bounds
  if (y > y_max)
    y = y_max;
  else if (y < y_min)
    y = y_min;

  // Check if the x position is out of bounds
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;

  // Equations of motion for coreXY design
  int32_t A = x + y;
  int32_t B = x - y;
  
  stepper1.moveTo(A);
  stepper2.moveTo(B);
}

// Command the steppers to move to a relative position if given an XY input
void moveSteppers(int32_t X, int32_t Y) {
  int32_t A = X + Y;
  int32_t B = X - Y;

  stepper1.move(A);
  stepper2.move(B);
}

// Verify the incoming serial packet is a legit command and set flag if true
void parseSerialPacket() {
  digitalWrite(13, HIGH);
  while (Serial.available() > 0) {
    switch (mode) {
      case 0:
        if (Serial.read() == 0x00) // Read the first byte that signifies a packet
          mode = 1;
        break;

      case 1:
        rcv_buff[i] = Serial.read(); // Read the rest of the bytes into a buffer
        i++;
        if (i == 11) {  // do on the last byte in the packet
          mode = 0;
          i = 0;
          UnStuffData( rcv_buff, 11, (uint8_t *)(&cmd_rcv));  // decode the packet
          local_check_sum = Fletcher16( (uint8_t *)(&cmd_rcv), 8);  // verify integrity of packet

          if (cmd_rcv.check_sum == local_check_sum)
            new_cmd = true;
        }
        break;
    }
  }
  digitalWrite(13, LOW);
}

// Check if there's a new command available and execute on it if true
void executeNewCmd() {
  if (new_cmd) {
    // Run the calibration procedure if the command matches the code
    if (cmd_rcv.pos_x == CALIBRATION_CODE && cmd_rcv.pos_y == CALIBRATION_CODE) {
      calibrateAxis('y');
      calibrateAxis('x');
      digitalWrite(13, HIGH); // turn onboard led on to signify calibration complete
    }
    else if (cmd_rcv.pos_x == STOP_CODE && cmd_rcv.pos_y == STOP_CODE) {
      stopMotors();
    }
    // Otherwise just send the command to the steppers
    else { 
      moveSteppersTo(cmd_rcv.pos_x, cmd_rcv.pos_y);
    }
    new_cmd = false;
  }
}

// Check the status of the limit switches to see if one has been triggered
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
    moveSteppers(0, y_max);
  }
  else if (y_min_pressed) {
    y_min = getCurrentY() + GANTRY_BOUND_OFFSET;
    stopMotors();
    moveSteppers(0, y_min);
  } 
  else if (x_max_pressed) {
    x_max = getCurrentX() - GANTRY_BOUND_OFFSET;
    stopMotors();
    moveSteppers(x_max, 0);
  }
  else if (x_min_pressed) {
    x_min = getCurrentX() + GANTRY_BOUND_OFFSET;
    stopMotors();
    moveSteppers(x_min, 0);
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

// Reset the software gantry limits if a switch has been pressed
// param: axis      - x or y/z axis in cartesian plane
// param: location  - 0 for min limit switch, 1 for max limit switch
void reset_limit(char axis, uint8_t location) {
  int32_t x = 0, y = 0;

  if (axis == 'x') {
    if (location == 0) {
      x_min = getCurrentX() + GANTRY_BOUND_OFFSET;
      x = x_min;
    }
    else {
      x_max = getCurrentX() - GANTRY_BOUND_OFFSET;
      x = x_max;
    }
    y = 0;
  }
  else if (axis == 'y') {
    if (location == 0) {
      y_min = getCurrentY() + GANTRY_BOUND_OFFSET;
      y = y_min;
    }
    else {
      y_max = getCurrentY() - GANTRY_BOUND_OFFSET;
      y = y_max;
    }
    x = 0;
  }

  stopMotors();
  moveSteppers(x, y); // move the gantry off the limit switch
}

// Stop both motors
void stopMotors() {
  stepper1.stop();
  stepper2.stop();
}

void setup() {
  pinMode(SM_X_STEP_PIN, OUTPUT);
  pinMode(SM_X_DIR_PIN, OUTPUT);
  pinMode(SM_Y_STEP_PIN, OUTPUT);
  pinMode(SM_Y_DIR_PIN, OUTPUT);
  pinMode(LS_X_MIN_PIN, INPUT_PULLUP);  // pullup the limit switches so it's normally high
  pinMode(LS_X_MAX_PIN, INPUT_PULLUP);
  pinMode(LS_Y_MIN_PIN, INPUT_PULLUP);
  pinMode(LS_Y_MAX_PIN, INPUT_PULLUP);

  pinMode(13, OUTPUT);  // onboard LED pin

  digitalWrite(13, LOW);  // turn LED on later after calibration

  // Initialize stepper motors
  stepper1.setMaxSpeed(MAX_SPEED / STEP_NUM);
  stepper1.setAcceleration(MAX_ACCEL / STEP_NUM);

  stepper2.setMaxSpeed(MAX_SPEED / STEP_NUM);
  stepper2.setAcceleration(MAX_ACCEL / STEP_NUM);

  // Invert pins if necessary
  stepper1.setPinsInverted(SM_X_DIR_INVERT);
  stepper2.setPinsInverted(SM_Y_DIR_INVERT);

  // Set origin to when it first turns on. Probably won't need this later
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  // Initialize serial communication
  Serial.begin(BAUD_RATE);
}

void loop() {  
  if (Serial.available() > 0) {
    parseSerialPacket();
    executeNewCmd();
  }

  // Check limit switch at reduced frequency
  // if (millis() - prev_lim_check > DELAY_LIM_SW_CHECK) {
  //   checkLimitSwitches();  
  //   prev_lim_check = millis();
  // }
  
  // Call these functions as often as possible
  stepper1.run();
  stepper2.run();
}
