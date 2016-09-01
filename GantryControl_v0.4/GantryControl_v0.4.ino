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

#define MAX_SPEED 500  // 1040 original
#define MAX_ACCEL 800  // 1040
#define X_STEP_PIN 6
#define X_DIR_PIN 7
#define Y_STEP_PIN 4
#define Y_DIR_PIN 5
#define X_MIN_PIN 8
#define X_MAX_PIN 9
#define Y_MIN_PIN 2
#define Y_MAX_PIN 3
#define STEP_NUM 0.5  // half-stepping
#define GANTRY_SIZE_X 4934  // 1.55m length, 2cm pulley radius, 1.8deg step size, 1/2 step = 4934
#define GANTRY_SIZE_Y 4950 
#define DEBOUNCE_DELAY 5
#define GANTRY_BOUND_OFFSET 25

// Initialize variables
uint8_t rcv_buff[12];
uint8_t i = 0;
boolean new_cmd = false;
uint16_t local_check_sum = 0;
uint8_t mode = 0;

// Initialize arbitrary limit switch boundaries
int32_t y_max = 99999999;
int32_t x_max = 99999999;
int32_t y_min = -99999999;
int32_t x_min = -99999999;

typedef struct CMD_RCV_S {
  int32_t pos_x;
  int32_t pos_y;
  uint16_t check_sum;
} cmd_rcv_t;

cmd_rcv_t cmd_rcv;

// Define steppers and the pins they will use
AccelStepper stepper1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);  // type, step pin, direction pin
AccelStepper stepper2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

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
//          stepper1.stop();
//          stepper2.stop();
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

LimitSwitch YMaxSwitch(Y_MAX_PIN);
LimitSwitch YMinSwitch(Y_MIN_PIN);
LimitSwitch XMaxSwitch(X_MAX_PIN);
LimitSwitch XMinSwitch(X_MIN_PIN); 

// Checksum to verify serial packet
uint16_t Fletcher16(uint8_t *data, int count)
{
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  int index;

  for (index = 0; index < count; ++index) {
    sum1 = (sum1 + data[index]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return (sum2 << 8) | sum1;
}

/*
  UnStuffData decodes "length" bytes of
  data at the location pointed to by "ptr",
  writing the output to the location pointed
  to by "dst".
*/
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

void calibrateAxes() { 
  stepper1.setSpeed(0.01);
  stepper2.setSpeed(0.01);

  int32_t y_max_prev = y_max;
  int32_t dist = 1;
  
  while(y_max == y_max_prev) {
    sendCommandToSteppers(0,dist);
    dist += 1;
    stepper1.runSpeed();
    stepper2.runSpeed();
    delay(5);
  }
//  stepper1.stop();
//  stepper2.stop();
  moveSteppers(0,0);
  y_min = y_max - GANTRY_SIZE_Y - GANTRY_BOUND_OFFSET;
}

// Command the steppers to move to an absolute position
void moveSteppers(int32_t dX, int32_t dY) {
  if (dX == 99999 && dY == 99999) {
    calibrateAxes();
    return;
  }
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

  if (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.stop();
    stepper2.stop();
  }
  
  stepper1.moveTo(dA);
  stepper2.moveTo(dB);

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
  bool y_max_pressed, y_min_pressed, x_max_pressed, x_min_pressed;
  
  y_max_pressed = YMaxSwitch.CheckSwitchPress();
  y_min_pressed = YMinSwitch.CheckSwitchPress();
  x_max_pressed = XMaxSwitch.CheckSwitchPress();
  x_min_pressed = XMinSwitch.CheckSwitchPress();

  if (y_max_pressed) {
    y_max = y - GANTRY_BOUND_OFFSET;
//    moveSteppers(cmd_rcv.pos_x, y_max);
  }
  else if (y_min_pressed) {
    y_min = y + GANTRY_BOUND_OFFSET;
//    moveSteppers(cmd_rcv.pos_x, y_min);
  } 
  else if (x_max_pressed) {
    x_max = x - GANTRY_BOUND_OFFSET;
//    moveSteppers(x_max, cmd_rcv.pos_y);
  }
  else if (x_min_pressed) {
    x_min = x + GANTRY_BOUND_OFFSET;
//    moveSteppers(x_min, cmd_rcv.pos_y);
  }
  else {
    moveSteppers(x, y);
  }
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
      sendCommandToSteppers(cmd_rcv.pos_x, cmd_rcv.pos_y);
      new_cmd = false;
    }
  }
  
  // Call these functions as often as possible
  stepper1.run();
  stepper2.run();
}

