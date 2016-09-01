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

#define MAX_SPEED 500  // 1040
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
#define GANTRY_SIZE_Y 4934 // 1.55m length, 2cm pulley radius, 1.8deg step size, 1/2 step
#define DEBOUNCE_DELAY 5
#define GANTRY_BOUND_OFFSET 5

// Initialize variables
uint8_t rcv_buff[12];
uint8_t i = 0;
boolean new_cmd = false;
uint16_t local_check_sum = 0;
uint8_t mode = 0;
volatile uint8_t sw_state = 0;
int32_t y_near_bound = 0;
int32_t y_max = 999999999;
int32_t y_min = 0;
int32_t x_max = 0;
int32_t x_min = 0;
int32_t y_max_switch_pos = 0;
int32_t x_offset = 0;
int32_t y_offset = 0;
unsigned long y_max_last_debounce = 0;

int y_max_prev_state = HIGH;
int y_max_state = 1;
bool y_max_limit = false;

typedef struct CMD_RCV_S {
  int32_t pos_x;
  int32_t pos_y;
  uint16_t check_sum;
} cmd_rcv_t;

cmd_rcv_t cmd_rcv;

// Define steppers and the pins they will use
AccelStepper stepper1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);  // type, step pin, direction pin
AccelStepper stepper2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

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

// Command the steppers to move to an absolute position
void moveSteppers(int32_t dX, int32_t dY) {
  dX += x_offset;
  dY += y_offset;

  if (dY < y_max) {
    int32_t dA = dX + dY;
    int32_t dB = dX - dY;
  
    if (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
      stepper1.stop();
      stepper2.stop();
    }
    
    stepper1.moveTo(dA);
    stepper2.moveTo(dB);
  }
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

void sendCommandToSteppers() {
 
  int y_max_reading = digitalRead(Y_MAX_PIN);   // read the limit switch state

  // Check if the switch changed, due to noise or actual pressing
  if (y_max_reading != y_max_prev_state) {
      y_max_last_debounce = millis();
  }

  // If the reading is longer than debounce_delay, then it's probably actually been pressed
  if ((millis() - y_max_last_debounce) > DEBOUNCE_DELAY) {
    if (y_max_reading != y_max_state) {   // if the button state has changed
      y_max_state = y_max_reading;
      
      if (y_max_state == LOW) {
        Serial.println("off");
        stepper1.stop();
        stepper2.stop();
        y_max = cmd_rcv.pos_y - GANTRY_BOUND_OFFSET;
      }
    }
    else {
      moveSteppers(cmd_rcv.pos_x, cmd_rcv.pos_y);
    }
    
  }
  else {
    moveSteppers(cmd_rcv.pos_x, cmd_rcv.pos_y);
  }
  y_max_prev_state = y_max_reading;
}

void calibrateY(int32_t y_actual) {
  y_offset = y_actual - GANTRY_SIZE_Y/2;
  moveSteppers(cmd_rcv.pos_x,0);
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
      sendCommandToSteppers();
      new_cmd = false;
    }
      
  }
  
  // Call these functions as often as possible
  stepper1.run();
  stepper2.run();
}

