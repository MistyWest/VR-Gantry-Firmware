/* Gantry Control v0.4
 *  
 * Control stepper motors through serial commands. Improved speed performance of serial communication.
 * 
 * Last edited: August 25, 2016
 * Author: Justin Lam
 */

#include <AccelStepper.h>

#define MAX_SPEED 1000 // 1800
#define MAX_ACCEL 2000 // 12500
#define X_STEP_PIN 2
#define X_DIR_PIN 3
#define Y_STEP_PIN 4
#define Y_DIR_PIN 5
#define STEP_NUM 0.5  // half-stepping






// Initialize variables
uint8_t rcv_buff[12];
uint8_t i = 0;
boolean new_cmd = false;
uint16_t local_check_sum = 0;
uint8_t mode = 0;
bool firstPass = true;

typedef struct CMD_RCV_S {
  uint32_t pos_x;
  uint32_t pos_y;
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
* UnStuffData decodes "length" bytes of
* data at the location pointed to by "ptr",
* writing the output to the location pointed
* to by "dst".
*/
void UnStuffData(const uint8_t *ptr, unsigned long length, uint8_t *dst) {
  const uint8_t *end = ptr + length;
  while (ptr < end)  {
    int i, code = *ptr++;
    for (i = 1; i<code; i++)
      *dst++ = *ptr++;
    if (code < 0xFF)
      *dst++ = 0;
  }
}

// Command the steppers to move to an absolute position
void moveSteppers(uint32_t dX, uint32_t dY) {
  
  uint32_t dA = dX + dY;
  uint32_t dB = dX - dY;

  stepper1.moveTo(dA);
  stepper2.moveTo(dB);
//  stepper1.moveTo(dX);
//  stepper2.moveTo(dX);
  
}

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);

  // Initialize stepper motors
  stepper1.setMaxSpeed(MAX_SPEED/STEP_NUM);
  stepper1.setAcceleration(MAX_ACCEL/STEP_NUM);  
  
  stepper2.setMaxSpeed(MAX_SPEED/STEP_NUM);
  stepper2.setAcceleration(MAX_ACCEL/STEP_NUM); 

  // Initialize serial communication  
  Serial.begin(115200);
}

void loop() {

  if(Serial.available() > 0) {
    // Set origin to when it first turns on. Probably won't need this later
    if (firstPass) {
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      firstPass = false;
    }
    while(Serial.available() > 0) {
      switch(mode) {
       case 0:
        if(Serial.read() == 0x00) // Read the first byte that signifies a packet
          mode = 1;
        break;
        
       case 1:
        rcv_buff[i] = Serial.read(); // Read the rest of the bytes into a buffer
        i++;
        if(i == 11) {
          mode = 0;
          i = 0;
          UnStuffData( rcv_buff, 11, (uint8_t *)(&cmd_rcv));
          local_check_sum = Fletcher16( (uint8_t *)(&cmd_rcv), 8);
         
          if(cmd_rcv.check_sum == local_check_sum)
            new_cmd = true;
          }
          break;
      }
    }
  }
  
  // Check if complete serial command was received
  if (new_cmd) {
    moveSteppers(cmd_rcv.pos_x, cmd_rcv.pos_y);
    new_cmd = false;
  }

  stepper1.run();
  stepper2.run();
}
