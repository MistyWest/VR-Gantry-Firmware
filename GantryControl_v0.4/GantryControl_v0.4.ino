/* Gantry Control v0.4
 *  
 * Control stepper motors through serial commands. Improved speed performance of serial communication.
 * 
 * Last edited: August 25, 2016
 * Author: Justin Lam
 */

#include <AccelStepper.h>

#define MAX_SPEED 1800 // 1200
#define MAX_ACCEL 5000 // 2000
#define BUFFER_SIZE 20
#define X_STEP_PIN 2
#define X_DIR_PIN 3
#define Y_STEP_PIN 4
#define Y_DIR_PIN 5
#define STEP_NUM 0.5 // 1/2 step    
#define STEP_SIZE 1.8 // degrees
#define PULLEY_RADIUS 0.009 // radius in m <- THIS DETERMINES THE DISTANCE OF MEASURE

// Define steppers and the pins they will use
AccelStepper stepper1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);  // type, step pin, direction pin
AccelStepper stepper2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Initialize variables
char escapeChar = ';';
String inputX ="";
String inputY ="";
boolean stringComplete = false;
boolean readNextVal = false;
char inChar;

int state = 1;
boolean stepper1_fin = false;
boolean stepper2_fin = false;
boolean nextState = false;
boolean runSteppers = false;
unsigned long currentMillis, previousMillis = 0;

float dX, dY;
float pos;

typedef struct CMD_RCV_S {
  uint32_t pos_x;
  uint32_t pos_y;
  uint16_t check_sum;
} cmd_rcv_t;

cmd_rcv_t cmd_rcv;

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

void UnStuffData(const uint8_t *ptr, unsigned long length, uint8_t *dst)
{
  const uint8_t *end = ptr + length;
  while (ptr < end)
  {
    int i, code = *ptr++;
    for (i = 1; i<code; i++)
      *dst++ = *ptr++;
    if (code < 0xFF)
      *dst++ = 0;
  }
}

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);

  // Initialize stepper motors
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(MAX_ACCEL);  
  
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(MAX_ACCEL); 

  // Initialize variables
  inputX.reserve(BUFFER_SIZE);
  inputY.reserve(BUFFER_SIZE);

  // Initialize serial communication  
  Serial.begin(115200);
}

uint8_t rcv_buff[12];
uint8_t i = 0;
boolean new_cmd = false;
uint32_t dA_prev = 0;
uint32_t dB_prev = 0;
bool changeDir_A = true;
bool changeDir_B = true;
uint16_t local_check_sum = 0;
uint8_t mode = 0;
bool firstPass = true;

void loop() {

  if(Serial.available() > 0) {
    if (firstPass) {
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      firstPass = false;
    }
    while(Serial.available() > 0) {
      switch(mode) {
       case 0:
        if(Serial.read() == 0x00)
          mode = 1;
        break;
        
       case 1:
        rcv_buff[i] = Serial.read();
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
    Serial.print(cmd_rcv.pos_x);
    moveSteppers(cmd_rcv.pos_x, cmd_rcv.pos_y);
    new_cmd = false;
  }

  stepper1.run();
  stepper2.run();
 

//  if (changeDir_A) {
//    stepper1.run();
//    changeDir_A = false;
//  } 
//  else {
//    stepper1.runSpeed();
//  }
//
//  if (changeDir_B) {
//    stepper2.run();
//    changeDir_B = false;
//  } 
//  else {
//    stepper2.runSpeed();
//  }
  

}

boolean moveSteppers(uint32_t dX, uint32_t dY) {
  
  uint32_t dA = dX + dY;
  uint32_t dB = dX - dY;

  // Check if steppers should be constant speed or not
  if (dX - dA_prev < 0) {
    changeDir_A = true;
  }
  if (dY - dB_prev < 0) {
    changeDir_B = true;
  }
  
  dA_prev = dX;
  dB_prev = dY;
  
  // Stop steppers if they're still moving
//  if (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
//      stepper1.stop();
//      stepper2.stop();
//  } 
  stepper1.moveTo(dA);
  stepper2.moveTo(dB);

//  if(dA == 0)
//    stepper1.stop();
//  else
//    stepper1.move(dA);
//    
//  if(dB == 0)
//    stepper2.stop();
//  else
//    stepper2.move(dB);
  
  return true;
  
}

void updateStatus() {
  if (!stepper1.isRunning() && !stepper2.isRunning()) {
//      Serial.write("ready");
    }
}
