/* Gantry Control v0.4
 *  
 * Control stepper motors through serial commands. Improved speed performance of serial communication.
 * 
 * Last edited: August 24, 2016
 * Author: Justin Lam
 */

#include <AccelStepper.h>

#define MAX_SPEED 2000 // 1200
#define MAX_ACCEL 2000 // 2000
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
  float pos_x;
  float pos_y;
} cmd_rcv_t;

cmd_rcv_t cmd_rcv;

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

  // Initialize variables
  inputX.reserve(BUFFER_SIZE);
  inputY.reserve(BUFFER_SIZE);

  // Initialize serial communication  
  Serial.begin(115200);
}

uint8_t rcv_buff[8];
uint8_t i = 0;
boolean new_cmd = false;

void loop() {
  if(Serial.available() > 0) {
    while(Serial.available() > 0) {
      rcv_buff[i] = Serial.read();
      i++;

      if(i == 8) {
        i = 0;
        memcpy( &cmd_rcv, rcv_buff, 8);
        new_cmd = true;
      }
    }
  }
  
  // Check if complete serial command was received
  if (new_cmd) {
    moveSteppers(cmd_rcv.pos_x, cmd_rcv.pos_y);
    new_cmd = false;
  }
 
  updateStatus();

  stepper1.run();
  stepper2.run();
}

boolean moveSteppers(float dX, float dY) {
  float distToSteps = 360 / (2 * 3.141592654 * PULLEY_RADIUS * STEP_NUM * STEP_SIZE);
  
  dX = dX * distToSteps;
  dY = dY * distToSteps;
  
  float dA = dX + dY;
  float dB = dX - dY;

  if (stepper1.distanceToGo() == 0) {
    stepper1_fin = true; 
  }
  if (stepper2.distanceToGo() == 0) {
    stepper2_fin = true;
  }

  if (stepper1_fin && stepper2_fin) {
    stepper1.move(dA);
    stepper2.move(dB);
    stepper1_fin = false;
    stepper2_fin = false;
    return true;
  } else {
    return false;
  }
}

void updateStatus() {
  if (!stepper1.isRunning() && !stepper2.isRunning()) {
      Serial.write("ready");
    }
}
