/* Gantry Control v0.4
 *  
 * Control stepper motors through serial commands.
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
#define STATUS_DELAY 5

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

void loop() {
  currentMillis = millis();
  
  // Check if complete serial command was received
  if (stringComplete) {
    
    if (inputX == "go") { 
      runSteppers = true;
    } 
    else if (inputX == "s") {
      stepper1.stop();
      stepper2.stop();
      runSteppers = false;
    } 
    else {
      dX = inputX.toFloat();
      dY = inputY.toFloat();
      moveSteppers(dX, dY);
    }
    
    inputX = "";
    inputY = "";
    stringComplete = false;
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
//    delay(250);
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
      if ((currentMillis - previousMillis) >= STATUS_DELAY) {
        Serial.write("ready");
        previousMillis = currentMillis;
      }
    }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    inChar = Serial.read();

    if (inChar == ',') {
      inputX += '\0';
      readNextVal = true;
    }

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == escapeChar) {
      inputY += '\0';
      inputY.remove(0,1);
      stringComplete = true;
      readNextVal = false;
      break;
    }
    
    if (readNextVal)
    {
      inputY += inChar;
    } else {
      inputX += inChar;
    }
  }
}
