#include <Arduino.h>
#include "MotorControl.h"

// --- Hardware and configuration parameters ---
const int MOTOR_PUL_PINS[] = {3, 4, 5};
const int MOTOR_DIR_PINS[] = {6, 7, 8};

const int MICROSTEP = 8;
const float MOTOR_STEP_ANGLE = 1.8f;
const float WHEEL_SIZE = 100.0f; // in mm (diameter)
const float STEP_SIZE = MOTOR_STEP_ANGLE / MICROSTEP;

// Global motor timers and flags:
long motor_timers[] = {0, 0, 0};
bool motor_flags[] = {false, false, false};

void setup() {
  // Initialize motor pins as outputs.
  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_PUL_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
  }
}

void loop() {
  static float theta = 0.0f; 
  const float speedPercent = 1.0f; 
  const float rotationSpeed = 0.001f;

  float moveDirection[2] = {cos(theta), sin(theta)};
  move(moveDirection, speedPercent);

  theta += rotationSpeed;
  if (theta > TWO_PI) 
    theta -= TWO_PI;
}
