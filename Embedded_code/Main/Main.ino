#include <Arduino.h>
#include "MotorControl.h"
#include "MPU6050_PID_YPR.h"
#include "DataReceiver.h"

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

// MPU 6050
float ypr[3];
float move_d[2];

// PID config
float pitchSetpoint = 0;
float rollSetpoint = 0; 
float max_angle = 90.0f * (PI / 180.0f); // Angle in rad
float pitchOutput, rollOutput;
float Kp = 10000, Ki = 0, Kd = 0;

// multithreading config
#define CORE_0 0
#define CORE_1 1

// Temp Functions
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Multithreading Tasks
void TaskReceive(void *pvParameters) {
  receiveLoop();
}

void TaskMotorControl(void *pvParameters) {
  // Do pid stuff
  // Update motors 
}

// Main Functions
void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Led -> Green
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_PUL_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
  }

  setup_MPU6050();

  // Register Data Receiver Values
  // registerAdress(*kp, "kp_value")

  xTaskCreatePinnedToCore(
    TaskReceive, "Task Receive", // A name just for humans
    2048, // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    NULL, // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    1, // Priority
    NULL, // Task handle is not used here - simply pass NULL
    CORE_0  // Core on which the task will run
  );

  xTaskCreatePinnedToCore(
    TaskMotorControl, "Task Motor Control", // A name just for humans
    2048, // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    NULL, // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    1, // Priority
    NULL, // Task handle is not used here - simply pass NULL
    CORE_1  // Core on which the task will run
  );

  // Led -> red
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void loop() { }
