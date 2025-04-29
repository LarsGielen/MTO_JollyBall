#include <Arduino.h>
#include "MotorControl.h"
#include "MPU6050_PID_YPR.h"

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
double pitchSetpoint = 0;
double rollSetpoint = 0; 
double max_angle = 90.0f * PI / 180.0f; // Angle in rad
double pitchOutput, rollOutput;
double Kp = 10000, Ki = 0, Kd = 0;

// Temp Functions


// Main Functions
void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_PUL_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
  }

  setup_MPU6050();

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void loop() {
  bool b_newMpuData = readout_MPU6050(ypr);
  
  // P only
  double pitchError = pitchSetpoint - ypr[1];
  double rollError = rollSetpoint - ypr[2];

  pitchOutput = map(-1.3f, max_angle, 1.3f, -1.0f, 1.0f);
  rollOutput = map(-max_angle, max_angle, rollError, -1.0f, 1.0f);

  double resultspeed = (fabs(rollOutput) + fabs(pitchOutput))/2;
  double magnitude = sqrt((pitchOutput * pitchOutput )+ (rollOutput*rollOutput));
  Serial.print("y: ");
  Serial.print(pitchOutput);
  Serial.print("p: ");
  Serial.print(pitchOutput);
  Serial.print("r: ");
  Serial.print(pitchOutput);

  Serial.print("pitchOutput: ");
  Serial.print(pitchOutput);
  Serial.print(" -- rollOutput: ");
  Serial.println(rollOutput);
}
