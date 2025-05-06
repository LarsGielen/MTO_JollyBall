#include <Arduino.h>
#include "MotorControl.h"
#include "MPU6050_YPR.h"

float move_d[2];
static float ypr[3];
static double resultspeed, magnitude, rollError, pitchError, rollOutput, pitchOutput;
static double pitchSetpoint = 0, rollSetpoint = 0;
static const double max_angle = 5.0f * PI / 180.0f; // Angle in rad
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

int checksignage(float number)
{
  if(number >= 0){
    return 1;
  }
  else return -1;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Initialize motor pins as outputs.
  for (int i = 0; i < 3; i++) 
  {
    pinMode(MOTOR_PUL_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
  }
  setup_MPU6050(false);
}

void loop() 
{
  bool b_newMpuData = readout_mpu_ypr(ypr);
  /*
  Serial.print("ypr\t");
  Serial.print(ypr[0]);
  Serial.print("\t");
  Serial.print(ypr[1]);
  Serial.print("\t");
  Serial.println(ypr[2]);
*/
  // P only
  pitchError = pitchSetpoint - ypr[1];
  rollError = rollSetpoint - ypr[2];

  Serial.println(max_angle);
  
  pitchOutput = mapFloat(pitchError, -max_angle, max_angle, -1.0f, 1.0f);
  rollOutput = mapFloat(rollError, -max_angle, max_angle, -1.0f, 1.0f);
  Serial.print(pitchOutput);
  Serial.print("\t"); 
  Serial.print(rollOutput);
  Serial.print("\t"); 
  resultspeed = (fabs(rollOutput) + fabs(pitchOutput))/2.0f;
  resultspeed = constrain(resultspeed, 0.0f, 1.0f);
  magnitude = sqrt((pitchOutput * pitchOutput )+ (rollOutput*rollOutput));
  
  
  Serial.print("Resultspeed:");
  Serial.print(resultspeed);
  Serial.print("\t");
  Serial.print("Magnitude: ");
  Serial.print(magnitude);
  Serial.print("\t");
  
  /*
  Serial.print("pitchOutput: ");
  Serial.print(pitchOutput);
  Serial.print("\t");
  Serial.print("rollOutput: ");
  Serial.println(rollOutput);
  */
  
  if(magnitude <= 0.00001f) return; // don't let NAN values through
  
  move_d[0] = (float)(pitchOutput / magnitude);
  move_d[1] = (float)(rollOutput / magnitude);
  //move_d[0] = 1;
  //move_d[1] = 0;
  Serial.print(", move_d0: ");
  Serial.print(move_d[0]);
  Serial.print(", move_d1: ");
  Serial.println(move_d[1]);  

  move(move_d, resultspeed);
}
