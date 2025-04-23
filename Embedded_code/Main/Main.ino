#include <Arduino.h>
#include "MotorControl.h"
#include "MPU6050_PID_YPR.h"
#include "PID_v1.h"
float ypr[3];
float move_d[2];
struct Vector3 {
  float x, y, z;

  void print(const char* label) {
    Serial.print(label);
    Serial.print(": (");
    Serial.print(x, 3); Serial.print(", ");
    Serial.print(y, 3); Serial.print(", ");
    Serial.print(z, 3); Serial.println(")");
  }
};

struct Vector3 resultvector;
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

// PID config
double pitchSetpoint = 0; // Upright
double rollSetpoint = 0;  // Upright
double pitchInput, rollInput;
double pitchOutput, rollOutput;
double Kp = 10000, Ki = 0, Kd = 0;
double Kp_ = 10000, Ki_ = 0, Kd_ = 0;

PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, 0, 0);


PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp_, Ki_, Kd_, 0, 0);

int checksignage(float number)
{
  if(number >= 0){
    return 1;
  }
  else return -1;
}


void setup() {
  // Initialize motor pins as outputs.
  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_PUL_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
  }
  setup_MPU6050();
   pitchPID.SetOutputLimits(-1.0, 1.0); // or -90 to 90 for angles
  rollPID.SetOutputLimits(-1.0, 1.0); // or -90 to 90 for angles

   //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
}

void loop() {
  
  //static float mv_dir[2];
  // Try to get sensor data, but don't block. Returns from the function and proceeds on old data.
  bool b_newMpuData = readout_MPU6050(ypr);
 
  /*
  // Z-Axis
  mv_dir[0] = sin(fabs(ypr[2])) * -checksignage(ypr[2]);
  // X-Axis
  mv_dir[1] = sin(fabs(ypr[1])) * -checksignage(ypr[1]);
*/
/*
  Serial.print("Yaw: ");
  Serial.print(ypr[0]);
  Serial.print(", Pitch: ");
  Serial.print(ypr[1]);
  Serial.print(", Roll: ");
  Serial.println(ypr[2]);
  Serial.print(", mvdir1: ");
  Serial.print(mv_dir[1]);
  Serial.print(", mvdir2: ");
  Serial.println(mv_dir[0]);
*/
  pitchInput = (double)ypr[1];
  rollInput = (double)ypr[2];

  //pitchPID.Compute();
  //rollPID.Compute();
  /*
  Serial.print("PitchOut: "); Serial.print(pitchOutput);
  Serial.print(", RollOut: "); Serial.println(rollOutput);
  */
  double resultspeed = (fabs(rollOutput) + fabs(pitchOutput))/2;
  double magnitude = sqrt((pitchOutput * pitchOutput )+ (rollOutput*rollOutput));

  Serial.print("resultspeed: ");
  Serial.println(resultspeed);
  /*
  Serial.print("magnitude: ");
  Serial.println(magnitude);
  */
  if( magnitude <= 0.01f)
  {
    //return;
  }
  //move_d[2] = { (float)(pitchOutput / magnitude), (float)(rollOutput / magnitude) };
  
  move_d[0] = 1;
  move_d[1] = 0;
  
  move(move_d, 1.0f);
  /*
  Serial.print(", move_d0: ");
  Serial.print(move_d[0]);
  Serial.print(", move_d1: ");
  Serial.print(move_d[1]);
  Serial.print(", speedpercentage: ");
  Serial.println(resultspeed);
  */
}
