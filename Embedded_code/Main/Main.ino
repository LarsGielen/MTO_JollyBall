#include <Arduino.h>
#include "MotorControl.h"
#include "MPU6050_PID_YPR.h"

float move_d[2];
static float ypr[3];

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

void setup() {
  // Initialize motor pins as outputs.
  for (int i = 0; i < 3; i++) 
  {
    pinMode(MOTOR_PUL_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
  }
  setup_MPU6050();
   
}

void loop() 
{
  
  //static float mv_dir[2];
  // Try to get sensor data, but don't block. Returns from the function and proceeds on "old" data.
  bool b_newMpuData = readout_mpu_ypr(ypr);
 
  Serial.print("Yaw: ");
  Serial.print(ypr[0]);
  Serial.print(", Pitch: ");
  Serial.print(ypr[1]);
  Serial.print(", Roll: ");
  Serial.println(ypr[2]);
  
/*
  pitchInput = (double)ypr[1];
  rollInput = (double)ypr[2];

  
  /*
  Serial.print("PitchOut: "); Serial.print(pitchOutput);
  Serial.print(", RollOut: "); Serial.println(rollOutput);
  */
  /*
  double resultspeed = (fabs(rollOutput) + fabs(pitchOutput))/2;
  double magnitude = sqrt((pitchOutput * pitchOutput )+ (rollOutput*rollOutput));

  Serial.print("resultspeed: ");
  Serial.println(resultspeed);
  */
  /*
  Serial.print("magnitude: ");
  Serial.println(magnitude);
  */
  /*
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
