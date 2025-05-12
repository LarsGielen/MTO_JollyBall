#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

// External declarations for pins and configuration parameters:
extern const int MOTOR_PUL_PINS[];
extern const int MOTOR_DIR_PINS[];
extern const int MICROSTEP;
extern const float MOTOR_STEP_ANGLE;
extern const float WHEEL_SIZE; // in millimeters
extern const float STEP_SIZE;

extern float motor_delay_min;
extern float motor_delay_max;

const float M[3][3] = {
  { -0.33,  0.58,  0.33 },
  { -0.33, -0.58,  0.33 },
  {  0.67,  0.00,  0.33 }
};

// Returns the delay (in microseconds) per step for a given wheel speed (0..1).
float wheelDelayMicros(float wheelSpeed);

// Convert a desired movement vector (x, y) into wheel velocities.
// The wheel arrangement is assumed to be at 0°, 120°, and 240°.
void direction2wheelVelocity(float dir_x, float dir_y, float speed, float wheelVel[3]);

// Rotates a given wheel at a specified speed and direction.
// 'speed' is the absolute speed (in m/s) and 'dir' indicates the direction.
void rotateWheel(float speed, bool dir, int wheelIndex);

// Takes a 2-element array for movement direction (x and y in m/s)
// and updates all wheels accordingly.
void move(const float moveDirection[2], float speedPercent);
void move(float dir_x, float dir_y, float speedPercent);

#endif
