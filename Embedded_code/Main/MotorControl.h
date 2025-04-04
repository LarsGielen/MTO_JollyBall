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
extern long motor_timers[];

extern bool motor_flags[];

// Returns the delay (in milliseconds) per step for a given wheel speed (in m/s).
int wheelDelay(float wheelSpeed);

// Convert a desired movement vector (x, y) into wheel velocities.
// The wheel arrangement is assumed to be at 0°, 120°, and 240°.
void direction2wheelVelocity(const float moveDirection[2], float wheelVel[3]);

// Rotates a given wheel at a specified speed and direction.
// 'speed' is the absolute speed (in m/s) and 'dir' indicates the direction.
void rotateWheel(float speed, bool dir, int wheelIndex);

// Takes a 2-element array for movement direction (x and y in m/s)
// and updates all wheels accordingly.
void move(const float moveDirection[2], float speedPercent);

void printMotorState();

#endif
