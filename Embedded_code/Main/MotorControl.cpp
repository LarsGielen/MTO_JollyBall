#include "MotorControl.h"
#include "utils.h"

static float nextStepTime[3] = {0.0f, 0.0f, 0.0f};
static bool nextStepflag[3] = {false, false, false};

float wheelDelayMicros(float speedRatio) {
  speedRatio = constrain(speedRatio, 0.0f, 1.0f);
  // invert speedRatio so faster speed means shorter delay
  return motor_delay_min + (motor_delay_max - motor_delay_min) * (1.0f - speedRatio);
}

// https://modwg.co.uk/wp-content/uploads/2015/06/OmniRoller-Holonomic-Drive-Tutorial.pdf
void computeWheelSpeeds(float x, float y, float w, float speed, float& s1, float& s2, float& s3) {
  float theta1 = 0;          //   0 degrees
  float theta2 = 2 * PI / 3; // 120 degrees
  float theta3 = 4 * PI / 3; // 240 degrees

  // Calculate wheel speeds
  s1 = (-x * sin(theta1) + y * cos(theta1) + w);
  s2 = (-x * sin(theta2) + y * cos(theta2) + w);
  s3 = (-x * sin(theta3) + y * cos(theta3) + w);

  // Max normalization
  float maxVal = max(max(fabs(s1), fabs(s2)), fabs(s3));
  if (maxVal <= 1e-9f) {
    s1 = 0.0f;
    s2 = 0.0f;
    s3 = 0.0f;
  }
  else {
    s1 /= maxVal;
    s2 /= maxVal;
    s3 /= maxVal;
  }

  s1 *= speed;
  s2 *= speed;
  s3 *= speed;
}

void rotateWheel(float speedRatio, bool dir, int wheelIndex) {
  if (speedRatio <= 0.0f) return;
  float delayUs = wheelDelayMicros(speedRatio);
  unsigned long now = micros();

  if (nextStepTime[wheelIndex] == 0.0f) {
    nextStepTime[wheelIndex] = now + delayUs;
  }
  
  if ((float)now >= nextStepTime[wheelIndex]) {
    // toggle step
    digitalWrite(MOTOR_DIR_PINS[wheelIndex], dir);
    digitalWrite(MOTOR_PUL_PINS[wheelIndex], nextStepflag[wheelIndex]);
    nextStepflag[wheelIndex] = !nextStepflag[wheelIndex];

    // schedule next step
    nextStepTime[wheelIndex] += delayUs;

    // avoid drift: if behind, catch up
    if (nextStepTime[wheelIndex] < now) {
      nextStepTime[wheelIndex] = now + delayUs;
    }
  }
}

void move(const float moveDirection[2], float speedPercent) {
  move(moveDirection[0], moveDirection[1], speedPercent);
}

void move(float dir_x, float dir_y, float speedPercent) {
  float s1, s2, s3;
  computeWheelSpeeds(dir_x, dir_y, 0.0f, speedPercent, s1, s2, s3);

  rotateWheel(fabs(s1), s1 >= 0, 0);
  rotateWheel(fabs(s2), s2 >= 0, 1);
  rotateWheel(fabs(s3), s3 >= 0, 2);
}