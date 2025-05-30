#include "MotorControl.h"

float lastWheelVel[3] = {0, 0, 0};

int wheelDelay(float percentSpeed) {
  const int MIN_DELAY = 2000;   // Minimum step delay (fastest speed) 2000
  const int MAX_DELAY = 100000; // Maximum step delay (slowest speed)  7500

  percentSpeed = constrain(percentSpeed, 0.0f, 1.0f);
  //Serial.println(MIN_DELAY + (MAX_DELAY - MIN_DELAY) * (1.0f-percentSpeed));
  return MIN_DELAY + (MAX_DELAY - MIN_DELAY) * (1.0f-percentSpeed);
}

void direction2wheelVelocity(const float moveDirection[2], float wheelVel[3]) {
  float phi = atan2(moveDirection[1], moveDirection[0]);
  const float wheelAngles[3] = { 0.0f, PI * 2/3, -PI * 2/3 };

  float raw[3];
  float maxVal = 0;

  for (int i = 0; i < 3; i++) {
    raw[i] = sin(wheelAngles[i] - phi);
    if (fabs(raw[i]) > maxVal) 
      maxVal = fabs(raw[i]);
  }

  if (maxVal < 1e-6) {
    for (int i = 0; i < 3; i++) 
      wheelVel[i] = 0;
  } 
  else {
    for (int i = 0; i < 3; i++) 
      wheelVel[i] = raw[i] / maxVal;
  }
}

void rotateWheel(float motorspeed, bool dir, int wheelIndex) {
  if(motorspeed <= 0.0f) return;
  float delayVal = wheelDelay(motorspeed);
  
  if (micros() - motor_timers[wheelIndex] >= (delayVal / 2.0f)) {
    digitalWrite(MOTOR_DIR_PINS[wheelIndex], dir);
    digitalWrite(MOTOR_PUL_PINS[wheelIndex], motor_flags[wheelIndex]);
    motor_timers[wheelIndex] = micros();
    motor_flags[wheelIndex] = !motor_flags[wheelIndex];
  }
}

void move(const float moveDirection[2], float speedPercent) {
  float wheelVel[3];
  direction2wheelVelocity(moveDirection, wheelVel);
  for (int i = 0; i < 3; i++) 
  {
    lastWheelVel[i] = wheelVel[i];
    bool dir = (wheelVel[i] >= 0);  
    float speed = fabs(wheelVel[i]) * speedPercent; 
    rotateWheel(speed, dir, i);
  }
}

void printMotorState() {
  Serial.println("Motor Status:");
  for (int i = 0; i < 3; i++) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(": Timer = ");
    Serial.print(motor_timers[i]);
    Serial.print(" | Flag = ");
    Serial.print(motor_flags[i] ? "HIGH" : "LOW");
    Serial.print(" | Speed = ");
    Serial.print(lastWheelVel[i], 4);
    Serial.println(" m/s");
  }
}
