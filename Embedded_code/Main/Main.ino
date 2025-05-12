#include <Arduino.h>
#include "MotorControl.h"
#include "MPU6050_YPR.h"
#include "DataReceiver.h"
#include "utils.h"

// --- Hardware and configuration parameters ---
const int MOTOR_PUL_PINS[] = {3, 4, 5};
const int MOTOR_DIR_PINS[] = {6, 7, 8};

const int MICROSTEP = 8;
const float MOTOR_STEP_ANGLE = 1.8f;
const float WHEEL_SIZE = 100.0f; // in mm (diameter)
const float STEP_SIZE = MOTOR_STEP_ANGLE / MICROSTEP;

float motor_delay_min = 2000;
float motor_delay_max = 7500;

// --- MPU 6050 --- 
float ypr[3];

// --- multithreading config --- 
#define CORE_0 0
#define CORE_1 1

// --- mode parameters ---
float mode = -2.0;

// - PID mode
float pitchSetpoint = 0;
float rollSetpoint = 0; 
float max_angle = 5.0f;
float kd = 0.0, ki = 0.0;

float prevPitchError = 0;
float prevRollError = 0;
float integralPitch = 0;
float integralRoll = 0;
unsigned long lastUpdateTime = 0;

// - Manual mode
float speed_dir = 0.0f;
float dir_x = 0.0f;
float dir_y = 0.0f;

// - Single Wheel mode
float speed_motor0 = 0.0;
float speed_motor1 = 0.0;
float speed_motor2 = 0.0;

// --- functions ---
// Helper functions


// Multithreading Tasks
void TaskReceive(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    receiveLoop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskStatusLED(void *pvParameters) {
  (void)pvParameters;

  for(;;) {
    if (mode == -2.0) {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (mode == -1.0) {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000));
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (mode == 0.0) {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    
    if (mode == 1.0) {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (mode == 2.0) {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskMotorControl(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    vTaskDelay(1);  
    readout_mpu_ypr(ypr);
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0f; // seconds
    lastUpdateTime = currentTime;

    // PID
    if (mode == 0.0) {
      float pitchError = pitchSetpoint - ypr[1];
      float rollError  = rollSetpoint  - ypr[2];

      // integrate
      integralPitch += pitchError;
      integralRoll  += rollError;

      // derivative
      float dPitch = (pitchError - prevPitchError) / deltaTime;
      float dRoll  = (rollError  - prevRollError) / deltaTime;
      prevPitchError = pitchError;
      prevRollError  = rollError;

      // dynamic proportional factor based on max_angle
      float pFactorPitch = constrain(fabs(pitchError) / (max_angle * (PI / 180.0f)), 0.0f, 1.0f);
      float pFactorRoll  = constrain(fabs(rollError)  / (max_angle * (PI / 180.0f)), 0.0f, 1.0f);

      // full PID outputs
      float outPitch = constrain(pFactorPitch * sign(pitchError) + ki * integralPitch + kd * dPitch, -1.0f, 1.0f);
      float outRoll  = constrain(pFactorRoll  * sign(rollError)  + ki * integralRoll  + kd * dRoll, -1.0f, 1.0f);

      float magnitude = sqrt(outPitch * outPitch + outRoll * outRoll);
      float speed = max(fabs(outPitch), fabs(outRoll));

      if (magnitude > 0) {
        outPitch /= magnitude;
        outRoll /= magnitude;
      }

      if (magnitude < 0.1f){
        outPitch = 0.0f;
        outRoll = 0.0f;
        speed = 0.0f;
      }

      move(outPitch, outRoll, speed);
      continue;
    }

    // MANUAL
    else if (mode == 1.0) {
      move(dir_x, dir_y, speed_dir);
      continue;
    }

    // SINGLE WHEEL
    else if (mode == 2.0) {
      rotateWheel(fabs(speed_motor0), speed_motor0 < 0 ? false : true, 0);
      rotateWheel(fabs(speed_motor1), speed_motor1 < 0 ? false : true, 1);
      rotateWheel(fabs(speed_motor2), speed_motor2 < 0 ? false : true, 2);
      continue;
    }
  }
}

// Main Functions
void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Led -> red
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_PUL_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
  }

  setup_MPU6050(false);

  // Register Data Receiver Values
  registerAdress(&motor_delay_min, "min_delay");
  registerAdress(&motor_delay_max, "max_delay");

  registerAdress(&mode, "mode");

  registerAdress(&max_angle, "pid_max_angle");
  registerAdress(&ki, "pid_ki");
  registerAdress(&kd, "pid_kd");

  registerAdress(&pitchSetpoint, "pitch_setpoint");
  registerAdress(&rollSetpoint, "roll_setpoint");

  registerAdress(&speed_dir, "speed");
  registerAdress(&dir_x, "direction_x");
  registerAdress(&dir_y, "direction_y");

  registerAdress(&speed_motor0, "wheel1_speed");
  registerAdress(&speed_motor1, "wheel2_speed");
  registerAdress(&speed_motor2, "wheel3_speed");

  // Run Tasks
  xTaskCreatePinnedToCore(
    TaskMotorControl, "Task Motor Control", // A name just for humans
    2048, // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    NULL, // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    1, // Priority
    NULL, // Task handle is not used here - simply pass NULL
    CORE_0  // Core on which the task will run
  );

  xTaskCreatePinnedToCore(
    TaskReceive, "Task Receive", // A name just for humans
    2048, // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    NULL, // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    1, // Priority
    NULL, // Task handle is not used here - simply pass NULL
    CORE_1  // Core on which the task will run
  );

  xTaskCreatePinnedToCore(
    TaskStatusLED, "Task Status LED", // A name just for humans
    2048, // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    NULL, // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    2, // Priority
    NULL, // Task handle is not used here - simply pass NULL
    CORE_1  // Core on which the task will run
  );

  mode = -1.0;
  Serial.println("Setup Ready!");
}

void loop() { yield(); }