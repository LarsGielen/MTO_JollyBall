#include "MPU6050_PID_YPR.h"
float ypr[3];

void setup(){
  setup_MPU6050();
  }

void loop(){
  readout_MPU6050(ypr);
  // print out the filtered values
            Serial.print("Yaw: ");
            Serial.print(ypr[0]);
            Serial.print("\t");
            Serial.print("Pitch: ");
            Serial.print(ypr[1]);
            Serial.print("\t");
            Serial.print("Roll: ");
            Serial.println(ypr[2]);
  }
