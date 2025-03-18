#include "MPU6050_PID_YPR.h"
#include <math.h>

const int SPEED_MS = 1;

float ypr[3];

float mv_dir[2];
void setup() {
  setup_MPU6050();
}
/*
void loop() {
  readout_MPU6050(ypr);
  // print out the filtered values
  Serial.print("Yaw: ");
  Serial.print(ypr[0]);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(ypr[1]);
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.print("\t");
  Serial.print(ypr[2]);

  
  
  // X-Axis
  mv_dir[0] = cos(ypr[2]) * -checksignage(ypr[2]);
  // Z-Axis
  mv_dir[1] = cos(ypr[1]) * -checksignage(ypr[1]);
  Serial.print("\t");
  Serial.print("move direction X-axis: ");
  Serial.print(mv_dir[0]);
  Serial.print("\t");
  Serial.print("move direction Z-axis: ");
  Serial.println(mv_dir[1]);
  
}
*/
void loop (){
  
}

std::array<float,3> dirtowheelvelocity(float mvdir[2]){
  
  return {   
    mvdir[0],
  (-0.5*mvdir[0]+sqrt(3)/2 * mvdir[1]),
  (-0.5*mvdir[0]-sqrt(3)/2 * mvdir[1])
  };
}

int checksignage(float number){
  if(number >= 0){
    return 1;
  }
  else return -1;
}

void rotatewheels(float wheelspeed){
  
}
