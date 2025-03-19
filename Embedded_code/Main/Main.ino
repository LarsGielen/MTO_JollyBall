#include "MPU6050_PID_YPR.h"
#include <math.h>
  float xy[2] = {1 , 0};
const int SPEED_MS = 1;
float ypr[3];
float mv_dir[2];
// wheels
enum Wheel {wheel1, wheel2, wheel3};
const float STEP_ANGLE_DEG = 1.8;
const int MICROSTEP = 8;
const float WHEEL_SIZE_MM = 100;
const float STEP_SIZE = STEP_ANGLE_DEG / MICROSTEP;
int pulse_pins[3] = {3,4,5};
int dir_pins[3] = {6,7,8};
long timer_c[3];
bool flag_wheel[3];
// wheels

std::array<float,3> dirtowheelvelocity(float mvdir[2]);

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


std::array<float,3> dirtowheelvelocity(float mvdir[2])
{
  
  return {   
    mvdir[1],
  (-0.5*mvdir[0]+sqrt(3)/2 * mvdir[1]),
  (-0.5*mvdir[0]-sqrt(3)/2 * mvdir[1])
  };
}

int checksignage(float number)
{
  if(number >= 0){
    return 1;
  }
  else return -1;
}

float calculatewheeldelay(float wheelspeed)
{
  float wheelCircumference = PI * WHEEL_SIZE_MM;
  float rev_per_s = wheelspeed * 1000.0 / wheelCircumference;
  float steps_per_s = rev_per_s * (360 / STEP_SIZE);
  float period_per_step_s = 1.0 / steps_per_s;
  float period_per_step_us = period_per_step_s * 1000000.0;
  
  return period_per_step_us;
}

void rotatewheel(float delay_us,bool dir, int wheel)
{
  digitalWrite(dir_pins[(int)wheel], dir);
  float wheelspeeddelay = calculatewheeldelay(.1);

  if (timer_c[(int)wheel] - millis() >= (delay_us/2) )
  {
    digitalWrite(pulse_pins[(int)wheel], flag_wheel[(int) wheel]);
    timer_c[(int)wheel] =  millis();
    flag_wheel[(int) wheel] = !flag_wheel[(int) wheel];
  }
 
}

void loop (){

  std::array<float,3> var = dirtowheelvelocity(xy);
  for(int i = 0; i < 3; i++){
    rotatewheel(calculatewheeldelay(abs(var[i])), var[i]> 0 ? 1 : 0 ,i);
     Serial.print("wheel1: ");
      Serial.println(var[0]);
       Serial.print("wheel2 ");
  Serial.println(var[1]);
   Serial.print("wheel3: ");
  Serial.println(var[2]);
  }
}
