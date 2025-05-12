#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

inline float sign(float value) {
  if (value >= 0) 
    return 1.0;
  else 
    return -1.0;
}
#endif