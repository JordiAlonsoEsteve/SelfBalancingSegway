#pragma once
#include <Arduino.h>

// Sensor scaling factors 
const float ACCEL_SCALE = 16384.0; // For +/- 2g
const float GYRO_SCALE = 32.768;   // For +/- 1000 dps

// Filter tuning parameter (0.0 to 1.0)
// 0.98 means: trust gyro 98% (short term), trust accelerometer 2% (long term correction)
const float COMP_ALPHA = 0.98;

void updateComplementary(float ay, float az, float gx, float &filtered_angle, float &filtered_rate, float &smoothed_rate);