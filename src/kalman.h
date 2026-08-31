#pragma once
#include <Arduino.h>

// Constants are completely safe in header files
const float ACCEL_SCALE = 16384.0; // For +/- 2g
const float GYRO_SCALE = 32.768;   // For +/- 1000 dps

const float Q_angle = 0.001; // Physics Noise, mostly assuming drift is constant
const float Q_gyro  = 0.003; // Gyro white noise
const float R_angle = 0.1;  // Expected variance of the accelerometer noise

// Function declarations
void updateKalman(float ay, float az, float gx, float dt, float &filtered_angle, float &filtered_rate, 
                  float &smoothed_rate);
