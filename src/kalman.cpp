#include "kalman.h"

// State variables are kept private inside this file using static
static float kalman_angle = 0.0;
static float kalman_bias = 0.0;
static float kalman_rate = 0.0;

static float P[2][2] = {
  {1.0, 0.0},
  {0.0, 1.0}
};

static unsigned long last_time = 0;

// Now takes references so it can update both angle and rate cleanly
void updateKalman(float ay, float az, float gx, float dt, float &filtered_angle, float &filtered_rate, 
float &smoothed_rate) {
  static float prev_smoothed_rate = 0.0;
  float alpha = 0.2;

  unsigned long current_time = micros();
  if (last_time == 0) last_time = current_time; 
  // float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  // if (dt > 0.1) dt = 0.01;

  float accel_pitch = atan2(ay, az) * 57.29578;

  // --- PREDICT STEP ---
  float predicted_rate = gx - kalman_bias; 
  kalman_angle += dt * predicted_rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;

  // --- UPDATE STEP ---
  float innovation = accel_pitch - kalman_angle; 
  float S = P[0][0] + R_angle;

  float K_0 = P[0][0] / S;
  float K_1 = P[1][0] / S;

  kalman_angle += K_0 * innovation; 
  kalman_bias  += K_1 * innovation; // The filter learns and corrects the gyro drift here!

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K_0 * P00_temp;
  P[0][1] -= K_0 * P01_temp;
  P[1][0] -= K_1 * P00_temp;
  P[1][1] -= K_1 * P01_temp;

  // --- POST-UPDATE CORRECTION ---
  // Recalculate the rate using the *newly corrected* bias
  kalman_rate = gx - kalman_bias;

  // Pass results back to main.cpp
  filtered_angle = kalman_angle;
  filtered_rate = kalman_rate;
  smoothed_rate = (alpha * kalman_rate) + ((1.0 - alpha) * prev_smoothed_rate);
  prev_smoothed_rate = smoothed_rate;
}
