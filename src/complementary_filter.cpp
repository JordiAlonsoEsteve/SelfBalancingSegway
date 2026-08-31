#include "complementary_filter.h"

// State variables kept private
static float comp_angle = 0.0;
static unsigned long last_time = 0;

void updateComplementary(float ay, float az, float gx, float &filtered_angle, float &filtered_rate, float &smoothed_rate) {
  static float prev_smoothed_rate = 0.0;
  const float rate_alpha = 0.2; // Low-pass filter weight for smoothed_rate

  unsigned long current_time = micros();
  if (last_time == 0) last_time = current_time; 
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  // Prevent massive integration jumps on startup or delay()
  if (dt > 0.1 || dt <= 0.0) dt = 0.01;

  // --- 1. ACCELEROMETER ---
  // atan2 relies on a ratio, so raw unscaled ay/az values are mathematically fine here
  float accel_pitch = atan2(ay, az) * 57.29578;

  // --- 2. GYROSCOPE ---
  // Convert raw gyro reading into degrees per second (dps) for the derivative
  float gyro_rate = gx / GYRO_SCALE;

  // --- 3. COMPLEMENTARY FILTER ---
  // Integrate the gyro rate for the short term, pull slightly toward accel pitch for long-term drift correction
  comp_angle = COMP_ALPHA * (comp_angle + gyro_rate * dt) + (1.0 - COMP_ALPHA) * accel_pitch;

  // --- 4. OUTPUTS ---
  filtered_angle = comp_angle;
  filtered_rate = gyro_rate;
  
  smoothed_rate = (rate_alpha * gyro_rate) + ((1.0 - rate_alpha) * prev_smoothed_rate);
  prev_smoothed_rate = smoothed_rate;
}