#pragma once

#include <Arduino.h>

// --- Pin Definitions ---
// It is best practice to use #define for pins in the header rather than 'int' 
// in the .cpp file. This saves RAM and lets your main.cpp know which pins are taken!
#define PIN_AIN1 19
#define PIN_AIN2 18
#define PIN_STBY 17
#define PIN_BIN1 33
#define PIN_BIN2 25

// --- Configuration Struct ---
struct MotorConfig {
  // 1. DEADZONE AVOIDANCE (Overcoming Static Friction)
  int minPWM_Left = 35;  
  int minPWM_Right = 35;

  // 2. ASYMMETRY CORRECTION (Driving Straight)
  float trimLeft = 1.00f;
  float trimRight = 1.00f;

  // 3. HARDWARE LIMITS
  int maxPWM = 254;

  // 4. THE MATH-TO-PHYSICS BRIDGE
  float u_max = 100.0f; // This s the maximum control effort. Reducing 
  // this number will increase the effect of the Kp, for instance, since it will 
  // yield a higher proportion of the maximum control effort for the same error.
};

// --- Public Function Prototypes ---
// These are the functions that your main loop or PID controller is allowed to call.

void setMotorOutputs(float leftCmd, float rightCmd, int &leftPWM, int &rightPWM);
void setMotorOutputsRaw(float leftCmd, float rightCmd, int &leftPWM, int &rightPWM);
void setDrive(float throttle, float steering);
