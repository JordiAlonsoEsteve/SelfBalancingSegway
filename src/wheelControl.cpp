#include "wheelControl.h"
#include <Arduino.h>
#include "wheelControl.h"

// Instantiate the config internally 
static MotorConfig config;

// Maps an unbounded input fraction [0.0 to 1.0+] to physical PWM
static int computeLinearizedPWM(float cmdFraction, int minPWM, float trim) {
  // Apply trim before deadband mapping
  float fraction = fabs(cmdFraction) * trim;
  
  // 1. HARD CLAMP: Saturate at 1.0 (100% effort)
  if (fraction > 1.0f) {
    fraction = 1.0f;
  }
  
  // 3. LINEAR MAP: Jump directly to minPWM, then scale up to maxPWM
  float pwm = minPWM + (fraction * (config.maxPWM - minPWM));
  
  return (int)pwm;
}

// Low-level independent control: Accepts UNBOUNDED raw left/right commands
void setMotorOutputs(float leftCmd, float rightCmd, int &leftPWM, int &rightPWM) {
  
  // Normalize the unbounded math inputs into a fraction
  // capped at 100% since it will saturate the PWM and breakdown.
  float leftFraction = min(leftCmd / config.u_max, 1.0f);
  float rightFraction = min(rightCmd / config.u_max, 1.0f);

  // Compute physical PWM 
  // (computeLinearizedPWM will handle clamping the 1.2 back down to 1.0 safely)
  // Note these are NOT signed, since the direction is handled by the H-bridge pins.
  leftPWM  = computeLinearizedPWM(leftFraction,  config.minPWM_Left,  config.trimLeft);
  rightPWM = computeLinearizedPWM(rightFraction, config.minPWM_Right, config.trimRight);

  // Actuate Left
  if (leftCmd > 0) {
    analogWrite(PIN_AIN1, 0);       analogWrite(PIN_AIN2, leftPWM);
  } else if (leftCmd < 0) {
    analogWrite(PIN_AIN1, leftPWM); analogWrite(PIN_AIN2, 0);
  } else {
    analogWrite(PIN_AIN1, 0);       analogWrite(PIN_AIN2, 0);
  }

  // Actuate Right
  if (rightCmd > 0) {
    analogWrite(PIN_BIN1, 0);       analogWrite(PIN_BIN2, rightPWM);
  } else if (rightCmd < 0) {
    analogWrite(PIN_BIN1, rightPWM); analogWrite(PIN_BIN2, 0);
  } else {
    analogWrite(PIN_BIN1, 0);       analogWrite(PIN_BIN2, 0);
  }
}

void setMotorOutputsRaw(float leftCmd, float rightCmd, int &leftPWM, int &rightPWM) {

  // Constraint to the hardware limits
  leftPWM  = constrain(leftPWM, 0, config.maxPWM);
  rightPWM = constrain(rightPWM, 0, config.maxPWM);
  // Actuate Left
  if (leftCmd > 0) {
    analogWrite(PIN_AIN1, 0);       analogWrite(PIN_AIN2, leftPWM);
  } else if (leftCmd < 0) {
    analogWrite(PIN_AIN1, leftPWM); analogWrite(PIN_AIN2, 0);
  } else {
    analogWrite(PIN_AIN1, 0);       analogWrite(PIN_AIN2, 0);
  }

  // Actuate Right
  if (rightCmd > 0) {
    analogWrite(PIN_BIN1, 0);       analogWrite(PIN_BIN2, rightPWM);
  } else if (rightCmd < 0) {
    analogWrite(PIN_BIN1, rightPWM); analogWrite(PIN_BIN2, 0);
  } else {
    analogWrite(PIN_BIN1, 0);       analogWrite(PIN_BIN2, 0);
  }
}

// High-level Drive: Mixes UNBOUNDED throttle and steering gracefully
void setDrive(float throttle, float steering) {
  
  float leftCmd  = throttle + steering;
  float rightCmd = throttle - steering;

  // --- PROPORTIONAL DESATURATION ---
  // If the LQR asks for extreme throttle AND extreme steering, 
  // adding them might exceed u_max. 
  // If we just clamp them individually, the robot loses its ability to turn!
  // Instead, we find the highest demand, and scale them BOTH down evenly.
  
  float maxVal = max(fabs(leftCmd), fabs(rightCmd));
  
  if (maxVal > config.u_max) {
    // Scale factor will be something like 0.8 to bring them back within bounds
    float scaleFactor = config.u_max / maxVal; 
    
    leftCmd  *= scaleFactor;
    rightCmd *= scaleFactor;
  }

  int leftPWM, rightPWM;
  setMotorOutputs(leftCmd, rightCmd, leftPWM, rightPWM);
}