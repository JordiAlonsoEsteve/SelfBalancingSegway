#include "PID.h"
#include <stdlib.h>

PID::PID(float kp, float ki, float kd, float minOutput, float maxOutput)
    : _kp(kp), _ki(ki), _kd(kd), _minOutput(minOutput), _maxOutput(maxOutput), 
      _integral(0.0f), _lastError(0.0f), _firstRun(true) {}

// For Balance Loop (Uses Gyro Rate / Encoder Speed directly)
float PID::compute(float target, float current, float currentRate, float dt, bool angle) {
    if (dt <= 0.0f) return 0.0f; 

    // If the angle is beyond 25 degrees, just return 0. 
    if (angle && abs(target - current) > 0.436332f) { // 25 degrees in radians
        return 0.0f;
    }
    float error = target - current;

    _integral += _ki * error * dt;
    if (_integral > _maxOutput) _integral = _maxOutput;
    else if (_integral < _minOutput) _integral = _minOutput;

    float D = -_kd * currentRate; 

    float output = (_kp * error) + _integral + D;

    if (output > _maxOutput) output = _maxOutput;
    else if (output < _minOutput) output = _minOutput;

    return output;
}

// For Standard Loops (Calculates derivative internally)
float PID::compute(float target, float current, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = target - current;

    if (_firstRun) {
        _lastError = error;
        _firstRun = false;
    }

    float derivative = (error - _lastError) / dt;
    _lastError = error;

    _integral += error * dt;
    if (_integral > _maxOutput) _integral = _maxOutput;
    else if (_integral < _minOutput) _integral = _minOutput;

    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    if (output > _maxOutput) output = _maxOutput;
    else if (output < _minOutput) output = _minOutput;

    return output;
}

void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setOutputLimits(float min, float max) {
    _minOutput = min;
    _maxOutput = max;
}

void PID::reset() {
    _integral = 0.0f;
    _firstRun = true;
}