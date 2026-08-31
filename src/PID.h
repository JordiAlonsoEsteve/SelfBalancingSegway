#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float minOutput, float maxOutput);

    // Compute when you ALREADY have the derivative (e.g. IMU gyro rate or encoder speed)
    float compute(float target, float current, float currentRate, float dt, bool angle = true);

    // Standard compute (calculates the derivative of the error internally)
    float compute(float target, float current, float dt);

    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void reset();

private:
    float _kp, _ki, _kd;
    float _minOutput, _maxOutput;
    float _integral;
    float _lastError;
    bool _firstRun;
};

#endif // PID_H