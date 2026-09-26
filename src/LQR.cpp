#include "LQR.h"
#include <Arduino.h>
LQR::LQR() {
}

void LQR::computeControl(float state[6], float &u_left, float &u_right) {
    u_left = 0.0f;
    u_right = 0.0f;

    if (fabs(state[0]) > 0.43) {
        u_left = 0.0f;
        u_right = 0.0f;
    }else{
        for (int i = 0; i < 6; ++i) {
        u_left  -= K[0][i] * state[i];
        u_right -= K[1][i] * state[i];
        }
        u_left *= 100; // scale to PWM range
        u_right *= 100; // scale to PWM range
    }
}