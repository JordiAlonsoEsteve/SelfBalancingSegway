# include "LQR.h"
# include <math.h>
LQR::LQR() {
}

void LQR::computeControl(float state[6], float &u_left, float &u_right) {
    u_left = 0.0f;
    u_right = 0.0f;

    if (abs(state[0]) > 0.43) {
        u_left = 0.0f;
        u_right = 0.0f;
    }else{
        for (int i = 0; i < 6; ++i) {
        u_left  -= K[0][i] * state[i] * 255.0f; // Scale to PWM range
        u_right -= K[1][i] * state[i] * 255.0f; // Scale to PWM range
        }
    }
}