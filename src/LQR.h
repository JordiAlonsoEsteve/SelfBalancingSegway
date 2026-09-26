#ifndef LQR_H
#define LQR_H

class LQR {
public:
    LQR();
    // Add any public methods or member variables here
    void computeControl(float state[6], float &u_left, float &u_right);
    private:
    // Add any private methods or member variables here
    float K[2][6] = {
  {0.2418f, 0.1243f, 0.0537f, -0.3124f, 0.5092f, 0.2264f},
  {1.5079f, 0.1281f, 0.5858f, -0.0472f, -0.0825f, -0.5420f},
};
};
#endif // LQR_H
