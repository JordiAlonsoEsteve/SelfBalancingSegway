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
  {1.6871f, -0.0759f, -0.6478f, 0.7176f, 0.4239f, -0.9204f},
  {2.6283f, -0.1214f, -2.0590f, -0.6398f, 2.0745f, 0.4609f},
};
};
#endif // LQR_H
