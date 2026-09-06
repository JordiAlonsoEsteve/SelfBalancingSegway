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
  {-8.9622f, -0.7619f, -0.3097f, 0.0882f, 0.0359f, 0.2745f},
  {-9.4309f, -0.8007f, -0.0266f, 0.1336f, -0.3102f, 0.2444f},
};
};
#endif // LQR_H
