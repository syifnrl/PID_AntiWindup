#ifndef PID_h
#define PID_h

#include <mbed.h>

struct lpf{
    float input[2];
    float integral;
    float coeff;
    float output;
};

class PID{
    public:
    PID();

    void enableAntiWindUp();
    void disableAntiWindUp();
    void setOutputLimits(double outMin, double outMax);
    void setInputLimits(double inMin, double inMax);
    void setInput(double in);
    void setGain(double p, double i, double d);
    double compute(double setpoint, float ts);
    double compute_filter(double setpoint, float ts, float n);

    private:
    double map(double in, double inMin, double inMax, double outMin, double outMax);
    lpf filter;
    double _setpoint;
    double k[3];
    double e[4];
    bool integralClamp;
    int _ts;
    double propotional, integral, differential;
    double out, lastOut, pwm_out;
    double _outMin, _outMax;
    double _inMin, _inMax;
    double _input;
    Timer t;
    volatile float nowT, prevT, dt;
    bool antiWindUp;
};
#endif