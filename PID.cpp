#include <mbed.h>
#include <PID.h>
#include <math.h>

PID::PID(){
    _setpoint = 0;
    k[0]=0;k[1]=0;k[2]=0;
    e[0]=0;e[1]=0;e[2]=0;e[3]=0;
    _ts = 0;
    out = 0;
    lastOut = 0;
    pwm_out = 0;
    _outMin = -24; _outMax = 24;
    _inMin = -1.0; _inMax = 1.0;
    nowT = 0; prevT = 0;
    integralClamp = false;
    antiWindUp = false;
    filter.integral = 0;
    filter.coeff = 0;
    filter.output = 0;
    filter.input[0] = 0;
    filter.input[1] = 0;
    t.reset();
    t.start();
    nowT = t.read_high_resolution_us();
}

void PID::enableAntiWindUp(){
    antiWindUp = true;
}

void PID::disableAntiWindUp(){
    antiWindUp = false;
}

void PID::setGain(double p, double i, double d){
    k[0] = p; k[1] = i; k[2] = d;
}

void PID::setOutputLimits(double outMin, double outMax){
    _outMin = outMin; _outMax = outMax;
}

void PID::setInputLimits(double inMin, double inMax){
    _inMin = inMin; _inMax = inMax;
}

double PID::map(double in, double inMin, double inMax, double outMin, double outMax){
  if (inMin<inMax) { 
    if (in <= inMin) 
      return outMin;
    if (in >= inMax)
      return outMax;
  } else {  // cope with input range being backwards.
    if (in >= inMin) 
      return outMin;
    if (in <= inMax)
      return outMax;
  }
  // calculate how far into the range we are
  double scale = ((double)in-inMin)/(inMax-inMin);
  // calculate the output.
  return outMin + scale*(outMax-outMin);
}

void PID::setInput(double in){
    _input = in;
}

double PID::compute(double setpoint, float ts){
    if(t.read_high_resolution_us() - nowT >= (ts*1.0e6)){
        e[0] = setpoint - _input;
        propotional = k[0] * e[0];

        if(antiWindUp){
            if(out >= _outMax && out * e[0] > 0)
                integralClamp = true;
            else if( out <= _outMin && out * e[0] < 0)
                integralClamp = true;
            else
                integralClamp = false;
        }

        if(integralClamp)
            e[1] = 0;
        else
            e[1] += e[0];
        
        integral = k[1] * e[1] * ts;

        e[2] = e[0] - e[3];
        differential = k[2] * e[2] / ts;

        out = propotional + integral + differential;
        e[3] = e[0];
        
        pwm_out = map(out, _outMin, _outMax, _inMin, _inMax);
        return pwm_out;
        lastOut = pwm_out;
        nowT = t.read_high_resolution_us();
    }

    else {
        return lastOut;
    }

}

double PID::compute_filter(double setpoint, float ts, float n){
    filter.coeff = n;

    if(t.read_high_resolution_us() - nowT >= (ts*1.0e6)){
        e[0] = setpoint - _input;
        propotional = k[0] * e[0];

        e[1] += e[0];
        integral = k[1] * e[1] * ts;
        
        if(antiWindUp){
            if((out * e[0]) > 0 || (out * e[0]) < 0){
                if(out >= _outMax || out <= _outMax)
                    integralClamp = true;
                else
                    integralClamp = false;
            }
            if(integralClamp)
                integral = 0;
        }
        
        e[2] = e[0] - e[3];
        differential = k[2] * e[2] / ts;
        filter.input[0] = differential;
        filter.output = filter.input[0] - filter.integral;

        out = propotional + integral + filter.output;
        e[3] = e[0];
        filter.input[1] = filter.input[0];
        filter.integral += filter.input[1] * filter.coeff;
        
        pwm_out = map(out, _outMin, _outMax, _inMin, _inMax);
        return pwm_out;
        lastOut = pwm_out;
        nowT = t.read_high_resolution_us();
    }

    else {
        return lastOut;
    }

}
