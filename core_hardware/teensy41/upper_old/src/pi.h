#pragma once

class PID {
    float kp_, ki_;
    float integral_;
    float dt_;
    float error_prev_;

   public:
    PID(float kp, float ki,float dt) : dt_(dt), error_prev_(0.0){
      reset();
      setGain(kp,ki);
    }

    float update(float error, float limit) {
      float output, potential, integral_diff;
      // P制御
      potential = kp_ * error;
      // 台形積分
      integral_diff = ki_ * 0.5f * (error + error_prev_) * dt_;
      error_prev_ = error;

      // P制御 + I制御
      output = potential + integral_ + integral_diff;
      
      // antiwindup - limit the output
      if(output > limit){
        if(integral_diff < 0){
          integral_ += integral_diff;
        }
        return limit;
      }else if(output < -limit){
        if(integral_diff > 0){
          integral_ += integral_diff;
        }
        return -limit;
      }
      integral_ += integral_diff;
      return output;
    }

    void reset(){
      integral_ = 0;
      error_prev_ = 0;
    }
    void setGain(float kp, float ki){
      kp_ = kp;
      ki_ = ki;
    }
};
