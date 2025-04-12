#ifndef PID_H
#define PID_H

class pid{
  public:
    void setTunings(float Kp, float Ki, float Kd);
    void setOutputLimits(float min, float max);
    void setMode(int mode);
    void setSampleTime(int newSampleTime);
    float compute(float input);
  private:
    float kp, ki, kd;
    float outputMin, outputMax;
    int mode;
    int sampleTime;
    float lastInput;
    float ITerm;
};

#endif