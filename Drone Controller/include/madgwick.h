#ifndef MADGWICK_H
#define MADGWICK_H

class madgwick{
  public:
    void begin(float sampleFreq, float beta);
    void update(float ax, float ay, float az, float gx, float gy, float gz);
    void getQuaternion(float *q);
  private:
    float beta;
    float q[4];
};

#endif