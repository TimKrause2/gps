#pragma once

#include <complex>

class DCO
{
    float theta;
    float dtheta;
    float fs;
    float f;
public:
    DCO(float fs);
    std::complex<float> evaluate(void);
    void set_frequency(float f);
    float get_frequency(void);
    void add_frequency(float offset);
    void reset(void);
    void advance_phase(float dphase);
};
