#include "dco.h"
#include <math.h>
using namespace std::complex_literals;

DCO::DCO(float fs):
    fs(fs)
{
    theta = 0.0f;
    f = 0.0f;
    dtheta = 0.0f;
}

std::complex<float> DCO::evaluate(void)
{
    float r,i;
    sincosf(theta, &i, &r);
    std::complex<float> t(r,i);
    theta+=dtheta;
    if(theta>=2*M_PI){
        theta-=2*M_PI;
    }else if(theta < 0.0){
        theta+=2*M_PI;
    }
    return t;
}

void DCO::set_frequency(float f)
{
    DCO::f = f;
    dtheta = 2*M_PI*f/fs;
}

float DCO::get_frequency(void)
{
    return f;
}

void DCO::add_frequency(float offset)
{
    set_frequency(f + offset);
}

void DCO::reset(void)
{
    theta = 0.0f;
}

void DCO::advance_phase(float dphase)
{
    theta += dphase;
    if(theta<0){
        theta+=2*M_PI;
    }else if(theta>=2*M_PI){
        theta-=2*M_PI;
    }
}
