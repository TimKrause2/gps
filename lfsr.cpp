#include "lfsr.h"
#include <iostream>


LFSR::LFSR(unsigned long poly0, unsigned long reg0)
{
    poly = poly0;
    reg = reg0;
    LFSR::reg0 = reg0;
    mask = 0;
    unsigned long bit = 1;
    while(poly0){
        if(bit==1){
            msb = 1;
        }else{
            msb<<=1;
        }
        mask |= bit;
        bit<<=1;
        poly0>>=1;
    }
}

int LFSR::advance(void){
    int r = read();
    unsigned long acc;
    acc = poly & reg;
    unsigned long f=0;
    while(acc){
        f^=(acc&1);
        acc>>=1;
    }
    reg<<=1;
    reg&=mask;
    reg|=f;
    //std::cout << "reg:" << reg << std::endl;
    return r;
}

int LFSR::read(void)
{
    return (reg&msb)?1:0;
}

unsigned long LFSR::period(void)
{
    reg = reg0;
    unsigned long count = 0;
    do{
        count++;
        advance();
    }while(reg!=reg0);
    return count;
}


unsigned long G2::ph_bit(int ph)
{
    unsigned long r=1;
    ph--;
    for(int b=0;b<ph;b++){
        r<<=1;
    }
    return r;
}


int G2::read(void)
{
    int ph1 = (reg&ph_bit1)?1:0;
    int ph2 = (reg&ph_bit2)?1:0;
    return ph1^ph2;
}

Channel channels[32] = {
    {2, 6,  5},   // 1
    {3, 7,  6},   // 2
    {4, 8,  7},   // 3
    {5, 9,  8},   // 4
    {1, 9,  17},  // 5
    {2, 10, 18},  // 6
    {1, 8,  139}, // 7
    {2, 9,  140}, // 8
    {3, 10, 141}, // 9
    {2, 3,  251}, // 10
    {3, 4,  252}, // 11
    {5, 6,  254}, // 12
    {6, 7,  255}, // 13
    {7, 8,  256}, // 14
    {8, 9,  257}, // 15
    {9, 10, 258}, // 16
    {1, 4,  469}, // 17
    {2, 5,  470}, // 18
    {3, 6,  471}, // 19
    {4, 7,  472}, // 20
    {5, 8,  473}, // 21
    {6, 9,  474}, // 22
    {1, 3,  509}, // 23
    {4, 6,  512}, // 24
    {5, 7,  513}, // 25
    {6, 8,  514}, // 26
    {7, 9,  515}, // 27
    {8, 10, 516}, // 28
    {1, 6,  859}, // 29
    {2, 7,  860}, // 30
    {3, 8,  861}, // 31
    {4, 9,  862}  // 32
};

CA::CA(int channel):
    g2(channels[channel-1].ph1, channels[channel-1].ph2)
{}

int CA::advance(void)
{
    return g1.advance() ^ g2.advance();
}

CA_t::CA_t(int channel, float fs):
    ca(channel)
{
    ca_t.reset(new float[1023*2]);
    for(int i=0,j=0;i<1023;i++){
        float x = (ca.advance())?1.0f:-1.0f;
        ca_t[j++] = x;
        ca_t[j++] = x;
    }
    float f = 1023*2/1e-3f;
    dperiod = f/fs;
    period = 0.0;
    index = 0;
    epoch_cb = nullptr;
}

float CA_t::evaluate(int offset)
{
    int i = (index + offset)%(1023*2);
    float r = ca_t[i];
    return r;
}

void CA_t::advance(void)
{
    period += dperiod;
    if(period>=1.0f){
        period-=1.0f;
        index++;
        if(index==(1023*2)){
            index = 0;
            if(epoch_cb){
                epoch_cb(epoch_cb_data);
            }
        }
    }
}

void CA_t::set_epoch_cb(void(*cb)(void *), void *data)
{
    epoch_cb = cb;
    epoch_cb_data = data;
}
