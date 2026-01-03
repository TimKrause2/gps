#pragma once

#include <memory>

class LFSR
{
protected:
    unsigned long poly;
    unsigned long reg0;
    unsigned long reg;
    unsigned long mask;
    unsigned long msb;
public:
    LFSR(unsigned long poly, unsigned long reg0);
    int advance(void);
    virtual int read(void);
    unsigned long period(void);
};

class G1 : public LFSR
{
public:
    G1(unsigned long poly=0x204, unsigned long reg0 = 0x3FF)
        :LFSR(poly,reg0)
    {
    }
};

class G2 : public LFSR
{
    unsigned long ph_bit1;
    unsigned long ph_bit2;
    unsigned long ph_bit(int ph);
public:
    G2(int ph1, int ph2, unsigned long poly=0x3a6, unsigned long reg0=0x3FF):
        LFSR(poly, reg0)
    {
        ph_bit1 = ph_bit(ph1);
        ph_bit2 = ph_bit(ph2);
    }
    int read(void);
};

struct Channel
{
    int ph1;
    int ph2;
    int delay;
};

class CA
{
private:
    G1 g1;
    G2 g2;
public:
    CA(int channel);
    int advance();
};

class CA_t
{
    CA ca;
    std::unique_ptr<float[]> ca_t;
    int index;
    float period;
    float dperiod;
    void (*epoch_cb)(void *data);
    void *epoch_cb_data;
public:
    CA_t(int channel, float fs);
    float evaluate(int offset);
    void advance(void);
    void set_epoch_cb(void(*epoch_cb)(void *), void *data);
};


