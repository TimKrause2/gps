#include "lnav.h"
#include <stdio.h>
#include <cmath>

LNAV::LNAV(void)
{
    int bit_select = 0x20000000;
    int bit_fill = 0x3FFFFFFF;
    for(int b=0;b<BITS_PER_WORD;b++){
        bit_select_mask[b] = bit_select;
        bit_isolate_mask[b] = (~bit_select)&WORD_MASK;
        bit_fill_mask[b] = bit_fill;
        bit_select>>=1;
        bit_fill>>=1;
    }
    bit_fill_mask[BITS_PER_WORD] = 0;
}


int LNAV::bit_parity(int Dlast, int d, int poly)
{
    int r = Dlast;
    int acc = d&poly;
    while(acc){
        r ^= (acc&1);
        acc>>=1;
    }
    return r;
}

int LNAV::bit_select(int word, int bit)
{
    int bit_s = bit_select_mask[bit-1];
    return (word&bit_s)?1:0;
}

int LNAV::word_read(int word, int bit1, int bit2)
{
    int mask = bit_fill_mask[bit1-1] ^ bit_fill_mask[bit2];
    int r = word & mask;
    r >>= BITS_PER_WORD - bit2;
    return r;
}

int LNAV::sword_read(int word, int bit1, int bit2)
{
    int mask = bit_fill_mask[bit1-1] ^ bit_fill_mask[bit2];
    int sign = (word&bit_select_mask[bit1-1])?1:0;
    int r = word & mask;
    r >>= BITS_PER_WORD - bit2;
    if(sign){
        mask >>= BITS_PER_WORD - bit2;
        r |= ~mask;
    }

    return r;
}

void LNAV::sign_extend(int &word, int Nbits)
{
    int sign_bit = BITS_PER_WORD - Nbits;
    int mask = bit_fill_mask[sign_bit];
    int sign = (word&bit_select_mask[sign_bit])?1:0;
    if(sign){
        word |= ~mask;
    }
}

double LNAV::scale_factor(int factor_exponent)
{
    return std::pow(2.0, (double)factor_exponent);
}

void LNAV::word_copy(int src_word, int &dst_word, int src_bit1, int src_bit2, int shiftl)
{
    int mask = bit_fill_mask[src_bit1-1] ^ bit_fill_mask[src_bit2];
    int r = src_word & mask;
    r >>= BITS_PER_WORD - src_bit2;
    mask >>= BITS_PER_WORD - src_bit2;
    r <<= shiftl;
    mask <<= shiftl;
    dst_word &= ~mask;
    dst_word |= r;
}

void LNAV::bit_set(int &word, int bit, int x)
{
    bit--;
    word &= bit_isolate_mask[bit];
    word |= (x)?bit_select_mask[bit]:0;
}

int LNAV::word_parity(int D, int Dlast, int &d)
{
    int D29last = bit_select(Dlast, 29);
    int D30last = bit_select(Dlast, 30);
    d = D & DATA_MASK;
    if(D30last){
        d = (~d) & DATA_MASK;
    }
    int D_result = 0;
    bit_set(D_result, 25, bit_parity(D29last, d, D25_POLY));
    bit_set(D_result, 26, bit_parity(D30last, d, D26_POLY));
    bit_set(D_result, 27, bit_parity(D29last, d, D27_POLY));
    bit_set(D_result, 28, bit_parity(D30last, d, D28_POLY));
    bit_set(D_result, 29, bit_parity(D30last, d, D29_POLY));
    bit_set(D_result, 30, bit_parity(D29last, d, D30_POLY));
    return D_result;
}

bool LNAV::word_validate(int D, int &Dlast, int &d)
{
    int D_parity = word_parity(D, Dlast, d);
    bool r;
    if(D_parity == (D&PARITY_MASK)){
        r = true;
    }else{
        r = false;
    }
    Dlast = D_parity;
    return r;
}

bool LNAV::tlm_test(int D, int &polarity)
{
    int Dlastp = 0;
    int Dlastn = 0;
    int d;
    if(word_validate(D, Dlastp, d)){
        int preamble = word_read(d, 1, 8);
        if(preamble==PREAMBLE){
            polarity = 0;
            return true;
        }
    }else if(word_validate((~D)&WORD_MASK, Dlastn, d)){
        int preamble = word_read(d, 1, 8);
        if(preamble==PREAMBLE){
            polarity = 1;
            return true;
        }
    }
    return false;
}

void LNAV::subframe_set_bit(int x, int bit)
{
    bit--;
    int word = bit/BITS_PER_WORD;
    int rem = bit%BITS_PER_WORD;
    int set_bit = rem + 1;
    bit_set(subframe_raw[word], set_bit, x);
}

bool LNAV::subframe_decode(int &subframe, long sample_index)
{
    int Dlast = 0;
    for(int w=0;w<WORDS_PER_SUBFRAME;w++){
        int d;
        if(!word_validate(subframe_raw[w], Dlast, d)){
            printf("invalid word:%d\n", w+1);
            return false;
        }else{
            subframe_decoded[w] = d;
        }
    }
    // test D29 and D30 for zero
    if(bit_select(Dlast, 29)!=0 || bit_select(Dlast, 30)!=0){
        printf("D29 D30 test failed.\n");
        return false;
    }
    subframe = word_read(subframe_decoded[1], 20, 22);
    // decode the TLM and HOW words
    TLM_HOW_decode(subframe, sample_index);

    // transfer this subframe to the frame
    for(int w=0;w<WORDS_PER_SUBFRAME;w++){
        frame[subframe-1][w] = subframe_decoded[w];
    }
    return true;
}

void LNAV::frame_decode(int &page)
{
    // Decode subframe 1
    int word3 = frame[SUBFRAME1][WORD3];
    int word4 = frame[SUBFRAME1][WORD4];
    int word7 = frame[SUBFRAME1][WORD7];
    int word8 = frame[SUBFRAME1][WORD8];
    int word9 = frame[SUBFRAME1][WORD9];
    int word10 = frame[SUBFRAME1][WORD10];
    WeekNumber      = word_read(word3, 1, 10);
    CodeOnL2Channel = word_read(word3, 11, 12);
    URA             = word_read(word3, 13, 16);
    HealthBad       = word_read(word3, 17, 17);
    HealthCode      = word_read(word3, 18, 22);
    IODC = 0;
    word_copy(word3, IODC, 23, 24, 8);
    word_copy(word8, IODC, 1, 8, 0);
    L2PCode = word_read(word4, 1, 1);
    T_GD    = sword_read(word7, 17, 24)*scale_factor(-31);
    t_OC    = word_read(word8, 9, 24)*scale_factor(4);
    a_f2    = sword_read(word9, 1, 8)*scale_factor(-55);
    a_f1    = sword_read(word9, 9, 24)*scale_factor(-43);
    a_f0    = sword_read(word10, 1, 22)*scale_factor(-32);

    // Subframe 2 and 3 orbital elements
    AODO = word_read(frame[SUBFRAME2][WORD10], 18, 22)*900;
    FitInterval = word_read(frame[SUBFRAME2][WORD10], 17, 17);
    int M_0_int = 0;
    word_copy(frame[SUBFRAME2][WORD4], M_0_int, 17, 24, 24);
    word_copy(frame[SUBFRAME2][WORD5], M_0_int, 1, 24, 0);
    M_0 = M_0_int*scale_factor(-31);
    delta_n = sword_read(frame[SUBFRAME2][WORD4], 1, 16)*scale_factor(-43);
    int e_int = 0;
    word_copy(frame[SUBFRAME2][WORD6], e_int, 17, 24, 24);
    word_copy(frame[SUBFRAME2][WORD7], e_int, 1, 24, 0);
    e = (unsigned int)(e_int)*scale_factor(-33);
    int sqrt_A_int = 0;
    word_copy(frame[SUBFRAME2][WORD8], sqrt_A_int, 17, 24, 24);
    word_copy(frame[SUBFRAME2][WORD9], sqrt_A_int, 1, 24, 0);
    sqrt_A = (unsigned int)(sqrt_A_int)*scale_factor(-19);
    int Omega_0_int = 0;
    word_copy(frame[SUBFRAME3][WORD3], Omega_0_int, 17, 24, 24);
    word_copy(frame[SUBFRAME3][WORD4], Omega_0_int, 1, 24, 0);
    Omega_0 = Omega_0_int*scale_factor(-31);
    int i_0_int = 0;
    word_copy(frame[SUBFRAME3][WORD5], i_0_int, 17, 24, 24);
    word_copy(frame[SUBFRAME3][WORD6], i_0_int, 1, 24, 0);
    i_0 = i_0_int*scale_factor(-31);
    int omega_int = 0;
    word_copy(frame[SUBFRAME3][WORD7], omega_int, 17, 24, 24);
    word_copy(frame[SUBFRAME3][WORD8], omega_int, 1, 24, 0);
    omega = omega_int*scale_factor(-31);
    Omega_dot = sword_read(frame[SUBFRAME3][WORD9], 1, 24)*scale_factor(-43);
    IDOT = sword_read(frame[SUBFRAME3][WORD10], 9, 22)*scale_factor(-43);
    C_uc = sword_read(frame[SUBFRAME2][WORD6], 1, 16)*scale_factor(-29);
    C_us = sword_read(frame[SUBFRAME2][WORD8], 1, 16)*scale_factor(-29);
    C_rc = sword_read(frame[SUBFRAME3][WORD7], 1, 16)*scale_factor(-5);
    C_rs = sword_read(frame[SUBFRAME2][WORD3], 9, 24)*scale_factor(-5);
    C_ic = sword_read(frame[SUBFRAME3][WORD3], 1, 16)*scale_factor(-29);
    C_is = sword_read(frame[SUBFRAME3][WORD5], 1, 16)*scale_factor(-29);
    IODE2 = word_read(frame[SUBFRAME2][WORD3], 1, 8);
    IODE3 = word_read(frame[SUBFRAME3][WORD10], 1, 8);

    // Subframe 5
    int sv_id = word_read(frame[SUBFRAME5][WORD3], 3, 8);
    if(sv_id<=24){
        page = sv_id;
        subframe5_decode(&svs[sv_id-1]);
    }else{
        page = 25;
    }
}

void LNAV::subframe5_decode(SV *svp)
{
    svp->health =     word_read(frame[SUBFRAME5][WORD5], 17, 24);
    svp->e =          word_read(frame[SUBFRAME5][WORD3], 9, 24)*scale_factor(-21);
    svp->t_oa =       word_read(frame[SUBFRAME5][WORD4], 1, 8)*scale_factor(12);
    svp->delta_i =   sword_read(frame[SUBFRAME5][WORD4], 9, 24)*scale_factor(-19) + 0.30;
    svp->Omega_dot = sword_read(frame[SUBFRAME5][WORD5], 1, 16)*scale_factor(-38);
    svp->sqrt_A =     word_read(frame[SUBFRAME5][WORD6], 1, 24)*scale_factor(-11);
    svp->Omega_0 =   sword_read(frame[SUBFRAME5][WORD7], 1, 24)*scale_factor(-23);
    svp->omega =     sword_read(frame[SUBFRAME5][WORD8], 1, 24)*scale_factor(-23);
    svp->M_0 =       sword_read(frame[SUBFRAME5][WORD9], 1, 24)*scale_factor(-23);
    int a_f0_int = 0;
    word_copy(frame[SUBFRAME5][WORD10], a_f0_int, 1, 8, 3);
    word_copy(frame[SUBFRAME5][WORD10], a_f0_int, 20, 22, 0);
    sign_extend(a_f0_int, 11);
    svp->a_f0 = a_f0_int*scale_factor(-20);
    svp->a_f1 = sword_read(frame[SUBFRAME5][WORD10], 9, 19)*scale_factor(-38);
    if(svp->e<0.0 || svp->e>0.03){
        printf("SV e is out of range (0.0 to 0.03) e:%lf\n", svp->e);
    }
    if(svp->t_oa<0.0 || svp->t_oa>602112){
        printf("SV t_oa is out of range. (0 to 602112) t_oa:%lf\n", svp->t_oa);
    }
    if(svp->Omega_dot<-1.19e-7 || svp->Omega_dot>0.0){
        printf("SV Omega_dot is out range. (-1.19e-7 to 0) Omega_dot:%lf\n", svp->Omega_dot);
    }
    if(svp->sqrt_A<2530 || svp->sqrt_A>8192){
        printf("SV sqrt_A is out of range (2530 to 8192) sqrt_A:%lf\n", svp->sqrt_A);
    }
}

void LNAV::TLM_HOW_decode(int subframe, long sample_index)
{
    TLM_HOW *t = &tlm_how[subframe-1];
    t->sample_index = sample_index;
    t->tlm_message = word_read(subframe_decoded[WORD1], 9, 22);
    t->integrity_status = bit_select(subframe_decoded[WORD1], 23);
    t->time_of_week = word_read(subframe_decoded[WORD2], 1, 17);
    t->alert_flag = bit_select(subframe_decoded[WORD2], 18);
    t->anti_spoof = bit_select(subframe_decoded[WORD2], 19);
}

//
// IS-GPS-200  Table 20-IV
//
double LNAV::E_k(double t)
{
    double A = sqrt_A*sqrt_A;
    double n_0 = std::sqrt(mu/(A*A*A));
    double n = n_0 + delta_n;
    double M_k = M_0 + n*t_k(t);
    double E_j = M_k;
    for(int i=0;i<10;i++){
        E_j = E_j + (M_k - E_j +e*std::sin(E_j))/(1.0-e*cos(E_j));
    }
    return E_j;
}

double LNAV::t_k(double t)
{
    double t_k_r = t - t_oe;
    if(t_k_r > 302400.0){
        t_k_r -= 604800.0;
    }else if(t_k_r < -302400.0){
        t_k_r += 604800.0;
    }
    return t_k_r;
}

double LNAV::gps_time(int subframe)
{
    double t_sv = tlm_how[subframe-1].time_of_week*6;
    double delta_t_sv = 0.0;
    for(int i=0;i<2;i++){
        double t = t_sv - delta_t_sv;
        double delta_t_r = F*e*sqrt_A*std::sin(E_k(t));
        double dt = t - t_OC;
        delta_t_sv = a_f0 + a_f1*dt + a_f2*dt*dt + delta_t_r - T_GD;
        //printf("delta_t_sv:%.18le\n", delta_t_sv);
    }
    return t_sv - delta_t_sv;
}

SatelliteFix LNAV::calculate_position(int subframe)
{
    double t = gps_time(subframe);
    double v_k = 2.0*std::atan(std::sqrt((1+e)/(1-e))*std::tan(E_k(t)/2));
    double Phi_k = v_k + omega;
    double delta_u_k = C_us*std::sin(2*Phi_k) + C_uc*std::cos(2*Phi_k);
    double delta_r_k = C_rs*std::sin(2*Phi_k) + C_rc*std::cos(2*Phi_k);
    double delta_i_k = C_is*std::sin(2*Phi_k) + C_ic*std::cos(2*Phi_k);
    double u_k = Phi_k + delta_u_k;
    double A = sqrt_A*sqrt_A;
    double r_k = A*(1-e*std::cos(E_k(t))) + delta_r_k;
    double i_k = i_0 + delta_i_k + IDOT*t_k(t);
    double x_k_p = r_k*std::cos(u_k);
    double y_k_p = r_k*std::sin(u_k);
    double Omega_k = Omega_0 +(Omega_dot - Omega_dot_e)*t_k(t) - Omega_dot_e*t_oe;
    double x_k = x_k_p*std::cos(Omega_k) - y_k_p*std::cos(i_k)*std::sin(Omega_k);
    double y_k = x_k_p*std::sin(Omega_k) + y_k_p*std::cos(i_k)*std::cos(Omega_k);
    double z_k = y_k_p*std::sin(i_k);
    // printf("Satellite fix. x_k:%.7le y_k:%.7le z_k:%.7le\n",
    //         x_k, y_k, z_k);
    SatelliteFix fix;
    fix.gps_time = t;
    fix.sample_index = tlm_how[subframe-1].sample_index;
    fix.x_k = x_k;
    fix.y_k = y_k;
    fix.z_k = z_k;

    return fix;
}
