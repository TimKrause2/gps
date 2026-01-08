/*
 *
 *
 *
 *
 */

#include "triangulate.h"

#define PREAMBLE 0x8B

#define BITS_PER_WORD 30
#define WORD_MASK 0x3FFFFFFF
#define DATA_MASK 0x3FFFFFC0
#define PARITY_MASK 0x3F
#define WORDS_PER_SUBFRAME 10
#define BITS_PER_SUBFRAME (BITS_PER_WORD*WORDS_PER_SUBFRAME)
#define SUBFRAMES_PER_FRAME 5
#define N_PAGES 25

#define mu_earth (3.986005e14)
#define Omega_dot_e (7.2921151467e-5)
#define speed_of_light (2.99792458e8)

enum WordID
{
    WORD1,
    WORD2,
    WORD3,
    WORD4,
    WORD5,
    WORD6,
    WORD7,
    WORD8,
    WORD9,
    WORD10
};

#define WORD_TLM WORD1
#define WORD_HOW WORD2

enum SubframeID
{
    SUBFRAME1,
    SUBFRAME2,
    SUBFRAME3,
    SUBFRAME4,
    SUBFRAME5
};

struct TLM_HOW
{
    long sample_index;
    int tlm_message;
    int integrity_status;
    int time_of_week;
    int alert_flag;
    int anti_spoof;
};

struct SV // Space Vehicle
{
    int  health;      // sf5 word5 17 24
    double e;         // sf5 word3 9 24 unsigned sfe -21
    double t_oa;      // sf5 word4 1 8  unsigned sfe 12
    double delta_i;   // sf5 word4 9 24 signed sfe -19
    double Omega_dot; // sf5 word5 1 16 signed sfe -38
    double sqrt_A;    // sf5 word6 1 24 unsigned sfe -11
    double Omega_0;   // sf5 word7 1 24 signed sfe -23
    double omega;     // sf5 word8 1 24 signed sfe -23
    double M_0;       // sf5 word9 1 24 signed sfe -23
    double a_f0;      // sf5 word10 1 8  word10 20 22 signed sfe -20
    double a_f1;      // sf5 word10 9 19 signed sfe -38
};

struct LNAV
{
    int bit_select_mask[BITS_PER_WORD];
    int bit_isolate_mask[BITS_PER_WORD];
    int bit_fill_mask[BITS_PER_WORD+1];

    int subframe_raw[WORDS_PER_SUBFRAME];
    int subframe_decoded[WORDS_PER_SUBFRAME];
    int frame[SUBFRAMES_PER_FRAME][WORDS_PER_SUBFRAME];
    int pages[2][N_PAGES];

    // Subframe 1 decoded data
    int WeekNumber;      // sf1 word3 1 10
    int CodeOnL2Channel; // sf1 word3 11 12
    int URA;             // sf1 word3 13 16
    int HealthBad;       // sf1 word3 17 17
    int HealthCode;
    int IODC;
    int L2PCode;
    double T_GD; //                         EstimatedGroupDelayDifferential;
    double t_OC;
    double a_f2;
    double a_f1;
    double a_f0;

    // Subframe 2 and 3
    int AODO;       // sf2 word10 18 22
    int FitInterval;// sf2 word10 17
    // M_0 - Mean Anomoly at Reference Time
    double M_0;      // sf2 word4 17 24 word5 1 24 signed sfe -31
    // delta_n - Mean Motion Difference From Computed Value
    double delta_n;  // sf2 word4 1 16 signed sfe -43
    // e - Eccentricity
    double e;        // sf2 word6 17 24 word7 1 24 unsigned sfe -33
    // sqrt_A - Square Root of the Semi-Major Axis
    double sqrt_A;   // sf2 word8 17 24 word9 1 24 unsigned sfe -19
    // Omega_0 - Longitude of Ascending Node of Orbit Plane at Weekly Epoch
    double Omega_0;  // sf3 word3 17 24 word4 1 24 signed sfe -31
    // i_0 - Inclination Angle at Referece Time
    double i_0;      // sf3 word5 17 24 word6 1 24 signed sfe -31
    // omega - Argument of Perigee
    double omega;    // sf3 word7 17 24 word8 1 24 signed sfe -31
    // Omega_dot - Rate of Right Ascension
    double Omega_dot;// sf3 word9 1 24 signed sfe -43
    // IDOT - Rate of Inclination Angle
    double IDOT;     // sf3 word10 9 22 signed sfe -43
    // C_uc - Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
    double C_uc;     // sf2 word6 1 16 signed sfe -29
    // C_us - Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude
    double C_us;     // sf2 word8 1 16 signed sfe -29
    // C_rc - Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    double C_rc;     // sf3 word7 1 16 signed sfe -5
    // C_rs - Amplitude of the Sine Harmonic Correction Term to the Orbit Radius
    double C_rs;     // sf2 word3 9 24 signed sfe -5
    // C_ic - Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
    double C_ic;     // sf3 word3 1 16 signed sfe -29
    // C_is - Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination
    double C_is;     // sf3 word5 1 16 signed sfe -29
    // t_oe - Reference Time Ephemeris
    double t_oe;     // sf2 word10 1 16 unsigned sfe 4
    // IODE - Issue of Data (Ephemeris)
    int IODE2;      // sf2 word3 1 8
    int IODE3;      // sf3 word10 1 8

    TLM_HOW tlm_how[SUBFRAMES_PER_FRAME];
    SV svs[32];

    LNAV(void);
    int bit_parity(int Dlast, int d, int poly);
    int bit_select(int word, int bit);
    void bit_set(int &word, int bit, int x);
    int word_read(int word, int bit1, int bit2);
    int sword_read(int word, int bit1, int bit2); // signed value read - sign extend
    void sign_extend(int &word, int sign_bit);
    double scale_factor(int factor_exponent);
    void word_copy(int src_word, int &dst_word, int src_bit1, int src_bit2, int shiftl);
    int word_parity(int D, int Dlast, int &d);
    bool word_validate(int D, int &Dlast, int &d);
    bool tlm_test(int D, int &polarity);
    void subframe_set_bit(int x, int bit);
    bool subframe_decode(int &subframe, long sample_index);
    void frame_decode(int &page);
    void subframe5_decode(SV *scv);
    void TLM_HOW_decode(int subframe, long sample_index);
    double E_k(double t);
    double t_k(double t);
    double gps_time(int subframe);
    SatelliteFix calculate_position(int subframe);
};
