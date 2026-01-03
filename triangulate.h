#pragma once

#include "queue.h"
#include <list>
#include <thread>
#include <memory>
#include <Eigen/Dense>

using namespace Eigen;

struct SatelliteFix
{
    double gps_time;
    long sample_index;
    double x_k;
    double y_k;
    double z_k;
};

enum TMsgType
{
    TYPE_ADD,
    TYPE_DEL
};


class TriangulateMessage
{
public:
    TMsgType type;
    int sat;
    TriangulateMessage(TMsgType type, int sat):
        type(type), sat(sat){}
};

class TriangulateAddMessage : public TriangulateMessage
{
public:
    SatelliteFix fix;
    TriangulateAddMessage(int sat, SatelliteFix &fix):
        TriangulateMessage(TYPE_ADD, sat), fix(fix){};
};

class TriangulateDelMessage : public TriangulateMessage
{
public:
    TriangulateDelMessage(int sat):
        TriangulateMessage(TYPE_DEL, sat){}
};

class Triangulator
{
    int fs; // sample rate
    std::thread thread;
    ThreadQueue<std::unique_ptr<TriangulateMessage>> queue;
    std::list<TriangulateAddMessage> collected_sats;
    SatelliteFix fixs[4];
    void thread_func(void);
    void add_sat(TriangulateAddMessage *tam);
    void del_sat(TriangulateDelMessage *tdm);
    Vector4d jacobian_vec(int f, Vector4d &X);
    Matrix4d jacobian(Vector4d &X); // X(x,y,z,bias)
    double func(int f, Vector4d &X);
    Vector4d dY(Vector4d &X);
    Vector4d X_guess(void);
    void gps_coordinates(Vector4d &X);
    void triangulate(void);
public:
    Triangulator(int fs);
    ~Triangulator(void);
    void send_add_message(int sat, SatelliteFix &fix);
    void send_del_message(int sat);
};


