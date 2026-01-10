#ifndef SENSORS_H
#define SENSORS_H

#include <complex>
#include <thread>
#include <memory>
#include "queue.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>

#define N_SATELLITES 32

struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};


class Constellation
{
    int N_points;
    ScrollingBuffer sbuff;
    std::string plot_name;
public:
    Constellation(int sat, int N_points);
    ~Constellation();
    void data_point(std::complex<float> x);
    void tab_item(void);
};

class Frequency
{

};

enum SensorMsgType
{
    SMT_ADD,
    SMT_DEL,
    SMT_DATA
};

struct SensorMsg
{
    SensorMsgType type;
    SensorMsg(SensorMsgType type):
        type(type) {}
};

struct SensorMsgSat : SensorMsg
{
    int sat;
    SensorMsgSat(SensorMsgType type, int sat):
        SensorMsg(type), sat(sat){}
};

struct SensorMsgAdd : SensorMsgSat
{
    SensorMsgAdd(int sat):
        SensorMsgSat(SMT_ADD, sat){}
};

struct SensorMsgDel : SensorMsgSat
{
    SensorMsgDel(int sat):
        SensorMsgSat(SMT_DEL, sat){}
};

struct SensorMsgData : SensorMsgSat
{
    int N_data;
    std::unique_ptr<std::complex<float>[]> iq;
    SensorMsgData(int sat,
                  int N_data,
                  std::unique_ptr<std::complex<float>[]> iq)
        : SensorMsgSat(SMT_DATA, sat),
        N_data(N_data), iq(std::move(iq))
    {
    }
};


struct SensorSlot
{
    Constellation constellation;
    SensorSlot(int sat, int N_points):
        constellation(sat, N_points){}
};

class Sensors
{
    int sat_slot[N_SATELLITES];
    std::unique_ptr<SensorSlot> slots[N_SATELLITES];
    ThreadQueue<std::unique_ptr<SensorMsg>> queue;
    std::thread thread;
    bool thread_enabled;
    void thread_func(void);
    void sat_tab_items(void);
    void sats_update(void);
    void add_sat(SensorMsgAdd *msg);
    void del_sat(SensorMsgDel *msg);
    void sat_data(SensorMsgData *msg);
public:
    Sensors(void);
    ~Sensors(void);
    void send_add_sat(int sat);
    void send_del_sat(int sat);
    void send_sat_data(int sat, int N_data, std::unique_ptr<std::complex<float>[]> iq);
};

#endif // SENSORS_H
