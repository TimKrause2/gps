#ifndef TIMER_H
#define TIMER_H

#include <signal.h>
#include <time.h>
#include <functional>

struct Timer
{
private:
    std::function<void(void)> cb;
    static void notify_function(union sigval val);
    timer_t timer_id;
public:
    Timer();
    void set_callback(std::function<void(void)> cb);
    void create();
    void set_time(double interval, double value);
};

#endif // TIMER_H
