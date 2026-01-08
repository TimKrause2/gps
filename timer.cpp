#include "timer.h"

#include "timer.h"
#include <stdio.h>
#include <math.h>

Timer::Timer()
{

}

void Timer::notify_function(union sigval val)
{
    Timer *timer = (Timer*)val.sival_ptr;
    if(timer->cb)
        timer->cb();
}

void Timer::set_callback(std::function<void(void)> cb)
{
    Timer::cb = cb;
}

void Timer::create(void)
{
    struct sigevent l_sigevent;
    l_sigevent.sigev_notify = SIGEV_THREAD;
    l_sigevent.sigev_value.sival_ptr = (void*)this;
    l_sigevent.sigev_notify_function = notify_function;
    l_sigevent.sigev_notify_attributes = NULL;
    int l_result;
    l_result = timer_create(CLOCK_REALTIME,&l_sigevent,&timer_id);
    if(l_result==-1){
        perror("timer_create");
        exit(0);
    }
}

void Timer::set_time(double interval, double value)
{
    double interval_int;
    double interval_frac = modf(interval, &interval_int);
    double value_int;
    double value_frac = modf(value, &value_int);

    struct itimerspec l_ispec;
    l_ispec.it_interval.tv_sec = interval_int;
    l_ispec.it_interval.tv_nsec = (long)(interval_frac*1e9);
    l_ispec.it_value.tv_sec = value_int;
    l_ispec.it_value.tv_nsec = (long)(value_frac*1e9);
    int l_result;
    l_result = timer_settime(timer_id,0,&l_ispec,NULL);
    if(l_result==-1){
        perror("timer_settime");
        exit(0);
    }
}
