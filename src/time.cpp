#include "time.hpp"
#include "msg.hpp"

namespace sim {

Time Time::diff(const Time &l, const Time &h) const
{
    struct timespec t;

    if (h._t.tv_nsec > l._t.tv_nsec){
        t.tv_nsec = h._t.tv_nsec - l._t.tv_nsec;
        t.tv_sec = h._t.tv_sec - l._t.tv_sec;
    }else{
        t.tv_nsec = h._t.tv_nsec + 1000000000L - l._t.tv_nsec;
        t.tv_sec = h._t.tv_sec - 1 - l._t.tv_sec;
    }

    return Time(t);
}
}
