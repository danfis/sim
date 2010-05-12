#include <iomanip>

#include "time.hpp"
#include "msg.hpp"

namespace sim {

void Time::diff(const Time &l, const Time &h, Time *diff)
{
    struct timespec t;

    if (h._t.tv_nsec > l._t.tv_nsec){
        t.tv_nsec = h._t.tv_nsec - l._t.tv_nsec;
        t.tv_sec = h._t.tv_sec - l._t.tv_sec;
    }else{
        t.tv_nsec = h._t.tv_nsec + 1000000000L - l._t.tv_nsec;
        t.tv_sec = h._t.tv_sec - 1 - l._t.tv_sec;
    }
    *diff = t;
}

std::ostream& operator<<(std::ostream &out, const Time &t)
{
    out << t.h() << ":" << t.m() << ":" << t.s()
        << "." << std::setfill('0') << std::setw(3) << t.ms()
        << "." << std::setfill('0') << std::setw(2) << t.us()
        << "." << std::setfill('0') << std::setw(3) << t.ns();
    return out;
}

}
