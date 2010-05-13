#ifndef _SIM_TIME_HPP_
#define _SIM_TIME_HPP_

#include <time.h>
#include <iostream>

namespace sim {

class Time {
  protected:
    struct timespec _t;

  public:
    Time() { _t.tv_sec = 0; _t.tv_nsec = 0; }
    Time(const struct timespec &t) : _t(t) {}
    Time(const Time &t) : _t(t._t) {}
    Time(long sec, long nsec) { _t.tv_sec = sec; _t.tv_nsec = nsec; }

    /* \{ */
    Time &operator=(const Time &t)
        { _t.tv_sec = t._t.tv_sec; _t.tv_nsec = t._t.tv_nsec; return *this; }
    Time &operator=(const struct timespec &t)
        { _t.tv_sec = t.tv_sec; _t.tv_nsec = t.tv_nsec; return *this; }
    Time &operator+=(const Time &t)
        { _t.tv_sec += t._t.tv_sec; _t.tv_nsec += t._t.tv_nsec;
          _t.tv_sec += _t.tv_nsec / 1000000000L; _t.tv_nsec %= 1000000000L;
          return *this; }
    Time &operator/=(unsigned int f)
        { double t = (double)_t.tv_sec + (double)_t.tv_nsec / 1000000000.;
          t /= (double)f;
          _t.tv_sec = (time_t)t;
          _t.tv_nsec = (long)(t * 1000000000.) % 1000000000L;
          return *this; }

    Time &setInMs(unsigned long ms)
        { _t.tv_sec = ms / 1000L; _t.tv_nsec = (ms % 1000L) * 1000000;
          return *this; }
    /* \} */


    /* \{ */
    bool operator>(const Time &t) const
        { return _t.tv_sec > t._t.tv_sec
            || (_t.tv_sec == t._t.tv_sec && _t.tv_nsec > t._t.tv_nsec); }
    bool operator<(const Time &t) const
        { return _t.tv_sec < t._t.tv_sec
            || (_t.tv_sec == t._t.tv_sec && _t.tv_nsec < t._t.tv_nsec); }
    bool operator==(const Time &t) const
        { return _t.tv_sec == t._t.tv_sec && _t.tv_nsec == t._t.tv_nsec; }
    bool operator>=(const Time &t) const { return *this == t || *this > t; }
    bool operator<=(const Time &t) const { return *this == t || *this < t; }
    /* \} */

    /* \{ */
    /**
     * Returns nanoseconds part.
     */
    unsigned long ns() const { return _t.tv_nsec % 1000L; }

    /**
     * Returns micro seconds part.
     */
    unsigned int us() const { return (_t.tv_nsec / 1000L) % 1000L; }

    /**
     * Returns miliseconds part.
     */
    unsigned int ms() const { return _t.tv_nsec / 1000000L; }

    /**
     * Returns seconds part.
     */
    unsigned int s() const { return _t.tv_sec % 60L; }

    /**
     * Returns minutes part.
     */
    unsigned int m() const { return (_t.tv_sec / 60L) % 60L; }

    /**
     * Returns hours part.
     */
    unsigned int h() const { return _t.tv_sec / 3600L; }

    /**
     * Returns time in ns.
     */
    unsigned long inNs() const { return _t.tv_nsec + _t.tv_sec * 1000000000L; }

    /**
     * Returns time in us.
     */
    unsigned long inUs() const
        { return _t.tv_nsec / 1000L + _t.tv_sec * 1000000L; }

    /**
     * Returns time in ms.
     */
    unsigned long inMs() const
        { return _t.tv_nsec / 1000000L + _t.tv_sec * 1000L; }

    /**
     * Returns time in s.
     */
    unsigned long inS() const { return _t.tv_sec; }
    double inSF() const
        { return (double)_t.tv_sec + ((double)_t.tv_nsec / 1000000000.); }

    /**
     * Returns time in m.
     */
    unsigned long inM() const { return _t.tv_sec / 60L; }

    /**
     * Returns time in h.
     */
    unsigned long inH() const { return _t.tv_sec / 3600L; }
    /* \} */


    /* \{ */
    /**
     * Returns difference between times. It is callers responsibility to
     * place lower and higher time properly.
     */
    static Time diff(const Time &lower, const Time &higher)
        { Time t; diff(lower, higher, &t); return t; }
    static void diff(const Time &lower, const Time &higher, Time *diff);
    /* \} */


    /* \{ */
    /**
     * Returns current time
     */
    static Time cur()
        { Time t; cur(&t); return t; }
    static void cur(Time *t)
        { clock_gettime(CLOCK_MONOTONIC, &t->_t); }
    /* \} */

    /* \{ */
    /**
     * Creates Time that equals to number of specified ms.
     */
    static Time fromMs(unsigned long ms)
        { Time t; t.setInMs(ms); return t; }
    /* \} */

    static int sleep(const Time &t)
        { return nanosleep(&t._t, 0); }
};


std::ostream& operator<<(std::ostream &out, const Time &t);


class Timer {
    Time _start, _stop, _elapsed;

  public:
    Timer() {}

    const Time &startTime() const { return _start; }
    const Time &stopTime() const { return _stop; }
    const Time &elapsedTime() const { return _elapsed; }

    /**
     * Starts timer. Returns start time.
     */
    const Time &start()
        { Time::cur(&_start); _stop = _start; _elapsed.setInMs(0); return _start; }

    /**
     * Stops timer. Returns elapsed time from start.
     * This method can be called repeatedly and time will be meassured
     * always from last call of start().
     */
    const Time &stop()
        { Time::cur(&_stop); Time::diff(_start, _stop, &_elapsed);
          return _elapsed; }
};

} /* namespace sim */

#endif /* _SIM_TIME_HPP_ */
