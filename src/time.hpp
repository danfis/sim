#ifndef _SIM_TIME_HPP_
#define _SIM_TIME_HPP_

#include <time.h>

namespace sim {

class Time {
  protected:
    struct timespec _t;

  public:
    Time() { _t.tv_sec = 0; _t.tv_nsec = 0; }
    Time(const struct timespec &t) : _t(t) {}
    Time(const Time &t) : _t(t._t) {}

    /* \{ */
    /**
     * Returns nanoseconds part.
     */
    unsigned long ns() const { return _t.tv_nsec % 1000000L; }

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
    Time diff(const Time &lower, const Time &higher) const;
    /* \} */


    /* \{ */
    /**
     * Returns elapsed time from beggining of current thread.
     */
    static Time clock()
        { Time t; clock(&t); return t; }

    /**
     * Fills given variable with time elapsed from beggining of current
     * thread.
     */
    static void clock(Time *t)
        { clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t->_t); }
    /* \} */

};

} /* namespace sim */

#endif /* _SIM_TIME_HPP_ */
