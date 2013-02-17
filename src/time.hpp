/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_TIME_HPP_
#define _SIM_TIME_HPP_

#include <time.h>
#include <iostream>

#include <sys/time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

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
    {
#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
        struct timespec ts;
        clock_serv_t cclock;
        mach_timespec_t mts;
        host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
        clock_get_time(cclock, &mts);
        mach_port_deallocate(mach_task_self(), cclock);
        ts.tv_sec = mts.tv_sec;
        ts.tv_nsec = mts.tv_nsec;
        t->_t = ts;
#else
        clock_gettime(CLOCK_MONOTONIC, &t->_t);
#endif
    }
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
    bool _paused;

  public:
    Timer() {}

    const Time &startTime() const { return _start; }
    const Time &stopTime() const { return _stop; }
    const Time &elapsedTime() const { return _elapsed; }

    /**
     * Starts timer. Returns start time.
     */
    inline const Time &start()
        { _paused = false; Time::cur(&_start);
          _stop = _start; _elapsed.setInMs(0); return _start; }

    /**
     * Stops timer. Returns elapsed time from start.
     * This method can be called repeatedly and time will be meassured
     * always from last call of start().
     */
    inline const Time &stop()
        { if (_paused) return _elapsed;
          Time::cur(&_stop); Time::diff(_start, _stop, &_elapsed);
          return _elapsed; }

    /**
     * Pauses timer.
     */
    inline void pause()
        { if (_paused) return;
          _paused = true; stop(); }

    /**
     * Unpauses timer and returns elapsed time.
     */
    inline const Time &unpause()
        { if (!_paused) return stop();
          _paused = false;
          Time diff; Time::diff(_stop, Time::cur(), &diff);
          _start += diff; return stop(); }
};

} /* namespace sim */

#endif /* _SIM_TIME_HPP_ */
