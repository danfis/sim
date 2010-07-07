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
        << "." << std::setfill('0') << std::setw(3) << t.us()
        << "." << std::setfill('0') << std::setw(3) << t.ns();
    return out;
}

}
