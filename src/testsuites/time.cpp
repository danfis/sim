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

#include <iostream>

#include "cu.h"
#include "sim/time.hpp"

using namespace std;
using sim::Time;
using sim::Timer;

TEST(timeSetUp)
{
}

TEST(timeTearDown)
{
}

TEST(timeCmp)
{
    assertTrue(Time(0, 0) < Time(1, 0));
    assertTrue(Time(0, 0) <= Time(1, 0));
    assertTrue(Time(0, 0) < Time(1, 0));
    assertTrue(Time(0, 99999999) < Time(1, 0));
    assertTrue(Time(0, 99999999) <= Time(1, 0));
    assertTrue(Time(1, 12) > Time(1, 11));
    assertTrue(Time(1, 12) >= Time(1, 11));
    assertTrue(Time(1, 1) == Time(1, 1));
}

TEST(timeDiff)
{
    Time tdiff;

    tdiff = Time::diff(Time(0, 0), Time(0, 0));
    assertEquals(tdiff.inNs(), 0);
    Time::diff(Time(0, 0), Time(0, 0), &tdiff);
    assertEquals(tdiff.inNs(), 0);

    tdiff = Time::diff(Time(0, 0), Time(0, 1));
    assertEquals(tdiff.inNs(), 1);

    tdiff = Time::diff(Time(0, 0), Time(1, 12345678));
    assertEquals(tdiff.inNs(), 12345678L + 1000000000L);
    assertEquals(tdiff.ns(), 678);
    assertEquals(tdiff.us(), 345);
    assertEquals(tdiff.ms(), 12);
    assertEquals(tdiff.s(), 1);
    assertEquals(tdiff.m(), 0);
    assertEquals(tdiff.h(), 0);

    tdiff = Time::diff(Time(2, 30000000), Time(3, 10000000));
    assertEquals(tdiff.inNs(), 980000000);
    assertEquals(tdiff.ns(), 0);
    assertEquals(tdiff.us(), 0);
    assertEquals(tdiff.ms(), 980);
    assertEquals(tdiff.s(), 0);
    assertEquals(tdiff.m(), 0);
    assertEquals(tdiff.h(), 0);

    tdiff = Time::diff(Time(1, 123), Time(2, 124));
    assertEquals(tdiff.inNs(), 1000000001);
    assertEquals(tdiff.ns(), 1);
    assertEquals(tdiff.us(), 0);
    assertEquals(tdiff.ms(), 0);
    assertEquals(tdiff.s(), 1);
    assertEquals(tdiff.m(), 0);
    assertEquals(tdiff.h(), 0);

    tdiff = Time::diff(Time(1, 123), Time(2 + 3 * 60 + 1 * 3600, 124));
    assertEquals(tdiff.ns(), 1);
    assertEquals(tdiff.us(), 0);
    assertEquals(tdiff.ms(), 0);
    assertEquals(tdiff.s(), 1);
    assertEquals(tdiff.m(), 3);
    assertEquals(tdiff.h(), 1);
    assertEquals(tdiff.inH(), 1);
    assertEquals(tdiff.inM(), 63);
    assertEquals(tdiff.inS(), 3600 + 3 * 60 + 1);
}

TEST(timeClock)
{
    Time t1, t2, t3;
   
    t1 = Time::cur();
    usleep(100);
    t2 = Time::cur();

    t3 = Time::diff(t1, t2);
    assertTrue(t3.h() == 0);
    assertTrue(t3.m() == 0);
    assertTrue(t3.s() == 0);
    assertTrue(t3.ms() == 0);
    assertTrue(t3.us() >= 100);
    assertTrue(t3.inUs() >= 100);

    Timer timer;

    for (size_t i = 0; i < 3; i++){
        timer.start();
        assertTrue(timer.elapsedTime().inNs() == 0);
        usleep(100);
        t2 = timer.stop();
        assertTrue(t2.inUs() >= 100);
        assertTrue(timer.elapsedTime().inUs() >= 100);

        usleep(100);
        t2 = timer.stop();
        assertTrue(t2.inUs() >= 200);
        assertTrue(timer.elapsedTime().inUs() >= 200);

        usleep(500);
        t2 = timer.stop();
        assertTrue(t2.inUs() >= 700);
        assertTrue(timer.elapsedTime().inUs() >= 700);
    }
}

TEST(timeFrom)
{
    Time t;

    t = Time::fromMs(1000);
    assertTrue(t.inMs() == 1000);
    assertTrue(t.inS() == 1);

    t = Time::fromMs(5010);
    assertEquals(t.inMs(), 5010);
    assertEquals(t.inS(), 5);
    assertEquals(t.ms(), 10);
    assertEquals(t.s(), 5);

    t = Time(1, 12);
    t += Time(1, 12);
    assertEquals(t.ns(), 24);
    assertEquals(t.s(), 2);

    t = Time::fromMs(2300);
    t += Time::fromMs(12345);
    assertEquals(t.inMs(), 14645);
    assertEquals(t.inS(), 14);
}
