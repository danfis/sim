#include <iostream>

#include "cu.h"
#include "sim/time.hpp"

using namespace std;
using sim::Time;

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
}

