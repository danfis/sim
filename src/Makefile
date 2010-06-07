###
# sim
# ---------------------------------
# Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
#
#  This file is part of sim.
#
#  sim is free software; you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as
#  published by the Free Software Foundation; either version 3 of
#  the License, or (at your option) any later version.
#
#  sim is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

-include ../Makefile.local
-include ../Makefile.include
CXXFLAGS += -I. $(OSG_CXXFLAGS)

ifeq '$(HAVE_SDL)' 'yes'
  CXXFLAGS += $(SDL_CXXFLAGS)
endif

SUBTARGETS =

ifeq '$(WANT_ODE)' 'yes'
  SUBTARGETS += ode
endif

ifeq '$(WANT_BT)' 'yes'
  SUBTARGETS += bullet
endif

TARGETS = libsim.a
OBJS = visbody.o visworld.o body.o joint.o sim.o component.o message.o \
       time.o visworldmanip.o
OBJS += sensor/camera.o sensor/rangefinder.o
OBJS += comp/povray.o comp/snake.o comp/frequency.o comp/watchdog.o \
        comp/syrotek.o comp/joystick.o
OBJS += robot/syrotek.o robot/sssa.o

all: $(TARGETS) $(SUBTARGETS)

libsim.a: $(OBJS)
	ar cr $@ $^
	ranlib $@

ode: libsim.a
	$(MAKE) -C ode
bullet: libsim.a
	$(MAKE) -C bullet

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f *.o *~ *.a
	rm -f $(OBJS)
	rm -f $(TARGETS)
	$(MAKE) -C bullet clean
	$(MAKE) -C ode clean
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;

check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

.PHONY: all clean help check check-valgrind bullet ode
