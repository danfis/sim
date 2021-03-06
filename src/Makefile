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
CXXFLAGS += -I.
CXXFLAGS += $(OSG_CXXFLAGS)
CXXFLAGS += $(SDL_CXXFLAGS)

CONFIG_FLAGS =
ifeq '$(HAVE_ODE)' 'yes'
  CONFIG_FLAGS += -DHAVE_ODE=1
endif
ifeq '$(HAVE_BT)' 'yes'
  CONFIG_FLAGS += -DHAVE_BULLET=1
endif
ifeq '$(HAVE_SDL)' 'yes'
  CONFIG_FLAGS += -DHAVE_SDL=1
endif
ifeq '$(HAVE_PHYSX)' 'yes'
  CONFIG_FLAGS += -DHAVE_PHYSX=1
endif

SUBTARGETS =

ifeq '$(HAVE_ODE)' 'yes'
  SUBTARGETS += ode
endif

ifeq '$(HAVE_BT)' 'yes'
  SUBTARGETS += bullet
endif

TARGETS = libsim.a config.hpp
OBJS = visbody.o visworld.o body.o joint.o sim.o component.o message.o \
       time.o visworldmanip.o world.o
OBJS += sensor/camera.o sensor/rangefinder.o
OBJS += comp/povray.o comp/snake.o comp/frequency.o comp/watchdog.o \
        comp/syrotek.o comp/joystick.o comp/sssa.o comp/blender.o \
        comp/povray_full.o comp/povray_step.o \
        comp/snake2.o comp/rserver.o
OBJS += robot/syrotek.o robot/sssa.o

ifeq '$(HAVE_OPENCV)' 'yes'
  CXXFLAGS += $(OPENCV_CXXFLAGS)
  OBJS += alg/surfnav.o
#  OBJS += ext/OpenSURF/fasthessian.o ext/OpenSURF/integral.o \
#          ext/OpenSURF/ipoint.o ext/OpenSURF/utils.o \
#          ext/OpenSURF/surf.o
  OBJS += alg/surf/fasthessian.o alg/surf/integral.o \
          alg/surf/utils.o alg/surf/surf.o
endif

all: $(TARGETS) $(SUBTARGETS)

libsim.a: $(OBJS) config.hpp
	ar cr $@ $(OBJS)
	ranlib $@

config.hpp: config.hpp.in
	$(M4) $(CONFIG_FLAGS) $< > $@

ode: libsim.a
	$(MAKE) -C ode
bullet: libsim.a
	$(MAKE) -C bullet

%.o: %.cpp %.hpp config.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp config.hpp
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
