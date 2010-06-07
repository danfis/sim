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
CXXFLAGS += -I../src
CXXFLAGS_BT = $(CXXFLAGS) $(OSG_CXXFLAGS) $(BT_CXXFLAGS)
CXXFLAGS_ODE = $(CXXFLAGS) $(OSG_CXXFLAGS) $(ODE_CXXFLAGS)

LDFLAGS +=  -L../src -lsim
LDFLAGS_BT = -L../src/bullet -lsim-bullet $(LDFLAGS) $(OSG_LDFLAGS) $(BT_LDFLAGS)
LDFLAGS_ODE = -L../src/ode -lsim-ode $(LDFLAGS) $(OSG_LDFLAGS) $(ODE_LDFLAGS)

LIBDEPS_BT = ../src/libsim.a ../src/bullet/libsim-bullet.a
LIBDEPS_ODE = ../src/libsim.a ../src/ode/libsim-ode.a

OBJS_SSSA = demo_sssa.cpp robot_sssa.cpp

TARGETS =

ifeq '$(WANT_BT)' 'yes'
  TARGETS += demo_base_bt
endif

ifeq '$(WANT_ODE)' 'yes'
  TARGETS += demo_base_ode
  TARGETS += demo_syrotek
  TARGETS += demo_follow_color
#  TARGETS += main
  TARGETS += demo_intersection
  TARGETS += demo_chameleon
  TARGETS += demo_sssa
  TARGETS += demo_movement_tunning
  TARGETS += demo_pso
endif

all: $(TARGETS)


demo_base_bt: demo_base_bt.cpp $(LIBDEPS_BT)
	$(CXX) $(CXXFLAGS_BT) -o $@ $< $(LDFLAGS_BT)

demo2: demo2.cpp $(LIBDEPS_BT)
	$(CXX) $(CXXFLAGS_BT) -o $@ $< $(LDFLAGS_BT)

main: main.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)

demo_base_ode: demo_base_ode.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)

demo_syrotek: demo_syrotek.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) $(SDL_CXXFLAGS) -o $@ $< $(LDFLAGS_ODE) $(SDL_LDFLAGS)

demo_follow_color: demo_follow_color.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) $(SDL_CXXFLAGS) -o $@ $< $(LDFLAGS_ODE) $(SDL_LDFLAGS)

demo_intersection: demo_intersection.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)

demo_chameleon: demo_chameleon.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) $(SDL_CXXFLAGS) -o $@ $< $(LDFLAGS_ODE) $(SDL_LDFLAGS)

demo_sssa: $(OBJS_SSSA) $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) $(SDL_CXXFLAGS) -o $@ $(OBJS_SSSA) $(LDFLAGS_ODE) $(SDL_LDFLAGS)

demo_movement_tunning: demo_movement_tunning.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)

demo_pso: demo_pso.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)

clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)

.PHONY: all clean help
