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

ifeq '$(HAVE_ODE)' 'yes'
  LIBDEPS += ../src/ode/libsim-ode.a
  LDFLAGS += -L../src/ode -lsim-ode
endif
ifeq '$(HAVE_BT)' 'yes'
  LIBDEPS += ../src/bullet/libsim-bullet.a
  LDFLAGS += -L../src/bullet -lsim-bullet
endif
LIBDEPS += ../src/libsim.a
LDFLAGS += -L../src -lsim

TARGETS =

TARGETS += demo_base

ifeq '$(WANT_ODE)' 'yes'
  TARGETS += demo_syrotek
  TARGETS += demo_follow_color
#  TARGETS += main
  TARGETS += demo_intersection
  TARGETS += demo_chameleon
  TARGETS += demo_sssa
  TARGETS += demo_sssa_gen
  TARGETS += demo_movement_tunning
  TARGETS += demo_pso
endif

all: $(TARGETS)


demo_base: demo_base.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo2: demo2.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGST)

main: main.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_base_ode: demo_base_ode.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_syrotek: demo_syrotek.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_follow_color: demo_follow_color.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_intersection: demo_intersection.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_chameleon: demo_chameleon.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_sssa: demo_sssa.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_sssa_gen: demo_sssa_gen.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_movement_tunning: demo_movement_tunning.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

demo_pso: demo_pso.cpp $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)

.PHONY: all clean help
