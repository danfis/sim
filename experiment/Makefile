###
# sim
# ---------------------------------
# Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
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

LIBDEPS += ../src/libsim.a
LDFLAGS := -L../src -lsim $(LDFLAGS)
ifeq '$(HAVE_ODE)' 'yes'
  LIBDEPS += ../src/ode/libsim-ode.a
  LDFLAGS := -L../src/ode -lsim-ode $(LDFLAGS)
endif
ifeq '$(HAVE_BT)' 'yes'
  LIBDEPS += ../src/bullet/libsim-bullet.a
  LDFLAGS := -L../src/bullet -lsim-bullet $(LDFLAGS)
endif

# GSL
CXXFLAGS += $(shell pkg-config gsl --cflags)
LDFLAGS += $(shell pkg-config gsl --libs)

CXXFLAGS += -Iblobf
LDFLAGS += -Lblobf -lblobf

TARGETS = exp
OBJS = exp.o robot.o arena.o sim.o

all: $(TARGETS)

exp: $(OBJS) blobf/libblobf.a
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

blobf/libblobf.a:
	$(MAKE) -C blobf

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.c
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)
	$(MAKE) -C blobf clean

.PHONY: all clean help
