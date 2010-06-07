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

-include ../../Makefile.local
-include ../../Makefile.include
CXXFLAGS = -g -Wall -pedantic
CXXFLAGS += -I./ -I../ -Icu/
CXXFLAGS += $(OSG_CXXFLAGS) $(ODE_CXXFLAGS) $(BT_CXXFLAGS)
LDFLAGS += -L../ode -lsim-ode
LDFLAGS += -L./ -Lcu/ -lcu -lm -L../ -lsim
LDFLAGS += $(OSG_LDFLAGS) $(ODE_LDFLAGS) $(BT_LDFLAGS)
LDFLAGS += -lrt

CHECK_REG=cu/check-regressions

OBJS = component.o message.o time.o


all: test

test: cu $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ main.cpp $(OBJS) $(LDFLAGS)

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

check: all
	@echo ""
	@echo "----------------------------------------";
	./test
	@echo "----------------------------------------";
	@echo "Checking regressions:";
	./$(CHECK_REG) regressions
	@echo ""

check-valgrind: all
	valgrind -q --leak-check=full --show-reachable=yes --trace-children=yes \
             --suppressions=opengl.supp \
             --suppressions=fglrx.supp \
             --suppressions=gen.supp \
             --gen-suppressions=all --log-file=out --error-limit=no \
             ./test

check-valgrind-gen-suppressions: all
	valgrind -q --leak-check=full --show-reachable=yes --trace-children=yes \
             --suppressions=opengl.supp \
             --suppressions=fglrx.supp \
             --suppressions=gen.supp \
             --gen-suppressions=all --log-file=out --error-limit=no \
             ./test

cu:
	$(MAKE) -C cu/

clean:
	rm -f *.o
	rm -f objs/*.o
	rm -f test
	rm -f tmp.*
	rm -f regressions/tmp.*

.PHONY: all clean check check-valgrind cu
	
