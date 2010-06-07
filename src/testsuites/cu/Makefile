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

CC = gcc
CFLAGS = -g -Wall -pedantic

TARGETS = libcu.a

TEST_OBJS = test.o test2.o

all: $(TARGETS)

libcu.a: cu.o
	ar cr $@ $^
	ranlib $@
cu.o: cu.c cu.h
	$(CC) $(CFLAGS) -c -o $@ $<

test: $(TEST_OBJS) libcu.a
	$(CC) $(CFLAGS) -o $@ $(TEST_OBJS) -L./ -lcu
test-segfault: test-segfault.c libcu.a
	$(CC) $(CFLAGS) -o $@ $^ -L./ -lcu

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

check: test test-segfault
	mkdir -p regressions
	touch regressions/testSuiteName{,2}.{out,err}
	touch regressions/testSuiteTest2.{out,err}
	-./test
	-cd regressions && ../check-regressions
	@echo ""
	@echo "======= SEGFAULT: ========="
	@echo ""
	-./test-segfault

clean:
	rm -f *.o
	rm -f test
	rm -f test-segfault
	rm -f $(TARGETS)
	rm -f tmp.*
	rm -rf regressions

.PHONY: all clean check
