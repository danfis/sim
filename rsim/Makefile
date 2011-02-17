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
CFLAGS += -I.
LDFLAGS += -L. -lrsim

SUBTARGETS =

TARGETS = librsim.a test-server test-client
OBJS = rsim_server.o alloc.o rsim.o common.o

all: $(TARGETS) $(SUBTARGETS)

test-server: test-server.c librsim.a
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
test-client: test-client.c librsim.a
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

librsim.a: $(OBJS)
	ar cr $@ $(OBJS)
	ranlib $@

%.o: %.c %.h
	$(CC) $(CFLAGS) -c -o $@ $<
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.o *~ *.a
	rm -f $(OBJS)
	rm -f $(TARGETS)
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;

check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

.PHONY: all clean help check check-valgrind bullet ode
