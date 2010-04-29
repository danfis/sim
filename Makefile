-include Makefile.include
CXXFLAGS += $(OSG_CXXFLAGS) $(ODE_CXXFLAGS) $(BT_CXXFLAGS)
LDFLAGS += $(OSG_LDFLAGS) $(ODE_LDFLAGS) $(BT_LDFLAGS)


TARGETS = libsim.a demo demo3
OBJS = sim.o object.o visobj.o visworld.o obj.o world.o joint.o

all: $(TARGETS)

libsim.a: $(OBJS)
	ar cr $@ $^
	ranlib $@

demo: demo.o libsim.a
	$(CXX) $(CXXFLAGS) -o $@ demo.o -L. -lsim $(LDFLAGS)

demo3: demo3.o libsim.a
	$(CXX) $(CXXFLAGS) -o $@ $< -L. -lsim $(LDFLAGS)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c -o $@ $<
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp %.hpp math.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.moc: %.hpp
	$(MOC) -p'.' $< > $@

clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;

check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

.PHONY: all clean help check check-valgrind
