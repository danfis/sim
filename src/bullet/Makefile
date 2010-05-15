-include ../../Makefile.include
CXXFLAGS += -I../ $(OSG_CXXFLAGS) $(BT_CXXFLAGS)


TARGETS = libsim-bullet.a
OBJS = body.o joint.o actuator.o world.o collision_detection.o sim.o

all: $(TARGETS)

libsim-bullet.a: $(OBJS)
	ar cr $@ $^
	ranlib $@

%.o: %.cpp %.hpp
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
