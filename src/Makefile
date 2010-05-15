-include ../Makefile.include
CXXFLAGS += -I. $(OSG_CXXFLAGS) $(BT_CXXFLAGS)


TARGETS = libsim.a
OBJS = visbody.o visworld.o body.o joint.o sim.o component.o message.o \
       time.o visworldmanip.o

all: bullet $(TARGETS)

libsim.a: $(OBJS)
	ar cr $@ $^
	ranlib $@

bullet:
	$(MAKE) -C bullet

%.o: %.cpp %.hpp math.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.moc: %.hpp
	$(MOC) -p'.' $< > $@

clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)
	$(MAKE) -C bullet clean
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;

check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

.PHONY: all clean help check check-valgrind bullet
