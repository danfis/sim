-include ../Makefile.include
CXXFLAGS += -I. $(OSG_CXXFLAGS) $(ODE_CXXFLAGS) $(BT_CXXFLAGS)
LDFLAGS += $(OSG_LDFLAGS) $(ODE_LDFLAGS) $(BT_LDFLAGS)


TARGETS = libsim.a demo2
OBJS = visbody.o visworld.o body.o joint.o sim.o component.o

all: bullet $(TARGETS)

libsim.a: $(OBJS)
	ar cr $@ $^
	ranlib $@

demo: demo.o libsim.a
	$(CXX) $(CXXFLAGS) -o $@ $< -L. -lsim $(LDFLAGS)
demo2: demo2.o libsim.a bullet/libsim-bullet.a
	$(CXX) $(CXXFLAGS) -o $@ $< -Lbullet -lsim-bullet -L. -lsim $(LDFLAGS)

bullet/libsim-bullet.a:
	$(MAKE) bullet

bullet:
	$(MAKE) -C bullet

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
	$(MAKE) -C bullet clean
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;

check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

.PHONY: all clean help check check-valgrind bullet
