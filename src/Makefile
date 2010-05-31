-include ../Makefile.local
-include ../Makefile.include
CXXFLAGS += -I. $(OSG_CXXFLAGS)

ifeq '$(HAVE_SDL)' 'yes'
  CXXFLAGS += $(SDL_CXXFLAGS)
endif

SUBTARGETS =

ifeq '$(WANT_ODE)' 'yes'
  SUBTARGETS += ode
endif

ifeq '$(WANT_BT)' 'yes'
  SUBTARGETS += bullet
endif

TARGETS = libsim.a
OBJS = visbody.o visworld.o body.o joint.o sim.o component.o message.o \
       time.o visworldmanip.o
OBJS += sensor/camera.o sensor/rangefinder.o
OBJS += comp/povray.o
OBJS += robot/syrotek.o robot/sssa.o

ifeq '$(HAVE_SDL)' 'yes'
  OBJS += comp/joystick.o
endif

all: $(TARGETS) $(SUBTARGETS)

libsim.a: $(OBJS)
	ar cr $@ $^
	ranlib $@

ode: libsim.a
	$(MAKE) -C ode
bullet: libsim.a
	$(MAKE) -C bullet

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f *.o *~ *.a
	rm -f $(OBJS)
	rm -f $(TARGETS)
	$(MAKE) -C bullet clean
	$(MAKE) -C ode clean
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;

check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

.PHONY: all clean help check check-valgrind bullet ode
