-include ../Makefile.local
-include ../Makefile.include
CXXFLAGS += -I../src
CXXFLAGS_BT = $(CXXFLAGS) $(OSG_CXXFLAGS) $(BT_CXXFLAGS)
CXXFLAGS_ODE = $(CXXFLAGS) $(OSG_CXXFLAGS) $(ODE_CXXFLAGS)

LDFLAGS += -L../src -lsim
LDFLAGS_BT = -L../src/bullet -lsim-bullet $(LDFLAGS) $(OSG_LDFLAGS) $(BT_LDFLAGS)
LDFLAGS_ODE = -L../src/ode -lsim-ode $(LDFLAGS) $(OSG_LDFLAGS) $(ODE_LDFLAGS)

LIBDEPS_BT = ../src/libsim.a ../src/bullet/libsim-bullet.a
LIBDEPS_ODE = ../src/libsim.a ../src/ode/libsim-ode.a


TARGETS =

ifeq '$(WANT_BT)' 'yes'
  TARGETS += demo_base_bt
endif

ifeq '$(WANT_ODE)' 'yes'
  TARGETS += demo_base_ode
endif

all: $(TARGETS)

demo_base_bt: demo_base_bt.cpp $(LIBDEPS_BT)
	$(CXX) $(CXXFLAGS_BT) -o $@ $< $(LDFLAGS_BT)

demo2: demo2.cpp $(LIBDEPS_BT)
	$(CXX) $(CXXFLAGS_BT) -o $@ $< $(LDFLAGS_BT)

demo_base_ode: demo_base_ode.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)


clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)

.PHONY: all clean help
