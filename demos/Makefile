-include ../Makefile.local
-include ../Makefile.include
CXXFLAGS += -I../src
CXXFLAGS_BT = $(CXXFLAGS) $(OSG_CXXFLAGS) $(BT_CXXFLAGS)
CXXFLAGS_ODE = $(CXXFLAGS) $(OSG_CXXFLAGS) $(ODE_CXXFLAGS)

LDFLAGS +=  -L../src -lsim
LDFLAGS_BT = -L../src/bullet -lsim-bullet $(LDFLAGS) $(OSG_LDFLAGS) $(BT_LDFLAGS)
LDFLAGS_ODE = -L../src/ode -lsim-ode $(LDFLAGS) $(OSG_LDFLAGS) $(ODE_LDFLAGS)

LIBDEPS_BT = ../src/libsim.a ../src/bullet/libsim-bullet.a
LIBDEPS_ODE = ../src/libsim.a ../src/ode/libsim-ode.a

OBJS_SYROTEK = demo_syrotek.cpp robot_syrotek.cpp
OBJS_FOLLOW_COLOR = demo_follow_color.cpp robot_syrotek.cpp

TARGETS =

ifeq '$(WANT_BT)' 'yes'
  TARGETS += demo_base_bt
endif

ifeq '$(WANT_ODE)' 'yes'
  TARGETS += demo_base_ode
  TARGETS += demo_syrotek
  TARGETS += demo_follow_color
  TARGETS += main
endif

all: $(TARGETS)


demo_base_bt: demo_base_bt.cpp $(LIBDEPS_BT)
	$(CXX) $(CXXFLAGS_BT) -o $@ $< $(LDFLAGS_BT)

demo2: demo2.cpp $(LIBDEPS_BT)
	$(CXX) $(CXXFLAGS_BT) -o $@ $< $(LDFLAGS_BT)

main: main.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)

demo_base_ode: demo_base_ode.cpp $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) -o $@ $< $(LDFLAGS_ODE)

demo_syrotek: $(OBJS_SYROTEK) $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) $(SDL_CXXFLAGS) -o $@ $(OBJS_SYROTEK) $(LDFLAGS_ODE) $(SDL_LDFLAGS) -lSDL
demo_follow_color: $(OBJS_FOLLOW_COLOR) $(LIBDEPS_ODE)
	$(CXX) $(CXXFLAGS_ODE) $(SDL_CXXFLAGS) -o $@ $(OBJS_FOLLOW_COLOR) $(LDFLAGS_ODE) $(SDL_LDFLAGS) -lSDL


clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)

.PHONY: all clean help
