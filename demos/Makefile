-include ../Makefile.include
CXXFLAGS += -I../src $(OSG_CXXFLAGS) $(ODE_CXXFLAGS) $(BT_CXXFLAGS)
LDFLAGS += $(OSG_LDFLAGS) $(ODE_LDFLAGS) $(BT_LDFLAGS)

LIBDEPS = ../src/libsim.a ../src/bullet/libsim-bullet.a \
          ../src/ode/libsim-ode.a


TARGETS = demo demo2 demo3

all: $(TARGETS)

demo: demo.o $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< -L../src/bullet -lsim-bullet -L../src -lsim $(LDFLAGS)
demo3: demo3.o $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< -L../src/ode -lsim-ode -L../src -lsim $(LDFLAGS)

demo%: demo%.o $(LIBDEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< -L../src/bullet -lsim-bullet -L../src -lsim $(LDFLAGS)

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.moc: %.hpp
	$(MOC) -p'.' $< > $@

clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)

.PHONY: all clean help
